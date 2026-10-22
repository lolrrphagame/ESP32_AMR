// ESP32-S3 AMR - Phase 2: dual quadrature encoder PCNT + retained UART ACK test
// ESP-IDF v6.1 | This firmware NEVER commands motors to run.
// Encoder left:  A GPIO4, B GPIO5. Right: A GPIO6, B GPIO7.
// UART1: TX GPIO17, RX GPIO18 @ 57600 8N1. Pi test: T 000001\r -> ACK 000001\r\n
// IMPORTANT: Keep BTS7960 motor power DISCONNECTED while testing encoders.
// Use external 10k pull-down resistors on driver EN/PWM inputs during reset.

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_PORT          UART_NUM_1
#define UART_TX_GPIO       17
#define UART_RX_GPIO       18
#define UART_BAUD          57600
#define UART_FRAME_MAX     32
#define PCNT_LIMIT         30000

static const char *TAG = "AMR_ENC";
static pcnt_unit_handle_t encoder_units[2];

static void set_motor_control_pins_low(void)
{
    // Force all RPWM/LPWM/R_EN/L_EN low during this encoder-only trial.
    const gpio_config_t cfg = {
        .pin_bit_mask = ((1ULL << 9) | (1ULL << 10) | (1ULL << 11) |
                         (1ULL << 12) | (1ULL << 13) | (1ULL << 14) |
                         (1ULL << 15) | (1ULL << 16)),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    for (int pin = 9; pin <= 16; ++pin) {
        ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)pin, 0));
    }
}

static pcnt_unit_handle_t encoder_init(int gpio_a, int gpio_b)
{
    pcnt_unit_config_t unit_cfg = {
        .low_limit = -PCNT_LIMIT,
        .high_limit = PCNT_LIMIT,
    };
    pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));

    // 1 us glitch rejection, appropriate as an initial value for manual spinning.
    // Must be re-evaluated after running motors at full speed.
    pcnt_glitch_filter_config_t glitch_cfg = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &glitch_cfg));

    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num = gpio_a,
        .level_gpio_num = gpio_b,
    };
    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num = gpio_b,
        .level_gpio_num = gpio_a,
    };
    pcnt_channel_handle_t channel_a = NULL;
    pcnt_channel_handle_t channel_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_a_cfg, &channel_a));
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_b_cfg, &channel_b));

    // Driver in ESP-IDF 6.x does not configure GPIO pull resistors for PCNT.
    // These weak pull-ups only establish an idle level if the sensor supports it.
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)gpio_a, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)gpio_b, GPIO_PULLUP_ONLY));

    // x4 quadrature decode: each rising/falling edge of both channels counts.
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
    return unit;
}

static bool valid_uart_request(const uint8_t *data, size_t len)
{
    if (len != 8 || data[0] != 'T' || data[1] != ' ') {
        return false;
    }
    for (size_t i = 2; i < 8; ++i) {
        if (data[i] < '0' || data[i] > '9') return false;
    }
    return true;
}

static void uart_ack_task(void *param)
{
    (void)param;
    uint8_t frame[UART_FRAME_MAX];
    size_t frame_len = 0;
    bool overflow = false;

    for (;;) {
        uint8_t b;
        int received = uart_read_bytes(UART_PORT, &b, 1, pdMS_TO_TICKS(20));
        if (received != 1) continue;
        if (b == '\n' && frame_len == 0 && !overflow) continue;
        if (b == '\r') {
            if (!overflow && valid_uart_request(frame, frame_len)) {
                uint8_t reply[12] = {'A','C','K',' ',0,0,0,0,0,0,'\r','\n'};
                memcpy(reply + 4, frame + 2, 6);
                int sent = uart_write_bytes(UART_PORT, reply, sizeof(reply));
                if (sent != (int)sizeof(reply)) {
                    ESP_LOGE(TAG, "UART short write: %d", sent);
                }
            } else {
                ESP_LOGW(TAG, "Invalid UART request length=%u", (unsigned)frame_len);
            }
            frame_len = 0;
            overflow = false;
        } else if (!overflow && frame_len < UART_FRAME_MAX) {
            frame[frame_len++] = b;
        } else {
            overflow = true;
        }
    }
}

static void uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_GPIO, UART_RX_GPIO,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    BaseType_t ok = xTaskCreate(uart_ack_task, "uart_ack", 4096, NULL, 6, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to launch UART task");
        abort();
    }
}

void app_main(void)
{
    set_motor_control_pins_low();
    encoder_units[0] = encoder_init(4, 5);
    encoder_units[1] = encoder_init(6, 7);
    uart_init();
    ESP_LOGI(TAG, "Ready: dual encoder PCNT x4, L=GPIO4/5 R=GPIO6/7");
    ESP_LOGI(TAG, "All motor control pins GPIO9..16 forced LOW");
    ESP_LOGI(TAG, "UART1 retained: TX=17 RX=18, 57600 baud, T xxxxxx -> ACK xxxxxx");
    ESP_LOGW(TAG, "Do NOT connect motor-power B+ during this encoder test");

    int prev_left = 0, prev_right = 0;
    while (true) {
        int left = 0, right = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_units[0], &left));
        ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_units[1], &right));
        ESP_LOGI(TAG, "Left=%d (delta=%d), Right=%d (delta=%d)",
                 left, left - prev_left, right, right - prev_right);
        prev_left = left;
        prev_right = right;
        // Avoid PCNT wrap: keep hand-rotated test to a few output revolutions.
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
