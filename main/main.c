// ESP32-S3 N16R8 AMR: Phase 3 (controlled SINGLE-motor MCPWM pulse tests)
// Based on the user's Phase 2 code: PCNT quadrature x4 and UART1 sequence ACK.
// ESP-IDF 6.1. UART1: GPIO17 TX, GPIO18 RX, 57600 8N1.
// WARNING: BENCH TEST ONLY. NEVER run on the floor. E-stop, fuse, EN pulldowns required.
// Set ENABLE_PULSE_TEST to 1 ONLY after hardware safety checklist and no-load checks.
#define ENABLE_PULSE_TEST 0

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_PORT UART_NUM_1
#define UART_TX_GPIO 17
#define UART_RX_GPIO 18
#define UART_BAUD 57600
#define FRAME_MAX 64
#define PCNT_LIMIT 30000

#define PWM_RESOLUTION_HZ 10000000
#define PWM_FREQUENCY_HZ 20000
#define PWM_PERIOD_TICKS (PWM_RESOLUTION_HZ / PWM_FREQUENCY_HZ)
#define MAX_DUTY_PERCENT 25
#define MIN_DUTY_PERCENT 10
#define MAX_PULSE_MS 500
#define MIN_PULSE_MS 100
#define ARM_WINDOW_US 5000000LL

static const char *TAG = "AMR_P3";
static pcnt_unit_handle_t enc[2];
static mcpwm_timer_handle_t pwm_timer;

typedef struct {
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t cmpr[2];
    mcpwm_gen_handle_t gen[2];
    gpio_num_t en_r;
    gpio_num_t en_l;
    char name;
} motor_t;

static motor_t motors[2];
static bool pulse_active = false;
static int active_motor = -1;
static int64_t stop_time_us = 0;
static int64_t armed_until_us = 0;

static void set_all_motor_gpio_low(void)
{
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
    for (int pin = 9; pin <= 16; ++pin) ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)pin, 0));
}

static pcnt_unit_handle_t encoder_init(int a, int b)
{
    pcnt_unit_config_t ucfg = {.low_limit = -PCNT_LIMIT, .high_limit = PCNT_LIMIT};
    pcnt_unit_handle_t u = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &u));
    pcnt_glitch_filter_config_t f = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(u, &f));
    pcnt_chan_config_t ca = {.edge_gpio_num = a, .level_gpio_num = b};
    pcnt_chan_config_t cb = {.edge_gpio_num = b, .level_gpio_num = a};
    pcnt_channel_handle_t cha = NULL, chb = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(u, &ca, &cha));
    ESP_ERROR_CHECK(pcnt_new_channel(u, &cb, &chb));
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)a, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)b, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(cha, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                    PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(cha, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                     PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chb, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                    PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chb, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                     PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_enable(u));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(u));
    ESP_ERROR_CHECK(pcnt_unit_start(u));
    return u;
}

static void motor_stop_all(void)
{
    // EN LOW FIRST, then force BOTH PWM outputs LOW. This is a coast/disable request.
    for (int i = 0; i < 2; ++i) {
        gpio_set_level(motors[i].en_r, 0);
        gpio_set_level(motors[i].en_l, 0);
        for (int ch = 0; ch < 2; ++ch) {
            if (motors[i].gen[ch]) mcpwm_generator_set_force_level(motors[i].gen[ch], 0, true);
        }
    }
    pulse_active = false;
    active_motor = -1;
    stop_time_us = 0;
}

static void motor_hw_init(void)
{
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = PWM_PERIOD_TICKS,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &pwm_timer));

    const int pwm_pins[2][2] = {{9, 10}, {11, 12}};
    const int en_pins[2][2] = {{13, 14}, {15, 16}};

    for (int i = 0; i < 2; ++i) {
        motor_t *m = &motors[i];
        m->name = (i == 0) ? 'L' : 'R';
        m->en_r = (gpio_num_t)en_pins[i][0];
        m->en_l = (gpio_num_t)en_pins[i][1];

        mcpwm_operator_config_t oper_cfg = {.group_id = 0};
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &m->oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(m->oper, pwm_timer));

        for (int ch = 0; ch < 2; ++ch) {
            mcpwm_comparator_config_t cmp_cfg = {.flags.update_cmp_on_tez = true};
            ESP_ERROR_CHECK(mcpwm_new_comparator(m->oper, &cmp_cfg, &m->cmpr[ch]));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->cmpr[ch], 0));

            mcpwm_generator_config_t gen_cfg = {.gen_gpio_num = pwm_pins[i][ch]};
            ESP_ERROR_CHECK(mcpwm_new_generator(m->oper, &gen_cfg, &m->gen[ch]));
            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                m->gen[ch], MCPWM_GEN_TIMER_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                m->gen[ch], MCPWM_GEN_COMPARE_EVENT_ACTION(
                    MCPWM_TIMER_DIRECTION_UP, m->cmpr[ch], MCPWM_GEN_ACTION_LOW)));
            // Hold both PWM outputs at a definite LOW, even while the timer runs.
            ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->gen[ch], 0, true));
        }
        ESP_ERROR_CHECK(gpio_set_level(m->en_r, 0));
        ESP_ERROR_CHECK(gpio_set_level(m->en_l, 0));
    }

    ESP_ERROR_CHECK(mcpwm_timer_enable(pwm_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(pwm_timer, MCPWM_TIMER_START_NO_STOP));
    motor_stop_all();
}

static bool motor_pulse_start(int wheel, int channel, int percent, int duration_ms)
{
#if !ENABLE_PULSE_TEST
    (void)wheel; (void)channel; (void)percent; (void)duration_ms;
    return false;
#else
    if (pulse_active || wheel < 0 || wheel > 1 || channel < 0 || channel > 1 ||
        percent < MIN_DUTY_PERCENT || percent > MAX_DUTY_PERCENT ||
        duration_ms < MIN_PULSE_MS || duration_ms > MAX_PULSE_MS) return false;

    motor_stop_all();
    motor_t *m = &motors[wheel];
    const uint32_t compare = (uint32_t)(PWM_PERIOD_TICKS * percent / 100);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->cmpr[channel], compare));
    // EN remains LOW until PWM is configured. Only ONE channel is released.
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->gen[channel], -1, true));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(gpio_set_level(m->en_r, 1));
    ESP_ERROR_CHECK(gpio_set_level(m->en_l, 1));
    active_motor = wheel;
    pulse_active = true;
    stop_time_us = esp_timer_get_time() + 1000LL * duration_ms;
    ESP_LOGW(TAG, "PULSE %c %s duty=%d%% duration=%dms", m->name,
             channel == 0 ? "RPWM" : "LPWM", percent, duration_ms);
    return true;
#endif
}

static void uart_send(const char *text)
{
    uart_write_bytes(UART_PORT, text, strlen(text));
}

static bool is_test_frame(const char *data, size_t len)
{
    if (len != 8 || data[0] != 'T' || data[1] != ' ') return false;
    for (size_t i = 2; i < 8; ++i) if (data[i] < '0' || data[i] > '9') return false;
    return true;
}

static void process_uart_line(const char *line, size_t n)
{
    if (is_test_frame(line, n)) {
        char ack[13] = "ACK 000000\r\n";
        memcpy(ack + 4, line + 2, 6);
        uart_send(ack);
        return;
    }
    if (strcmp(line, "STOP") == 0) {
        motor_stop_all();
        armed_until_us = 0;
        uart_send("OK STOP\r\n");
        return;
    }
    if (strcmp(line, "ARM") == 0) {
#if ENABLE_PULSE_TEST
        if (pulse_active) {
            uart_send("ERR BUSY\r\n");
        } else {
            armed_until_us = esp_timer_get_time() + ARM_WINDOW_US;
            uart_send("OK ARMED 5S\r\n");
        }
#else
        uart_send("ERR TEST_DISABLED\r\n");
#endif
        return;
    }

    char wheel, ch; int duty, duration, parsed = 0;
    if (sscanf(line, "M %c %c %d %d %n", &wheel, &ch, &duty, &duration, &parsed) == 4 &&
        parsed == (int)n && (wheel == 'L' || wheel == 'R') && (ch == 'R' || ch == 'L')) {
        if (armed_until_us == 0 || esp_timer_get_time() > armed_until_us) {
            uart_send("ERR NOT_ARMED\r\n");
            return;
        }
        armed_until_us = 0; // One ARM authorizes one and only one bounded pulse.
        if (motor_pulse_start(wheel == 'L' ? 0 : 1, ch == 'R' ? 0 : 1, duty, duration)) {
            uart_send("OK PULSE_STARTED\r\n");
        } else {
            uart_send("ERR INVALID_OR_DISABLED\r\n");
        }
        return;
    }
    uart_send("ERR COMMAND\r\n");
}

static void uart_task(void *param)
{
    (void)param;
    char frame[FRAME_MAX + 1];
    size_t n = 0;
    bool overflow = false;
    while (true) {
        // Software stop after a short bounded pulse, even with no new UART data.
        if (pulse_active && esp_timer_get_time() >= stop_time_us) {
            motor_stop_all();
            ESP_LOGI(TAG, "Pulse completed, motor disabled");
        }
        uint8_t b;
        int rd = uart_read_bytes(UART_PORT, &b, 1, pdMS_TO_TICKS(10));
        if (rd != 1) continue;
        if (b == '\n' && n == 0) continue;
        if (b == '\r' || b == '\n') {
            if (!overflow && n > 0) {
                frame[n] = '\0';
                process_uart_line(frame, n);
            } else if (overflow) {
                uart_send("ERR FRAME_OVERFLOW\r\n");
            }
            n = 0;
            overflow = false;
        } else if (!overflow && n < FRAME_MAX) {
            frame[n++] = (char)b;
        } else {
            overflow = true;
        }
    }
}

static void uart_init(void)
{
    uart_config_t cfg = {
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
    if (xTaskCreate(uart_task, "amr_uart", 6144, NULL, 6, NULL) != pdPASS) abort();
}

void app_main(void)
{
    set_all_motor_gpio_low();
    enc[0] = encoder_init(4, 5);
    enc[1] = encoder_init(6, 7);
    motor_hw_init();
    uart_init();
    ESP_LOGI(TAG, "Phase3 MCPWM 20kHz (L:9/10 EN13/14, R:11/12 EN15/16)");
    ESP_LOGI(TAG, "PCNT quadrature x4 L=4/5 R=6/7; count/rev approx 3960");
    ESP_LOGI(TAG, "UART1 GPIO17/18, baud 57600, old T/ACK sequence test kept");
#if ENABLE_PULSE_TEST
    ESP_LOGW(TAG, "MOTOR TEST ENABLED: ARM then single M command; never run on floor");
#else
    ESP_LOGW(TAG, "MOTOR TEST DISABLED IN FIRMWARE. NO MOTOR PULSE CAN BE REQUESTED");
#endif

    int prev_l = 0, prev_r = 0;
    for (;;) {
        int left = 0, right = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(enc[0], &left));
        ESP_ERROR_CHECK(pcnt_unit_get_count(enc[1], &right));
        const int delta_l = left - prev_l;
        const int delta_r = right - prev_r;
        ESP_LOGI(TAG, "raw L=%d dL=%d | raw R=%d dR=%d | forward-normalized L=%d R=%d",
                 left, delta_l, right, delta_r, left, -right);
        prev_l = left;
        prev_r = right;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
