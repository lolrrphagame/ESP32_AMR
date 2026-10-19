// ESP32-S3 N16R8: UART sequence/ACK test compatible with pi_uart_seq.py
// UART1: ESP TX=GPIO17, ESP RX=GPIO18, 57600 baud, 8N1
// UART0 (COM USB connector) remains available for ESP-IDF logs and flashing.
// No motor outputs are enabled.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TEST_UART       UART_NUM_1
#define TEST_TX_GPIO    17
#define TEST_RX_GPIO    18
#define TEST_BAUD       57600
#define FRAME_MAX       32

static const char *TAG = "UART_SEQ";

// Valid request is exactly 8 bytes before CR: 'T', ' ', six decimal digits.
static bool is_valid_request(const uint8_t *data, size_t len)
{
    if (len != 8 || data[0] != 'T' || data[1] != ' ') {
        return false;
    }
    for (size_t i = 2; i < 8; i++) {
        if (data[i] < '0' || data[i] > '9') {
            return false;
        }
    }
    return true;
}

static void report_bad_frame(const uint8_t *data, size_t len, bool overflow)
{
    char hex[3 * FRAME_MAX + 1] = {0};
    const size_t count = len < FRAME_MAX ? len : FRAME_MAX;
    for (size_t i = 0; i < count; i++) {
        snprintf(hex + 3 * i, sizeof(hex) - 3 * i, "%02X ", (unsigned)data[i]);
    }
    ESP_LOGW(TAG, "INVALID frame len=%u overflow=%d HEX=[%s]",
             (unsigned)len, (int)overflow, hex);
}

void app_main(void)
{
    const uart_config_t config = {
        .baud_rate = TEST_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(TEST_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(TEST_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(TEST_UART,
                                 TEST_TX_GPIO, TEST_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "Ready: UART1 TX=GPIO%d RX=GPIO%d baud=%d (8N1)",
             TEST_TX_GPIO, TEST_RX_GPIO, TEST_BAUD);
    ESP_LOGI(TAG, "Pi: T 000001<CR> -> ESP: ACK 000001<CR><LF>");

    uint8_t frame[FRAME_MAX];
    size_t frame_len = 0;
    bool overflow = false;

    while (true) {
        uint8_t byte;
        const int received = uart_read_bytes(TEST_UART, &byte, 1,
                                              pdMS_TO_TICKS(20));
        if (received != 1) {
            continue;
        }

        // Ignore a leftover LF between frames (the Pi script sends CR only).
        if (byte == '\n' && frame_len == 0 && !overflow) {
            continue;
        }

        if (byte == '\r') {
            if (!overflow && is_valid_request(frame, frame_len)) {
                // Exactly "ACK " + six digits + CRLF (12 bytes).
                uint8_t reply[12] = {'A', 'C', 'K', ' ', 0, 0, 0, 0, 0, 0, '\r', '\n'};
                memcpy(reply + 4, frame + 2, 6);
                const int sent = uart_write_bytes(TEST_UART, reply, sizeof(reply));
                if (sent != (int)sizeof(reply)) {
                    ESP_LOGE(TAG, "UART write error: %d bytes", sent);
                } else {
                    ESP_LOGI(TAG, "ACK %.*s", 6, (const char *)(frame + 2));
                }
            } else {
                report_bad_frame(frame, frame_len, overflow);
            }
            frame_len = 0;
            overflow = false;
            continue;
        }

        if (frame_len < FRAME_MAX && !overflow) {
            frame[frame_len++] = byte;
        } else {
            overflow = true;
        }
    }
}
