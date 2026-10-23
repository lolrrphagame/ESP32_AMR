// ESP32-S3 AMR - Phase 4: BENCH-ONLY single-wheel closed-loop PI velocity control.
// ESP-IDF 6.1. GPIO assignments retained from tested Phase 3.
// STARTUP: motors are OFF. No autonomous movement. A valid ARM + repeated V
// commands are necessary. STOP, bad serial framing, heartbeat or arm expiry disable motors.
// WARNING: Software stops are NOT a substitute for a physical E-stop, fuse, and EN pull-downs.
// Confirm IBT-2 74HC244 + driver supply levels and raise wheels BEFORE testing.

#define ENABLE_CLOSED_LOOP_TEST 1 // Change to 1 ONLY after hardware safety checks.
#define ALLOW_DUAL_WHEEL_TEST 0  // Keep at 0 in the initial bench tests.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define UART_PORT           UART_NUM_1
#define UART_TX_GPIO        17
#define UART_RX_GPIO        18
#define UART_BAUD           57600
#define FRAME_MAX          64
#define FRAME_GAP_US       200000LL
#define PCNT_LIMIT         30000
#define ENCODER_CPR        3960.0f
#define CTRL_PERIOD_MS     20
#define HEARTBEAT_US       300000LL
#define ARM_WINDOW_US      5000000LL
#define MAX_SETPOINT_RPM   40.0f
#define MAX_MEASURED_RPM   160.0f
#define PWM_RESOLUTION_HZ  10000000
#define PWM_FREQUENCY_HZ   20000
#define PWM_PERIOD_TICKS   (PWM_RESOLUTION_HZ / PWM_FREQUENCY_HZ)
#define MAX_DUTY_PERCENT   45.0f
#define DUTY_RAMP_PER_TICK 3.0f
// INITIAL experimental gains (NOT tuned). Units: duty-percent / rpm and duty-percent / (rpm*s).
#define PI_KP              0.40f
#define PI_KI              0.80f
#define FEEDFORWARD_BIAS   12.0f
#define FEEDFORWARD_SLOPE  0.24f

static const char *TAG = "AMR_P4";
static pcnt_unit_handle_t enc[2];
static mcpwm_timer_handle_t pwm_timer;
static QueueHandle_t uart_events;
static SemaphoreHandle_t state_mutex;

typedef struct {
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t cmp[2];
    mcpwm_gen_handle_t gen[2];
    gpio_num_t en_r, en_l;
    bool primed;
    int channel;
    int64_t ready_at_us;
} motor_hw_t;
static motor_hw_t motors[2];

typedef struct {
    bool armed;
    bool active;
    int64_t arm_deadline_us;
    int64_t last_command_us;
    float target[2];              // Normalized: POSITIVE == robot forward on BOTH wheels.
    float measured[2];            // Filtered signed RPM, normalized forward-positive.
    float duty[2];                // Applied PWM duty magnitude in percent.
    int32_t ticks[2];             // Normalized signed ticks (R has inverted raw sign).
    uint32_t uart_errors;
    uint32_t control_faults;
} app_state_t;
static app_state_t st;

static void lock_state(void)   { xSemaphoreTake(state_mutex, portMAX_DELAY); }
static void unlock_state(void) { xSemaphoreGive(state_mutex); }
static float limitf(float v, float lo, float hi) { return fminf(hi, fmaxf(lo, v)); }

static void gpio_motor_safe_at_boot(void)
{
    // Prepare output latch at 0 before switching pins to outputs.
    for (int pin = 9; pin <= 16; pin++) ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)pin, 0));
    gpio_config_t c = {
        .pin_bit_mask = (0xffULL << 9), .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&c));
    for (int pin = 9; pin <= 16; pin++) ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)pin, 0));
}

static pcnt_unit_handle_t encoder_init(int a, int b)
{
    pcnt_unit_config_t cfg = {
        .low_limit = -PCNT_LIMIT, .high_limit = PCNT_LIMIT,
        .flags.accum_count = true
    };
    pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&cfg, &unit));
    pcnt_glitch_filter_config_t filter = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter));
    pcnt_chan_config_t ca = {.edge_gpio_num = a, .level_gpio_num = b};
    pcnt_chan_config_t cb = {.edge_gpio_num = b, .level_gpio_num = a};
    pcnt_channel_handle_t ch_a = NULL, ch_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &ca, &ch_a));
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &cb, &ch_b));
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)a, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)b, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    // Required for 16-bit PCNT overflow compensation when accum_count=true.
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(unit, -PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit)); // Also activates watch points.
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
    return unit;
}

static void motor_disable_one_locked(int wheel)
{
    motor_hw_t *m = &motors[wheel];
    ESP_ERROR_CHECK(gpio_set_level(m->en_r, 0));
    ESP_ERROR_CHECK(gpio_set_level(m->en_l, 0));
    for (int j = 0; j < 2; ++j)
        if (m->gen[j]) ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->gen[j], 0, true));
    m->primed = false;
    st.duty[wheel] = 0.0f;
}

static void stop_all_locked(void)
{
    for (int i = 0; i < 2; ++i) motor_disable_one_locked(i);
    st.armed = false;
    st.active = false;
    st.arm_deadline_us = 0;
    st.last_command_us = 0;
    st.target[0] = st.target[1] = 0.0f;
}

static void motor_hw_init(void)
{
    mcpwm_timer_config_t cfg = {
        .group_id = 0, .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = PWM_PERIOD_TICKS
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&cfg, &pwm_timer));
    const int pwm[2][2] = {{9, 10}, {11, 12}};
    const int ens[2][2] = {{13, 14}, {15, 16}};
    for (int i = 0; i < 2; ++i) {
        motor_hw_t *m = &motors[i];
        m->en_r = (gpio_num_t)ens[i][0];
        m->en_l = (gpio_num_t)ens[i][1];
        mcpwm_operator_config_t ocfg = {.group_id = 0};
        ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &m->oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(m->oper, pwm_timer));
        for (int j = 0; j < 2; ++j) {
            mcpwm_comparator_config_t ccfg = {.flags.update_cmp_on_tez = true};
            ESP_ERROR_CHECK(mcpwm_new_comparator(m->oper, &ccfg, &m->cmp[j]));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->cmp[j], 0));
            mcpwm_generator_config_t gcfg = {.gen_gpio_num = pwm[i][j]};
            ESP_ERROR_CHECK(mcpwm_new_generator(m->oper, &gcfg, &m->gen[j]));
            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                m->gen[j], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                    MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                m->gen[j], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                    m->cmp[j], MCPWM_GEN_ACTION_LOW)));
            ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->gen[j], 0, true));
        }
        ESP_ERROR_CHECK(gpio_set_level(m->en_r, 0));
        ESP_ERROR_CHECK(gpio_set_level(m->en_l, 0));
    }
    ESP_ERROR_CHECK(mcpwm_timer_enable(pwm_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(pwm_timer, MCPWM_TIMER_START_NO_STOP));
    stop_all_locked();
}

static void motor_apply_locked(int wheel, float target_rpm, float duty, int64_t now_us)
{
    motor_hw_t *m = &motors[wheel];
    if (fabsf(target_rpm) < 0.1f || duty < 0.1f) {
        motor_disable_one_locked(wheel);
        return;
    }
    // As verified on the real robot: L forward = LPWM; R forward = RPWM.
    const int channel = wheel == 0 ? (target_rpm > 0 ? 1 : 0)
                                   : (target_rpm > 0 ? 0 : 1);
    const uint32_t ticks = (uint32_t)roundf(PWM_PERIOD_TICKS * duty / 100.0f);
    if (!m->primed || m->channel != channel) {
        motor_disable_one_locked(wheel);  // EN LOW FIRST, both PWM pins low.
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->cmp[channel], ticks));
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->gen[channel], -1, true));
        m->primed = true;
        m->channel = channel;
        m->ready_at_us = now_us + 10000LL; // Allow >=10ms for timer comparator update.
        st.duty[wheel] = duty;
        return;
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->cmp[channel], ticks));
    // Only enable the bridge once the correct PWM comparator/output is primed.
    if (now_us >= m->ready_at_us) {
        ESP_ERROR_CHECK(gpio_set_level(m->en_r, 1));
        ESP_ERROR_CHECK(gpio_set_level(m->en_l, 1));
    }
    st.duty[wheel] = duty;
}

static void control_task(void *arg)
{
    (void)arg;
    int previous[2] = {0, 0};
    int32_t count[2] = {0, 0};
    float filtered[2] = {0, 0};
    float integral[2] = {0, 0};
    float duty_last[2] = {0, 0};
    int stall_ms[2] = {0, 0};
    int64_t previous_us = esp_timer_get_time();
    int64_t log_us = previous_us;
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(CTRL_PERIOD_MS));
        int raw[2] = {0, 0};
        esp_err_t a = pcnt_unit_get_count(enc[0], &raw[0]);
        esp_err_t b = pcnt_unit_get_count(enc[1], &raw[1]);
        const int64_t now = esp_timer_get_time();
        float dt = (float)(now - previous_us) / 1000000.0f;
        previous_us = now;
        if (dt <= 0.0f) dt = CTRL_PERIOD_MS / 1000.0f;
        count[0] = raw[0];
        count[1] = -raw[1];
        for (int i = 0; i < 2; ++i) {
            const int step = count[i] - previous[i];
            previous[i] = count[i];
            const float instantaneous = step * 60.0f / (ENCODER_CPR * dt);
            filtered[i] = 0.7f * filtered[i] + 0.3f * instantaneous;
        }

        bool report = false, fault = false;
        app_state_t snapshot = {0};
        lock_state();
        for (int i = 0; i < 2; ++i) {
            st.measured[i] = filtered[i];
            st.ticks[i] = count[i];
        }
        const bool expired = st.armed && now > st.arm_deadline_us;
        const bool heartbeat_lost = st.active && (now - st.last_command_us > HEARTBEAT_US);
        const bool sample_late = dt > 0.1f; // Missed several control cycles.
        if (a != ESP_OK || b != ESP_OK || expired || heartbeat_lost || sample_late) {
            if (st.active) { st.control_faults++; fault = true; }
            stop_all_locked();
        }
        if (st.active) {
            for (int i = 0; i < 2; ++i) {
                const float ref = st.target[i];
                if (fabsf(ref) < 0.1f) {
                    integral[i] = 0; duty_last[i] = 0; stall_ms[i] = 0;
                    motor_disable_one_locked(i);
                    continue;
                }
                const float measured_along_command = (ref > 0.0f) ? filtered[i] : -filtered[i];
                const float error = fabsf(ref) - measured_along_command;
                // Anti-windup: cap integral in duty-percent units.
                integral[i] = limitf(integral[i] + PI_KI * error * dt, -15.0f, 15.0f);
                const float ff = FEEDFORWARD_BIAS + FEEDFORWARD_SLOPE * fabsf(ref);
                const float desired_duty = limitf(ff + PI_KP * error + integral[i], 0.0f, MAX_DUTY_PERCENT);
                const float duty = limitf(desired_duty, fmaxf(0.0f, duty_last[i] - DUTY_RAMP_PER_TICK),
                                           duty_last[i] + DUTY_RAMP_PER_TICK);
                duty_last[i] = duty;
                if (fabsf(filtered[i]) > MAX_MEASURED_RPM) {
                    st.control_faults++; fault = true; stop_all_locked(); break;
                }
                if (duty > 35.0f && fabsf(filtered[i]) < 1.0f) stall_ms[i] += CTRL_PERIOD_MS;
                else stall_ms[i] = 0;
                if (stall_ms[i] >= 800) {
                    st.control_faults++; fault = true; stop_all_locked(); break;
                }
                motor_apply_locked(i, ref, duty, now);
            }
        }
        if (!st.active) {
            for (int i = 0; i < 2; ++i) {
                integral[i] = duty_last[i] = 0;
                stall_ms[i] = 0;
                motor_disable_one_locked(i);
            }
        }
        if (now - log_us >= 200000LL) {
            log_us = now;
            snapshot = st;
            report = true;
        }
        unlock_state();
        if (fault) ESP_LOGW(TAG, "CONTROL STOP: deadline, heartbeat, PCNT, stall or timing fault");
        if (report) ESP_LOGI(TAG,
            "armed=%d active=%d | L ref=%+.1f act=%+.1f duty=%.1f ticks=%ld | R ref=%+.1f act=%+.1f duty=%.1f ticks=%ld",
            snapshot.armed, snapshot.active,
            snapshot.target[0], snapshot.measured[0], snapshot.duty[0], (long)snapshot.ticks[0],
            snapshot.target[1], snapshot.measured[1], snapshot.duty[1], (long)snapshot.ticks[1]);
    }
}

static void send_uart(const char *message)
{
    uart_write_bytes(UART_PORT, message, strlen(message));
}

static bool seq_test_frame(const char *s, size_t n)
{
    if (n != 8 || s[0] != 'T' || s[1] != ' ') return false;
    for (size_t i = 2; i < 8; ++i) if (s[i] < '0' || s[i] > '9') return false;
    return true;
}

static void process_line(char *line, size_t n)
{
    if (seq_test_frame(line, n)) {
        char out[13] = "ACK 000000\r\n";
        memcpy(out + 4, line + 2, 6);
        send_uart(out);
        return;
    }
    if (!strcmp(line, "STOP")) {
        lock_state(); stop_all_locked(); unlock_state();
        send_uart("OK STOP\r\n");
        return;
    }
    if (!strcmp(line, "Q")) {
        app_state_t s;
        lock_state(); s = st; unlock_state();
        char out[192];
        snprintf(out, sizeof(out), "STATUS A=%d RUN=%d L=%+.1f/%+.1f DL=%.1f TL=%ld R=%+.1f/%+.1f DR=%.1f TR=%ld UE=%lu CF=%lu\r\n",
                 s.armed, s.active, s.target[0], s.measured[0], s.duty[0], (long)s.ticks[0],
                 s.target[1], s.measured[1], s.duty[1], (long)s.ticks[1],
                 (unsigned long)s.uart_errors, (unsigned long)s.control_faults);
        send_uart(out);
        return;
    }
    if (!strcmp(line, "ARM")) {
#if ENABLE_CLOSED_LOOP_TEST
        lock_state();
        bool busy = st.active;
        if (!busy) {
            st.armed = true;
            st.arm_deadline_us = esp_timer_get_time() + ARM_WINDOW_US;
        }
        unlock_state();
        send_uart(busy ? "ERR BUSY\r\n" : "OK ARMED 5S\r\n");
#else
        send_uart("ERR TEST_DISABLED\r\n");
#endif
        return;
    }
    if (n >= 3 && line[0] == 'V' && line[1] == ' ') {
        float l = 0, r = 0;
        int end = 0;
        if (sscanf(line, "V %f %f %n", &l, &r, &end) != 2 || end != (int)n ||
            !isfinite(l) || !isfinite(r) || fabsf(l) > MAX_SETPOINT_RPM || fabsf(r) > MAX_SETPOINT_RPM ||
            (fabsf(l) > 0.1f && fabsf(l) < 5.0f) || (fabsf(r) > 0.1f && fabsf(r) < 5.0f)) {
            lock_state(); stop_all_locked(); unlock_state();
            send_uart("ERR BAD_V\r\n"); return;
        }
#if !ENABLE_CLOSED_LOOP_TEST
        send_uart("ERR TEST_DISABLED\r\n");
#else
#if !ALLOW_DUAL_WHEEL_TEST
        if (fabsf(l) > 0.1f && fabsf(r) > 0.1f) {
            lock_state(); stop_all_locked(); unlock_state();
            send_uart("ERR SINGLE_WHEEL_ONLY\r\n"); return;
        }
#endif
        const int64_t now = esp_timer_get_time();
        lock_state();
        if (!st.armed || now > st.arm_deadline_us) {
            stop_all_locked();
            unlock_state();
            send_uart("ERR NOT_ARMED\r\n");
            return;
        }
        bool reversal = false;
        if (st.active) {
            if ((st.target[0] * l < -0.01f) || (st.target[1] * r < -0.01f)) reversal = true;
        }
        if (reversal) {
            stop_all_locked(); unlock_state();
            send_uart("ERR STOP_BEFORE_REVERSE\r\n");
            return;
        }
        st.target[0] = fabsf(l) < 0.1f ? 0.0f : l;
        st.target[1] = fabsf(r) < 0.1f ? 0.0f : r;
        st.last_command_us = now;
        st.active = fabsf(st.target[0]) > 0.1f || fabsf(st.target[1]) > 0.1f;
        if (!st.active) stop_all_locked();
        unlock_state();
        send_uart("OK V\r\n");
#endif
        return;
    }
    lock_state(); stop_all_locked(); unlock_state();
    send_uart("ERR COMMAND\r\n");
}

static void uart_task(void *arg)
{
    (void)arg;
    char frame[FRAME_MAX + 1] = {0};
    size_t used = 0;
    bool overflow = false;
    int64_t last_byte = 0;
    uart_event_t e;
    for (;;) {
        if (xQueueReceive(uart_events, &e, pdMS_TO_TICKS(10))) {
            if (e.type == UART_FIFO_OVF || e.type == UART_BUFFER_FULL ||
                e.type == UART_FRAME_ERR || e.type == UART_PARITY_ERR) {
                lock_state(); st.uart_errors++; stop_all_locked(); unlock_state();
                uart_flush_input(UART_PORT);
                xQueueReset(uart_events);
                used = 0; overflow = false;
                ESP_LOGW(TAG, "UART hardware error event=%d; motors stopped", (int)e.type);
                send_uart("ERR UART_HW\r\n");
            } else if (e.type == UART_DATA) {
                size_t unread = e.size;
                while (unread > 0) {
                    uint8_t data[64];
                    size_t want = unread > sizeof(data) ? sizeof(data) : unread;
                    int got = uart_read_bytes(UART_PORT, data, want, pdMS_TO_TICKS(20));
                    if (got <= 0) break;
                    unread -= (size_t)got;
                    for (int i = 0; i < got; i++) {
                        const int64_t now = esp_timer_get_time();
                        if (used && now - last_byte > FRAME_GAP_US) {
                            used = 0; overflow = false;
                            lock_state(); st.uart_errors++; stop_all_locked(); unlock_state();
                            ESP_LOGW(TAG, "UART partial frame expired, motor disabled");
                        }
                        last_byte = now;
                        uint8_t b = data[i];
                        if (b == '\r' || b == '\n') {
                            if (overflow) {
                                lock_state(); st.uart_errors++; stop_all_locked(); unlock_state();
                                send_uart("ERR FRAME_OVERFLOW\r\n");
                            } else if (used > 0) {
                                frame[used] = 0;
                                process_line(frame, used);
                            }
                            used = 0; overflow = false;
                        } else if (b < 0x20 || b > 0x7e) {
                            used = 0; overflow = false;
                            lock_state(); st.uart_errors++; stop_all_locked(); unlock_state();
                            send_uart("ERR BAD_BYTE\r\n");
                        } else if (!overflow && used < FRAME_MAX) {
                            frame[used++] = (char)b;
                        } else {
                            overflow = true;
                        }
                    }
                }
            }
        }
        if (used && esp_timer_get_time() - last_byte > FRAME_GAP_US) {
            used = 0; overflow = false;
            lock_state(); st.uart_errors++; stop_all_locked(); unlock_state();
            ESP_LOGW(TAG, "UART frame timeout (no CR/LF), motors stopped");
        }
    }
}

static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = UART_BAUD, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 2048, 512, 20, &uart_events, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_GPIO, UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    if (xTaskCreate(uart_task, "amr_serial", 6144, NULL, 6, NULL) != pdPASS) abort();
}

void app_main(void)
{
    gpio_motor_safe_at_boot();
    state_mutex = xSemaphoreCreateMutex();
    if (!state_mutex) abort();
    enc[0] = encoder_init(4, 5);
    enc[1] = encoder_init(6, 7);
    motor_hw_init();
    uart_init();
    if (xTaskCreate(control_task, "amr_pi_control", 6144, NULL, 7, NULL) != pdPASS) abort();
    ESP_LOGI(TAG, "Phase 4: dual PI code (bench SINGLE wheel enabled option)");
    ESP_LOGI(TAG, "PCNT x4 CPR=3960; L forward=LPWM; R forward=RPWM; UART1=57600");
#if ENABLE_CLOSED_LOOP_TEST
    ESP_LOGW(TAG, "BENCH PI TEST ENABLED: ARM then V repeated; wheel raised + fuse + E-stop REQUIRED");
#else
    ESP_LOGW(TAG, "BENCH PI TEST DISABLED, motors remain OFF. Change ENABLE_CLOSED_LOOP_TEST to 1 after checks");
#endif
}
