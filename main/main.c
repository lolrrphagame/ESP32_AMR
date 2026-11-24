// ESP32-S3 AMR - Phase 4L.1: guarded bench-only PI velocity control.
// ESP-IDF 6.1. GPIO assignments retained from tested Phase 3.
// STARTUP: motors are OFF. ARM or ARM_CONT and valid V frames are necessary.
// STOP, framing faults, heartbeat loss and applicable arm expiry disable motors.
// WARNING: Software stops are NOT a substitute for a physical E-stop, fuse, and EN pull-downs.
// Confirm IBT-2 74HC244 + driver supply levels and raise wheels BEFORE testing.

#define ENABLE_CLOSED_LOOP_TEST 1 // Change to 1 ONLY after hardware safety checks.
#define ALLOW_DUAL_WHEEL_TEST 1  // Keep at 0 in the initial bench tests.

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <limits.h>
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
#include "esp_random.h"
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
#define STATIONARY_US      250000LL
#define STATIONARY_RPM     1.0f
#define STALL_TIMEOUT_US   800000LL
#define MIN_SETPOINT_RPM   5.0f
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
// Phase 4E.3: keep the experimentally tested left-wheel forward trim.
#define LEFT_FORWARD_FF_TRIM 1.5f

// Phase 4F: small outer encoder-position synchronizer for STRAIGHT FORWARD only.
// Preserve original UART protocol and independent PI speed controllers.
// Set to 0 for an A/B comparison using the same firmware; rebuild and flash.
#define ENABLE_STRAIGHT_SYNC     1
#define SYNC_MIN_BASE_RPM       7.0f
#define SYNC_MAX_BASE_RPM      20.0f
#define SYNC_KP_RPM_PER_TICK    0.006f // 100 ticks error -> 0.6 RPM per wheel.
#define SYNC_MAX_CORR_RPM       1.0f   // Bounds adjustment to commanded target.
#define SYNC_SLEW_RPM_PER_TICK  0.06f  // Max step per 20 ms controller tick.
#define SYNC_MAX_ERROR_TICKS    800    // Stop if straight-wheel mismatch is excessive.

static const char *TAG = "AMR_P4";
static pcnt_unit_handle_t enc[2];
static mcpwm_timer_handle_t pwm_timer;
static QueueHandle_t uart_events;
static SemaphoreHandle_t state_mutex;
static uint32_t boot_id;

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
    bool continuous;
    bool stationary;
    int64_t quiet_since_us;
    const char *fault_reason;     // First fault, cleared only by an ESP32 reboot.
    int last_direction[2];       // Survives zero-wheel targets and STOP.
    int64_t arm_deadline_us;
    int64_t last_command_us;
    float target[2];              // Normalized: POSITIVE == robot forward on BOTH wheels.
    float measured[2];            // Filtered signed RPM, normalized forward-positive.
    float duty[2];                // Applied PWM duty magnitude in percent.
    int32_t ticks[2];             // Normalized signed ticks (R has inverted raw sign).
    uint32_t uart_errors;
    uint32_t control_faults;
    uint32_t motion_epoch;        // Invalidate synchronizer after ANY STOP/re-arm.
    bool sync_enabled;            // Diagnostic fields; not part of UART protocol.
    int32_t sync_error_ticks;     // (right delta - left delta) since straight segment start.
    float sync_correction_rpm;    // Added to L, subtracted from R when straight forward.
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
    st.motion_epoch++; // Handles STOP followed by ARM/V between control cycles.
    st.sync_enabled = false;
    st.sync_error_ticks = 0;
    st.sync_correction_rpm = 0.0f;
    for (int i = 0; i < 2; ++i) motor_disable_one_locked(i);
    if (st.active) {
        st.stationary = false;
        st.quiet_since_us = 0;
    }
    st.continuous = false;
    st.armed = false;
    st.active = false;
    st.arm_deadline_us = 0;
    st.last_command_us = 0;
    st.target[0] = st.target[1] = 0.0f;
}

static void latch_fault_locked(const char *reason, bool uart_fault)
{
    if (uart_fault) st.uart_errors++;
    else st.control_faults++;
    if (!st.fault_reason) st.fault_reason = reason;
    stop_all_locked();
}

// Both the control loop and UART admission enforce deadlines, so a late V
// cannot renew an expired heartbeat between control ticks.
static void check_deadlines_locked(int64_t now)
{
    if (st.active && now - st.last_command_us >= HEARTBEAT_US) {
        latch_fault_locked("HEARTBEAT", false);
    } else if (st.armed && (!st.continuous || !st.active) && now >= st.arm_deadline_us) {
        if (st.active) latch_fault_locked("ARM_EXPIRED", false);
        else stop_all_locked();
    }
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
        return;
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->cmp[channel], ticks));
    // Only enable the bridge once the correct PWM comparator/output is primed.
    if (now_us >= m->ready_at_us) {
        ESP_ERROR_CHECK(gpio_set_level(m->en_r, 1));
        ESP_ERROR_CHECK(gpio_set_level(m->en_l, 1));
        st.duty[wheel] = ticks * 100.0f / PWM_PERIOD_TICKS;
    }
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
    int64_t no_progress_since[2] = {0, 0};
    bool sync_tracking = false;
    uint32_t sync_epoch = 0;
    uint32_t pi_epoch = 0;
    int32_t sync_start[2] = {0, 0};
    float sync_base_rpm = 0.0f;
    float sync_corr_rpm = 0.0f;
    int64_t previous_us = esp_timer_get_time();
    int64_t log_us = previous_us;
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(CTRL_PERIOD_MS));
        int raw[2] = {0, 0};
        esp_err_t a = pcnt_unit_get_count(enc[0], &raw[0]);
        esp_err_t b = pcnt_unit_get_count(enc[1], &raw[1]);
        const int64_t now = esp_timer_get_time();
        const float dt = (float)(now - previous_us) / 1000000.0f;
        previous_us = now;
        const bool sample_valid = a == ESP_OK && b == ESP_OK && raw[1] != INT_MIN;
        const bool timing_valid = dt > 0.0f && dt <= 0.1f;
        bool no_ticks = true;
        bool overspeed = false;
        if (sample_valid) {
            count[0] = raw[0];
            count[1] = -raw[1];
            for (int i = 0; i < 2; ++i) {
                const int64_t step = (int64_t)count[i] - previous[i];
                previous[i] = count[i];
                no_ticks = no_ticks && step == 0;
                if (timing_valid) {
                    const float instantaneous = step * 60.0f / (ENCODER_CPR * dt);
                    filtered[i] = 0.7f * filtered[i] + 0.3f * instantaneous;
                    overspeed = overspeed || fabsf(filtered[i]) > MAX_MEASURED_RPM;
                }
            }
        }

        bool report = false;
        app_state_t snapshot = {0};
        lock_state();
        const char *prior_fault = st.fault_reason;
        for (int i = 0; i < 2; ++i) {
            st.measured[i] = filtered[i];
            st.ticks[i] = count[i];
        }
        check_deadlines_locked(esp_timer_get_time());
        if (!sample_valid) latch_fault_locked("ENCODER", false);
        else if (!timing_valid) latch_fault_locked("TIMING", false);
        else if (overspeed) latch_fault_locked("OVERSPEED", false);

        if (st.active || !sample_valid || !timing_valid || !no_ticks ||
            fabsf(filtered[0]) >= STATIONARY_RPM || fabsf(filtered[1]) >= STATIONARY_RPM) {
            st.quiet_since_us = 0;
            st.stationary = false;
        } else {
            if (!st.quiet_since_us) st.quiet_since_us = now;
            st.stationary = now - st.quiet_since_us >= STATIONARY_US;
        }
        // Outer loop is deliberately OFF for reverse, turning, unequal wheel setpoints,
        // very slow/fast test motions, or whenever the motors are not active.
        // Q and the log retain ORIGINAL commanded RPM in st.target[].
        float effective_ref[2] = {st.target[0], st.target[1]};
#if ENABLE_STRAIGHT_SYNC
        const bool straight_forward = st.active &&
            st.target[0] >= SYNC_MIN_BASE_RPM &&
            st.target[0] <= SYNC_MAX_BASE_RPM &&
            fabsf(st.target[0] - st.target[1]) < 0.05f;
        if (straight_forward) {
            // Reset reference counts at every new straight segment or motion session.
            if (!sync_tracking || sync_epoch != st.motion_epoch ||
                fabsf(sync_base_rpm - st.target[0]) > 0.05f) {
                sync_start[0] = count[0];
                sync_start[1] = count[1];
                sync_corr_rpm = 0.0f;
                sync_base_rpm = st.target[0];
                sync_epoch = st.motion_epoch;
                sync_tracking = true;
            }
            const int64_t left_delta = (int64_t)count[0] - sync_start[0];
            const int64_t right_delta = (int64_t)count[1] - sync_start[1];
            // When right is ahead, speed up left slightly and slow down right.
            const int64_t error_ticks = right_delta - left_delta;
            if (error_ticks > SYNC_MAX_ERROR_TICKS || error_ticks < -SYNC_MAX_ERROR_TICKS) {
                latch_fault_locked("ENCODER_SYNC", false);
                sync_tracking = false;
                sync_corr_rpm = 0.0f;
            } else {
                const float wanted = limitf(error_ticks * SYNC_KP_RPM_PER_TICK,
                    -SYNC_MAX_CORR_RPM, SYNC_MAX_CORR_RPM);
                sync_corr_rpm = limitf(wanted,
                    sync_corr_rpm - SYNC_SLEW_RPM_PER_TICK,
                    sync_corr_rpm + SYNC_SLEW_RPM_PER_TICK);
                effective_ref[0] = st.target[0] + sync_corr_rpm;
                effective_ref[1] = st.target[1] - sync_corr_rpm;
                st.sync_enabled = true;
                st.sync_error_ticks = error_ticks;
                st.sync_correction_rpm = sync_corr_rpm;
            }
        } else {
            sync_tracking = false;
            sync_corr_rpm = 0.0f;
            st.sync_enabled = false;
            st.sync_error_ticks = 0;
            st.sync_correction_rpm = 0.0f;
        }
#endif
        if (pi_epoch != st.motion_epoch) {
            pi_epoch = st.motion_epoch;
            for (int i = 0; i < 2; ++i) {
                integral[i] = duty_last[i] = 0;
                stall_ms[i] = 0;
                no_progress_since[i] = 0;
            }
        }
        if (st.active) {
            for (int i = 0; i < 2; ++i) {
                const float ref = effective_ref[i];
                if (fabsf(ref) < 0.1f) {
                    integral[i] = 0; duty_last[i] = 0; stall_ms[i] = 0;
                    no_progress_since[i] = 0;
                    motor_disable_one_locked(i);
                    continue;
                }
                const float measured_along_command = (ref > 0.0f) ? filtered[i] : -filtered[i];
                const float error = fabsf(ref) - measured_along_command;
                // Anti-windup: cap integral in duty-percent units.
                integral[i] = limitf(integral[i] + PI_KI * error * dt, -15.0f, 15.0f);
                // Wheel-specific forward trim; leave right wheel and reverse unchanged.
                const float wheel_trim =
                    (i == 0 && ref > 0.0f) ? LEFT_FORWARD_FF_TRIM : 0.0f;
                const float ff = FEEDFORWARD_BIAS + FEEDFORWARD_SLOPE * fabsf(ref)
                               + wheel_trim;
                const float desired_duty = limitf(ff + PI_KP * error + integral[i], 0.0f, MAX_DUTY_PERCENT);
                const float duty = limitf(desired_duty, fmaxf(0.0f, duty_last[i] - DUTY_RAMP_PER_TICK),
                                           duty_last[i] + DUTY_RAMP_PER_TICK);
                duty_last[i] = duty;
                if (duty > 35.0f && fabsf(filtered[i]) < 1.0f) stall_ms[i] += CTRL_PERIOD_MS;
                else stall_ms[i] = 0;
                // At low targets, feedforward and PI may never reach 35% duty.
                // Require measured progress in the commanded direction in continuous mode.
                if (!no_progress_since[i] || measured_along_command >= STATIONARY_RPM) {
                    no_progress_since[i] = now;
                }
                if (stall_ms[i] >= 800 ||
                    (st.continuous && now - no_progress_since[i] >= STALL_TIMEOUT_US)) {
                    latch_fault_locked("STALL", false);
                    break;
                }
                motor_apply_locked(i, ref, duty, now);
            }
        }
        if (!st.active) {
            sync_tracking = false;
            sync_corr_rpm = 0.0f;
            st.sync_enabled = false;
            st.sync_error_ticks = 0;
            st.sync_correction_rpm = 0.0f;
            for (int i = 0; i < 2; ++i) {
                integral[i] = duty_last[i] = 0;
                stall_ms[i] = 0;
                no_progress_since[i] = 0;
                motor_disable_one_locked(i);
            }
        }
        if (now - log_us >= 200000LL) {
            log_us = now;
            snapshot = st;
            report = true;
        }
        const char *new_fault = prior_fault ? NULL : st.fault_reason;
        unlock_state();
        if (new_fault) ESP_LOGW(TAG, "CONTROL FAULT: %s", new_fault);
        if (report) ESP_LOGI(TAG,
            "armed=%d active=%d | L ref=%+.1f act=%+.1f duty=%.1f ticks=%ld | R ref=%+.1f act=%+.1f duty=%.1f ticks=%ld | SYNC=%d e=%ld corr=%+.2f",
            snapshot.armed, snapshot.active,
            snapshot.target[0], snapshot.measured[0], snapshot.duty[0], (long)snapshot.ticks[0],
            snapshot.target[1], snapshot.measured[1], snapshot.duty[1], (long)snapshot.ticks[1],
            snapshot.sync_enabled, (long)snapshot.sync_error_ticks, snapshot.sync_correction_rpm);
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

static void uart_fault(const char *reason)
{
    lock_state();
    latch_fault_locked(reason, true);
    unlock_state();
}

#if ENABLE_CLOSED_LOOP_TEST
static const char *arm_locked(bool continuous, int64_t now)
{
    check_deadlines_locked(now);
    if (st.fault_reason) return "ERR FAULT\r\n";
    if (st.active || (st.armed && (continuous || st.continuous))) return "ERR BUSY\r\n";
    if (continuous && !st.stationary) return "ERR NOT_STATIONARY\r\n";
    st.armed = true;
    st.continuous = continuous;
    st.arm_deadline_us = now + ARM_WINDOW_US;
    return continuous ? "OK ARMED CONT\r\n" : "OK ARMED 5S\r\n";
}

static const char *velocity_locked(float left, float right, int64_t now)
{
    check_deadlines_locked(now);
    if (st.fault_reason) return "ERR FAULT\r\n";

    float target[] = {left, right};
    for (int i = 0; i < 2; ++i) {
        // Retain the legacy bench deadband while explicitly rejecting every
        // unsupported nonzero continuous-mode setpoint.
        if (!st.continuous && fabsf(target[i]) <= 0.1f) target[i] = 0;
        if (target[i] != 0 && fabsf(target[i]) < MIN_SETPOINT_RPM) {
            latch_fault_locked("LOW_SPEED", false);
            return "ERR LOW_SPEED\r\n";
        }
    }
    if (target[0] == 0 && target[1] == 0) {
        stop_all_locked();
        return "OK V\r\n";
    }
    if (!st.armed) return "ERR NOT_ARMED\r\n";
#if !ALLOW_DUAL_WHEEL_TEST
    if (target[0] != 0 && target[1] != 0) {
        latch_fault_locked("SINGLE_WHEEL_ONLY", false);
        return "ERR SINGLE_WHEEL_ONLY\r\n";
    }
#endif
    if (st.continuous && !st.active && !st.stationary) {
        latch_fault_locked("NOT_STATIONARY", false);
        return "ERR NOT_STATIONARY\r\n";
    }
    for (int i = 0; i < 2; ++i) {
        if (target[i] != 0 && st.last_direction[i] * target[i] < 0 &&
            (st.active || !st.stationary)) {
            latch_fault_locked("REVERSAL", false);
            return "ERR STOP_BEFORE_REVERSE\r\n";
        }
    }
    for (int i = 0; i < 2; ++i) {
        st.target[i] = target[i];
        if (target[i] != 0) st.last_direction[i] = target[i] > 0 ? 1 : -1;
    }
    st.stationary = false;
    st.quiet_since_us = 0;
    st.last_command_us = now;
    st.active = true;
    return "OK V\r\n";
}
#endif

static bool parse_velocity(const char *line, float *left, float *right)
{
    const char *start = line + 2;
    char *end;
    errno = 0;
    *left = strtof(start, &end);
    if (end == start || *end != ' ' || errno == ERANGE || !isfinite(*left)) return false;

    start = end + 1;
    errno = 0;
    *right = strtof(start, &end);
    if (end == start || errno == ERANGE || !isfinite(*right)) return false;
    while (*end == ' ') end++;
    return *end == '\0';
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
    if (!strcmp(line, "VER")) {
        send_uart("VERSION PV=3 CAP=ARM_CONT,STICKY_FAULT,STATIONARY,BOOT_ID\r\n");
        return;
    }
    if (!strcmp(line, "Q")) {
        app_state_t s;
        lock_state();
        check_deadlines_locked(esp_timer_get_time());
        s = st;
        unlock_state();
        char out[256];
        snprintf(out, sizeof(out),
                 "STATUS A=%d RUN=%d L=%+.1f/%+.1f DL=%.1f TL=%ld R=%+.1f/%+.1f DR=%.1f TR=%ld UE=%lu CF=%lu"
                 " MODE=%s FL=%d FR=%s ST=%d BOOT=%08lx\r\n",
                 s.armed, s.active, s.target[0], s.measured[0], s.duty[0], (long)s.ticks[0],
                 s.target[1], s.measured[1], s.duty[1], (long)s.ticks[1],
                 (unsigned long)s.uart_errors, (unsigned long)s.control_faults,
                 s.armed ? (s.continuous ? "CONT" : "LEGACY") : "IDLE",
                 s.fault_reason != NULL, s.fault_reason ? s.fault_reason : "NONE",
                 s.stationary, (unsigned long)boot_id);
        send_uart(out);
        return;
    }
    if (!strcmp(line, "ARM") || !strcmp(line, "ARM_CONT")) {
#if ENABLE_CLOSED_LOOP_TEST
        lock_state();
        const char *reply = arm_locked(!strcmp(line, "ARM_CONT"), esp_timer_get_time());
        unlock_state();
        send_uart(reply);
#else
        send_uart("ERR TEST_DISABLED\r\n");
#endif
        return;
    }
    if (n >= 2 && line[0] == 'V' && line[1] == ' ') {
        float l, r;
        if (!parse_velocity(line, &l, &r)) {
            uart_fault("BAD_V");
            send_uart("ERR BAD_V\r\n");
            return;
        }
        if (fabsf(l) > MAX_SETPOINT_RPM || fabsf(r) > MAX_SETPOINT_RPM) {
            lock_state(); latch_fault_locked("SETPOINT_RANGE", false); unlock_state();
            send_uart("ERR BAD_V\r\n");
            return;
        }
#if ENABLE_CLOSED_LOOP_TEST
        lock_state();
        const char *reply = velocity_locked(l, r, esp_timer_get_time());
        unlock_state();
        send_uart(reply);
#else
        send_uart("ERR TEST_DISABLED\r\n");
#endif
        return;
    }
    uart_fault("COMMAND");
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
                e.type == UART_FRAME_ERR || e.type == UART_PARITY_ERR || e.type == UART_BREAK) {
                uart_fault("UART_HW");
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
                    if (got <= 0) {
                        uart_fault("UART_RX");
                        used = 0;
                        overflow = true;
                        last_byte = esp_timer_get_time();
                        break;
                    }
                    unread -= (size_t)got;
                    for (int i = 0; i < got; i++) {
                        const int64_t now = esp_timer_get_time();
                        if ((used || overflow) && now - last_byte > FRAME_GAP_US) {
                            used = 0; overflow = true;
                            uart_fault("FRAME_GAP");
                            ESP_LOGW(TAG, "UART partial frame expired, motor disabled");
                        }
                        last_byte = now;
                        uint8_t b = data[i];
                        if (b == '\r' || b == '\n') {
                            if (overflow) {
                                send_uart("ERR FRAME\r\n");
                            } else if (used > 0) {
                                frame[used] = 0;
                                process_line(frame, used);
                            }
                            used = 0; overflow = false;
                        } else if (b < 0x20 || b > 0x7e) {
                            used = 0; overflow = true;
                            uart_fault("BAD_BYTE");
                            send_uart("ERR BAD_BYTE\r\n");
                        } else if (!overflow && used < FRAME_MAX) {
                            frame[used++] = (char)b;
                        } else if (!overflow) {
                            overflow = true;
                            uart_fault("FRAME_OVERFLOW");
                        }
                    }
                }
            }
        }
        if (used && esp_timer_get_time() - last_byte > FRAME_GAP_US) {
            used = 0; overflow = false;
            uart_fault("FRAME_GAP");
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
    boot_id = esp_random();
    uart_init();
    if (xTaskCreate(control_task, "amr_pi_control", 6144, NULL, 7, NULL) != pdPASS) abort();
    ESP_LOGI(TAG, "Phase 4L.1: guarded continuous bench mode");
    ESP_LOGI(TAG, "Straight sync enabled=%d, kp=%.4f RPM/tick, cap=%.1f RPM",
        ENABLE_STRAIGHT_SYNC, (double)SYNC_KP_RPM_PER_TICK, (double)SYNC_MAX_CORR_RPM);
    ESP_LOGI(TAG, "PCNT x4 CPR=3960; L forward=LPWM; R forward=RPWM; UART1=57600");
#if ENABLE_CLOSED_LOOP_TEST
    ESP_LOGW(TAG, "BENCH: raise wheels, verify fuse and physical E-stop before ARM or ARM_CONT");
#else
    ESP_LOGW(TAG, "BENCH PI TEST DISABLED, motors remain OFF. Change ENABLE_CLOSED_LOOP_TEST to 1 after checks");
#endif
}
