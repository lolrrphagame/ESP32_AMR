#ifndef AMR_LOW_SPEED_POLICY_H
#define AMR_LOW_SPEED_POLICY_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define LOW_SPEED_SESSION_US 15000000LL
#define SIGNED_PROGRESS_TIMEOUT_US 800000LL

static inline bool low_speed_candidate(int rpm)
{
    return (rpm >= 2 && rpm <= 5) || (rpm >= -5 && rpm <= -2);
}

static inline bool parse_low_speed_arm(const char *selection, int *rpm)
{
    const int sign = *selection == '-' ? -1 : 1;
    if (sign < 0) selection++;
    if (*selection < '2' || *selection > '5' ||
        strcmp(selection + 1, " WHEELS_UP")) return false;
    *rpm = sign * (*selection - '0');
    return true;
}

static inline bool low_speed_pair(int selected_rpm, double left, double right)
{
    return (left == 0.0 && right == 0.0) ||
           (low_speed_candidate(selected_rpm) && left == selected_rpm && right == selected_rpm);
}

static inline bool low_speed_session_expired(int selected_rpm, int64_t deadline_us, int64_t now)
{
    return selected_rpm != 0 && now >= deadline_us;
}

static inline bool signed_progress_stalled(float measured_rpm, float minimum_rpm,
                                           int64_t now, int64_t *last_progress_us)
{
    if (!*last_progress_us || measured_rpm >= minimum_rpm) *last_progress_us = now;
    return now - *last_progress_us >= SIGNED_PROGRESS_TIMEOUT_US;
}

#endif
