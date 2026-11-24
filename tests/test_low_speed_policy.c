/* Native test of the same admission and deadline helpers used by main.c. */
#include <assert.h>
#include <math.h>

#include "low_speed_policy.h"

int main(void)
{
    for (int rpm = -7; rpm <= 7; rpm++) {
        const bool candidate = (rpm >= 2 && rpm <= 5) || (rpm >= -5 && rpm <= -2);
        assert(low_speed_candidate(rpm) == candidate);
        assert(low_speed_pair(rpm, rpm, rpm) == (candidate || rpm == 0));
        assert(low_speed_pair(rpm, 0.0, -0.0));
    }
    int selection;
    assert(parse_low_speed_arm("2 WHEELS_UP", &selection) && selection == 2);
    assert(parse_low_speed_arm("-5 WHEELS_UP", &selection) && selection == -5);
    assert(!parse_low_speed_arm("2", &selection));
    assert(!parse_low_speed_arm("2.0 WHEELS_UP", &selection));
    assert(!parse_low_speed_arm(". WHEELS_UP", &selection));
    assert(!parse_low_speed_arm("6 WHEELS_UP", &selection));
    assert(!parse_low_speed_arm("-2 WHEELS_UP extra", &selection));
    assert(!low_speed_pair(2, 2.0, 3.0));
    assert(!low_speed_pair(2, 0.0, 2.0));
    assert(!low_speed_pair(2, -2.0, -2.0));
    assert(!low_speed_pair(2, 1.99999999, 2.0));
    assert(!low_speed_pair(2, NAN, 2.0));
    assert(!low_speed_pair(2, INFINITY, 2.0));
    assert(!low_speed_session_expired(2, LOW_SPEED_SESSION_US, LOW_SPEED_SESSION_US - 1));
    assert(low_speed_session_expired(-2, LOW_SPEED_SESSION_US, LOW_SPEED_SESSION_US));
    assert(!low_speed_session_expired(0, 0, LOW_SPEED_SESSION_US));
    int64_t progress_us = 0;
    assert(!signed_progress_stalled(0.0f, 1.0f, 100, &progress_us));
    assert(!signed_progress_stalled(-2.0f, 1.0f, 800099, &progress_us));
    assert(signed_progress_stalled(0.9f, 1.0f, 800100, &progress_us));
    assert(!signed_progress_stalled(1.0f, 1.0f, 800100, &progress_us));
    assert(!signed_progress_stalled(0.0f, 1.0f, 1600099, &progress_us));
    assert(signed_progress_stalled(0.0f, 1.0f, 1600100, &progress_us));
    return 0;
}
