// Native tests for the firmware's shared UART startup policy. No peripherals.
#include <assert.h>
#include <string.h>
#include "uart_startup.h"

static void prove_all(uart_startup_t *link, uint64_t challenge)
{
    for (unsigned i = 1; i <= UART_PROOF_COUNT; ++i)
        assert(uart_startup_prove(link, challenge, i));
}

static void test_clean_start(void)
{
    uart_startup_t link = {0};
    assert(link.state == WAIT_LINK);
    assert(!uart_startup_motion_allowed(&link));
    uart_startup_stop(&link);
    assert(link.state == WAIT_LINK);
    assert(uart_startup_begin(&link, 17, 0));
    assert(!uart_startup_motion_allowed(&link));
    assert(!uart_startup_prove(&link, 17, 1));  // Needs STOP in this attempt.
    uart_startup_stop(&link);
    assert(!uart_startup_prove(&link, 17, 2));
    assert(uart_startup_prove(&link, 17, 1));
    assert(!uart_startup_prove(&link, 17, 1));  // Duplicate cannot count.
    assert(!uart_startup_commit(&link, 17, UART_CLEAN_US, true));
    uart_startup_stop(&link);
    prove_all(&link, 17);
    assert(!uart_startup_commit(&link, 17, UART_CLEAN_US - 1, true));
    assert(!uart_startup_commit(&link, 17, UART_CLEAN_US, false));
    assert(uart_startup_commit(&link, 17, UART_CLEAN_US, true));
    assert(uart_startup_motion_allowed(&link));
    assert(!uart_startup_commit(&link, 17, UART_CLEAN_US, true));
    uart_startup_stop(&link);
    assert(link.state == LINK_READY);
}

static void test_startup_errors_and_replay(void)
{
    uart_startup_t link = {0};
    assert(!uart_startup_record_error(&link, AMR_EVENT_UART_BREAK, 10));
    assert(!uart_startup_record_error(&link, AMR_EVENT_UART_FRAME_ERR, 20));
    assert(uart_startup_begin(&link, 123, 30));
    uart_startup_stop(&link);
    prove_all(&link, 123);
    assert(!uart_startup_record_error(&link, AMR_EVENT_UART_BREAK, 40));
    assert(link.state == WAIT_LINK && !uart_startup_motion_allowed(&link));
    assert(!uart_startup_commit(&link, 123, UART_CLEAN_US + 30, true));
    assert(uart_startup_begin(&link, 456, 50));
    uart_startup_stop(&link);
    assert(!uart_startup_prove(&link, 123, 1));
    prove_all(&link, 456);
    assert(!uart_startup_commit(&link, 123, UART_CLEAN_US + 50, true));
    assert(uart_startup_commit(&link, 456, UART_CLEAN_US + 50, true));
    assert(link.startup_total == 3 && link.operational_total == 0);
    assert(link.startup[AMR_EVENT_UART_BREAK] == 2);
    assert(link.startup[AMR_EVENT_UART_FRAME_ERR] == 1);
    assert(link.last_error_us == 40 && strcmp(link.last_event, "UART_BREAK") == 0);
}

static void test_expire_and_persistent_fault(void)
{
    uart_startup_t link = {0};
    assert(uart_startup_begin(&link, 1, 0));
    uart_startup_stop(&link);
    prove_all(&link, 1);
    assert(!uart_startup_commit(&link, 1, UART_HANDSHAKE_US, true));
    assert(link.state == WAIT_LINK);

    assert(uart_startup_begin(&link, 2, UART_HANDSHAKE_US));
    uart_startup_stop(&link);
    prove_all(&link, 2);
    assert(uart_startup_commit(&link, 2, UART_HANDSHAKE_US + UART_CLEAN_US, true));

    // An error during a reconnect is operational because this boot was ready.
    assert(uart_startup_begin(&link, 3, 3000000));
    assert(!uart_startup_motion_allowed(&link));
    assert(uart_startup_record_error(&link, AMR_EVENT_UART_FIFO_OVF, 3000001));
    uart_startup_latch_fault(&link);
    uart_startup_stop(&link);
    assert(!uart_startup_begin(&link, 4, 4000000));
    assert(!uart_startup_motion_allowed(&link));
    assert(link.state == FAULT_LATCHED && link.operational_total == 1 && link.startup_total == 0);
}

static void test_control_fault_before_link(void)
{
    uart_startup_t link = {0};
    uart_startup_latch_fault(&link);
    assert(!uart_startup_record_error(&link, AMR_EVENT_BAD_BYTE, 1));
    uart_startup_stop(&link);
    assert(link.state == FAULT_LATCHED);
    assert(!uart_startup_begin(&link, 1, 2));
    assert(link.startup_total == 1 && link.operational_total == 0);
}

int main(void)
{
    test_clean_start();
    test_startup_errors_and_replay();
    test_expire_and_persistent_fault();
    test_control_fault_before_link();
    return 0;
}
