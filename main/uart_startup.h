#pragma once

#include <stdbool.h>
#include <stdint.h>

#define UART_CLEAN_US 500000LL
#define UART_HANDSHAKE_US 2000000LL
#define UART_PROOF_COUNT 5

typedef enum {
    WAIT_LINK,
    HANDSHAKE,
    LINK_READY,
    FAULT_LATCHED,
} uart_link_state_t;

// Keep the wire order in UART_DIAG stable.
#define AMR_UART_EVENTS(X) \
    X(UART_BREAK) X(UART_BUFFER_FULL) X(UART_FIFO_OVF) X(UART_FRAME_ERR) X(UART_PARITY_ERR) \
    X(UART_RX) X(FRAME_GAP) X(BAD_BYTE) X(FRAME_OVERFLOW) X(BAD_ARM_LOW) X(BAD_V) \
    X(COMMAND) X(LINK_PROTOCOL) X(UART_QUEUE_FULL)

#define AMR_EVENT_ENUM(name) AMR_EVENT_##name,
typedef enum {
    AMR_UART_EVENTS(AMR_EVENT_ENUM)
    AMR_EVENT_COUNT,
} amr_uart_event_t;
#undef AMR_EVENT_ENUM

typedef struct {
    uart_link_state_t state;
    bool established;  // First LINK_READY is a one-way boundary until ESP reset.
    bool stop_seen;
    unsigned proofs;
    int64_t started_us;
    uint64_t challenge;
    uint32_t startup_total;
    uint32_t operational_total;
    uint32_t startup[AMR_EVENT_COUNT];
    uint32_t operational[AMR_EVENT_COUNT];
    const char *last_event;
    int64_t last_error_us;
} uart_startup_t;

static inline bool uart_startup_motion_allowed(const uart_startup_t *link)
{
    return link->state == LINK_READY;
}

static inline void uart_startup_cancel(uart_startup_t *link)
{
    if (link->state == HANDSHAKE) link->state = WAIT_LINK;
    link->stop_seen = false;
    link->proofs = 0;
    link->challenge = 0;
}

static inline void uart_startup_tick(uart_startup_t *link, int64_t now)
{
    if (link->state == HANDSHAKE && now - link->started_us >= UART_HANDSHAKE_US)
        uart_startup_cancel(link);
}

static inline bool uart_startup_begin(uart_startup_t *link, uint64_t challenge, int64_t now)
{
    if (link->state == FAULT_LATCHED) return false;
    uart_startup_cancel(link);
    link->state = HANDSHAKE;
    link->challenge = challenge;
    link->started_us = now;
    return true;
}

static inline void uart_startup_stop(uart_startup_t *link)
{
    if (link->state == HANDSHAKE) {
        link->stop_seen = true;
        link->proofs = 0;
    }
}

static inline bool uart_startup_prove(uart_startup_t *link, uint64_t challenge, unsigned proof)
{
    if (link->state != HANDSHAKE || link->challenge != challenge || !link->stop_seen ||
        proof != link->proofs + 1 || proof > UART_PROOF_COUNT)
        return false;
    link->proofs = proof;
    return true;
}

static inline bool uart_startup_commit(uart_startup_t *link, uint64_t challenge, int64_t now,
                                       bool safely_stationary)
{
    uart_startup_tick(link, now);
    if (link->state != HANDSHAKE || link->challenge != challenge || !link->stop_seen ||
        link->proofs != UART_PROOF_COUNT || !safely_stationary ||
        now - link->started_us < UART_CLEAN_US)
        return false;
    uart_startup_cancel(link);
    link->established = true;
    link->state = LINK_READY;
    return true;
}

static inline void uart_startup_latch_fault(uart_startup_t *link)
{
    uart_startup_cancel(link);
    link->state = FAULT_LATCHED;
}

static inline bool uart_startup_record_error(uart_startup_t *link, amr_uart_event_t event, int64_t now)
{
#define AMR_EVENT_NAME(name) #name,
    static const char *const names[] = { AMR_UART_EVENTS(AMR_EVENT_NAME) };
#undef AMR_EVENT_NAME
    link->last_event = names[event];
    link->last_error_us = now;
    uint32_t *total = link->established ? &link->operational_total : &link->startup_total;
    uint32_t *counts = link->established ? link->operational : link->startup;
    if (*total != UINT32_MAX) ++*total;
    if (counts[event] != UINT32_MAX) ++counts[event];
    uart_startup_cancel(link);
    return link->established;
}
