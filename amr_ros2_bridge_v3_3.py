#!/usr/bin/env python3
"""AMR Bridge V3.3 with UART safe startup for ROS 2 Humble."""
import re
import secrets
import sys
import time
from dataclasses import dataclass
from functools import partial

from amr_ros2_bridge_v3 import ProtocolError, State, ZERO
from amr_ros2_bridge_v3_2 import BridgeV32, main as commissioning_cli, run_ros as run_commissioning

VERSION = 'VERSION PV=3.3 CAP=ARM_CONT,STICKY_FAULT,STATIONARY,BOOT_ID,SAFE_STARTUP'
VERSION_READ_ONLY = 'VERSION PV=3.3 CAP=STICKY_FAULT,STATIONARY,BOOT_ID,SAFE_STARTUP'
CAPS = 'CAPS PV=3.3 CAP=SAFE_STARTUP,UART_DIAG'
LOW_CAPS = CAPS + ',ARM_LOW,LOW_FIXED_PAIR,LOW_SESSION_15S'
STARTUP_TIMEOUT_S = 5.0
PROOF_INTERVAL_S = 0.13
PROOF_COUNT = 5
LINK_STATES = ('WAIT_LINK', 'HANDSHAKE', 'LINK_READY', 'FAULT_LATCHED')
UART_EVENTS = (
    'UART_BREAK', 'UART_BUFFER_FULL', 'UART_FIFO_OVF', 'UART_FRAME_ERR', 'UART_PARITY_ERR',
    'UART_RX', 'FRAME_GAP', 'BAD_BYTE', 'FRAME_OVERFLOW', 'BAD_ARM_LOW', 'BAD_V',
    'COMMAND', 'LINK_PROTOCOL', 'UART_QUEUE_FULL',
)
UINT32_MAX = (1 << 32) - 1


def uint(value, maximum=UINT32_MAX):
    if re.fullmatch(r'[0-9]+', value) and int(value) <= maximum:
        return int(value)
    raise ValueError('invalid unsigned counter')


def fields(reply):
    items = reply.split()[1:]
    result = dict(item.split('=', 1) for item in items)
    if len(items) != len(result):
        raise ValueError('duplicate fields')
    return result


@dataclass(frozen=True)
class UartDiagnostics:
    boot: str
    state: str
    startup_errors: int
    operational_errors: int
    last_event: str | None
    last_error_us: int
    startup_counts: dict
    operational_counts: dict

    @classmethod
    def parse(cls, reply):
        try:
            data = fields(reply)
            boot, state, ready, last = (data[key] for key in ('BOOT', 'LS', 'READY', 'LAST'))
            if (not re.fullmatch(r'[0-9a-f]{8}', boot) or state not in LINK_STATES
                    or ready != str(int(state == 'LINK_READY')) or last not in ('NONE', *UART_EVENTS)):
                raise ValueError('invalid UART link state or event')
            startup, operational = uint(data['SU']), uint(data['UE'])
            stamp = uint(data['US'], (1 << 63) - 1)
            counts = []
            for key, total in (('S', startup), ('O', operational)):
                values = [uint(value) for value in data[key].split(',')]
                if len(values) != len(UART_EVENTS) or min(sum(values), UINT32_MAX) != total:
                    raise ValueError('inconsistent UART counters')
                counts.append(dict(zip(UART_EVENTS, values)))
            if ((last == 'NONE') != (startup == operational == 0)
                    or (last == 'NONE' and stamp != 0)
                    or (last != 'NONE' and counts[0][last] + counts[1][last] == 0)
                    or (operational and state != 'FAULT_LATCHED')):
                raise ValueError('inconsistent last UART event or operational fault')
            return cls(boot, state, startup, operational, None if last == 'NONE' else last, stamp, *counts)
        except (KeyError, ValueError) as exc:
            raise ProtocolError(f'invalid UART_DIAG: {reply!r}') from exc


class BridgeV33(BridgeV32):
    extended_version = '3.3'

    def __init__(self, *args, sleep=time.sleep, **kwargs):
        super().__init__(*args, **kwargs)
        self.sleep = sleep
        self.uart_diag = None
        self.uart_startup_state = None
        self.uart_startup_errors = None
        self.next_diag_at = 0.0

    def _parse_feedback(self, reply):
        try:
            data = fields(reply)
            state = data['LS']
            startup_errors = uint(data['SU'])
            if data['PV'] != self.extended_version or state not in LINK_STATES:
                raise ValueError('invalid Q link state or protocol')
            if ((data['FL'] == '1') != (state == 'FAULT_LATCHED')
                    or (state != 'LINK_READY' and (data['A'] != '0' or data['RUN'] != '0'))):
                raise ValueError('inconsistent Q link state')
        except (KeyError, ValueError) as exc:
            raise ProtocolError(f'invalid V3.3 Q: {reply!r}') from exc
        feedback = super()._parse_feedback(reply)
        self.uart_startup_state = state
        self.uart_startup_errors = startup_errors
        return feedback

    def read_uart_diag(self):
        reply = self.uart.exchange('UART_DIAG', 'UART_DIAG ', prefix=True,
                                   reply_limit=512, timeout=0.15)
        diag = UartDiagnostics.parse(reply)
        if self.feedback is not None and diag.boot != self.feedback.boot:
            raise ProtocolError('firmware boot changed between Q and UART_DIAG')
        self.uart_diag = diag
        return diag

    def handshake(self, deadline):
        version = self.uart.exchange('VER', 'VERSION ', prefix=True, drain=True)
        if version not in (VERSION, VERSION_READ_ONLY):
            raise ProtocolError('firmware V3.3 SAFE_STARTUP version required')
        if self.commissioning and version != VERSION:
            raise ProtocolError('firmware does not advertise continuous commissioning')
        capabilities = self.uart.exchange('CAPS', CAPS, prefix=True)
        if capabilities not in (CAPS, LOW_CAPS):
            raise ProtocolError('firmware V3.3 SAFE_STARTUP and UART_DIAG capabilities required')

        # Stop immediately on healthy reconnects as well as on initial startup.
        self.uart.exchange('STOP', 'OK STOP', drain=True)
        client = secrets.token_hex(8)
        reply = self.uart.exchange(f'LINK_BEGIN {client}', 'LINK_BEGIN ', prefix=True)
        match = re.fullmatch(r'LINK_BEGIN ([0-9a-f]{8}) ([0-9a-f]{16}) ([0-9a-f]{16})', reply)
        if match is None or match[2] != client:
            raise ProtocolError('stale or invalid LINK_BEGIN challenge')
        boot, challenge = match[1], match[3]
        self.uart.exchange('STOP', 'OK STOP')
        for step in range(1, PROOF_COUNT + 1):
            if self.now() + PROOF_INTERVAL_S >= deadline:
                raise ProtocolError('UART startup deadline expired')
            self.sleep(PROOF_INTERVAL_S)
            request = f'LINK_STEP {boot} {challenge} {step}'
            self.uart.exchange(request, f'OK {request}')
        request = f'LINK_COMMIT {boot} {challenge}'
        self.uart.exchange(request, f'OK {request}')
        feedback = self.query()
        diag = self.read_uart_diag()
        if (self.now() >= deadline or boot != feedback.boot or self.uart_startup_state != 'LINK_READY'
                or diag.state != 'LINK_READY' or diag.startup_errors != self.uart_startup_errors
                or diag.operational_errors != 0 or feedback.counters != (0, 0)
                or feedback.fault or feedback.armed or feedback.running or feedback.duty != ZERO
                or not feedback.stationary or not self.encoder_valid):
            raise ProtocolError('UART startup did not confirm a healthy stationary link')
        return feedback, capabilities

    def start(self):
        if self.baseline is not None or self.state == State.FAULT_LATCHED:
            return
        deadline = self.now() + STARTUP_TIMEOUT_S
        error = 'firmware V3.3 SAFE_STARTUP required'
        while self.now() < deadline:
            try:
                feedback, capabilities = self.handshake(deadline)
            except (RuntimeError, OSError) as exc:
                error = str(exc)
                self.sleep(min(0.05, max(0.0, deadline - self.now())))
                continue
            try:
                self.protocol = self.extended_version
                self.extended_status = True
                self.low_capable = capabilities == LOW_CAPS
                self.baseline = feedback.boot, feedback.counters
                self.check_feedback(armed=False, running=False)
                if self.commissioning:
                    self.begin_stop()
                else:
                    self.stop_verified = True
            except (RuntimeError, OSError) as exc:
                self.latch(str(exc))
            return
        self.latch(f'UART startup failed: {error}')

    def check_feedback(self, **kwargs):
        super().check_feedback(**kwargs)
        if self.uart_startup_state != 'LINK_READY':
            raise ProtocolError('firmware UART link lost readiness')

    def tick(self):
        super().tick()
        # UART_DIAG can take longer than Q. Poll only after revoking authority.
        if self.state == State.FAULT_LATCHED and self.now() >= self.next_diag_at:
            try:
                self.read_uart_diag()
            except (RuntimeError, OSError):
                self.uart_diag = None
            self.next_diag_at = self.now() + 1.0

    def status(self):
        payload = super().status()
        diag = self.uart_diag
        if self.feedback is None or (diag and (diag.boot != self.feedback.boot
                or diag.state != self.uart_startup_state
                or diag.startup_errors != self.uart_startup_errors
                or diag.operational_errors != self.feedback.counters[0])):
            diag = None
        payload.update(
            version='3.3',
            uart_startup_state=self.uart_startup_state,
            uart_link_ready=self.uart_startup_state == 'LINK_READY',
            uart_startup_errors=self.uart_startup_errors,
            uart_operational_errors=self.feedback.counters[0] if self.feedback else None,
            uart_last_error_event=diag.last_event if diag else None,
            uart_last_error_us=diag.last_error_us if diag else None,
        )
        return payload


run_ros = partial(run_commissioning, bridge_class=BridgeV33)


def main():
    return commissioning_cli(runner=run_ros, description=__doc__)


if __name__ == '__main__':
    sys.exit(main())
