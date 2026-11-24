#!/usr/bin/env python3
"""Run one supervised wheels-raised low-speed candidate and save both wheels to CSV."""
import argparse
import csv
from dataclasses import dataclass, field
from pathlib import Path
import signal
import statistics
import sys
import time

# Allow direct invocation from tools/ or from another working directory.
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from amr_ros2_bridge_v3 import LOOP_PERIOD_S, SETTLE_TIMEOUT_S, State
from amr_ros2_bridge_v3_1 import (
    LOW_CANDIDATES, MPS_PER_RPM, BridgeV31, Mode, confirm_bench, exclusive_uart,
)

MAX_DURATION_S = 10.0
START_PROGRESS_TICKS = 3
FIELDS = (
    'record', 'wheel', 'elapsed_s', 'state', 'requested_rpm', 'acknowledged_rpm',
    'measured_rpm', 'pwm_duty_percent', 'encoder_ticks', 'encoder_delta',
    'firmware_run_confirmed', 'time_to_start_s', 'mean_abs_error_rpm',
    'max_abs_error_rpm', 'tracking_samples', 'firmware_fault', 'event',
    'stationary_stop_verified',
)


@dataclass
class Run:
    candidate: int
    initial_ticks: tuple | None = None
    final_ticks: tuple | None = None
    samples: list = field(default_factory=list)
    event: str | None = None
    firmware_fault: str | None = None
    stop_verified: bool = False

    def write_csv(self, stream):
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for index, wheel in enumerate(('left', 'right')):
            started = None
            errors = []
            for elapsed, status in self.samples:
                measured = status['measured_rpm'][index]
                duty = status['pwm_duty_percent'][index]
                ticks = status['encoder_ticks'][index]
                delta = ticks - self.initial_ticks[index]
                confirmed = status['firmware_run_confirmed']
                ack = status['acknowledged_rpm']
                if confirmed:
                    errors.append(abs(self.candidate - measured))
                    signed_delta = delta if self.candidate > 0 else -delta
                    if started is None and duty > 0 and signed_delta >= START_PROGRESS_TICKS:
                        started = elapsed
                writer.writerow({
                    'record': 'sample', 'wheel': wheel, 'elapsed_s': elapsed,
                    'state': status['state'], 'requested_rpm': self.candidate,
                    'acknowledged_rpm': ack[index] if ack else None,
                    'measured_rpm': measured, 'pwm_duty_percent': duty,
                    'encoder_ticks': ticks, 'encoder_delta': delta,
                    'firmware_run_confirmed': confirmed,
                    'firmware_fault': status['firmware_fault_reason'],
                    'event': status['fault_reason'],
                })
            delta = (self.final_ticks[index] - self.initial_ticks[index]
                     if self.final_ticks is not None and self.initial_ticks is not None else None)
            writer.writerow({
                'record': 'summary', 'wheel': wheel, 'requested_rpm': self.candidate,
                'encoder_delta': delta, 'time_to_start_s': started,
                'mean_abs_error_rpm': statistics.fmean(errors) if errors else None,
                'max_abs_error_rpm': max(errors) if errors else None,
                'tracking_samples': len(errors), 'firmware_fault': self.firmware_fault,
                'event': self.event or ('no encoder-confirmed start' if started is None else None),
                'stationary_stop_verified': self.stop_verified,
            })


def characterize(bridge, duration, run, sleep=time.sleep):
    """Collect one run with injected clock/UART support for offline verification."""
    try:
        bridge.start()
        deadline = bridge.now() + SETTLE_TIMEOUT_S
        while bridge.state == State.STOPPING and bridge.now() < deadline:
            bridge.tick()
            if bridge.state == State.STOPPING:
                sleep(LOOP_PERIOD_S)
        if bridge.state != State.READY:
            raise RuntimeError(bridge.fault_reason or 'bridge did not reach stationary READY')
        run.initial_ticks = bridge.feedback.ticks
        components = (run.candidate * MPS_PER_RPM, 0., 0., 0., 0., 0.)
        ack_at = None
        last_sample_at = None
        start_deadline = bridge.now() + 1.0
        # Separate the first command timestamp from the READY cutoff.
        sleep(LOOP_PERIOD_S)
        while bridge.state != State.FAULT_LATCHED:
            cycle_at = bridge.now()
            if ack_at is not None and cycle_at >= ack_at + duration:
                break
            if ack_at is None and cycle_at >= start_deadline:
                raise RuntimeError('initial velocity acknowledgement timed out')
            bridge.command(components, bridge.wall_ns())
            bridge.tick()
            if ack_at is None:
                ack_at = bridge.acknowledged_at
            if (ack_at is not None and bridge.feedback_at != last_sample_at
                    and bridge.feedback_at >= ack_at):
                last_sample_at = bridge.feedback_at
                run.samples.append((last_sample_at - ack_at, bridge.status()))
            until = cycle_at + LOOP_PERIOD_S
            if ack_at is not None:
                until = min(until, ack_at + duration)
            if bridge.state != State.FAULT_LATCHED:
                sleep(max(0., until - bridge.now()))
        run.event = bridge.fault_reason
    except KeyboardInterrupt:
        run.event = 'operator interrupted'
    except (RuntimeError, OSError) as exc:
        run.event = str(exc)
    finally:
        # This path is required even when startup or motion faults latched.
        try:
            stopped = bridge.verify_stationary_stop(sleep)
        except (KeyboardInterrupt, RuntimeError, OSError) as exc:
            stop_error = f'final stationary STOP unverified ({exc}). CUT MOTOR POWER PHYSICALLY'
            run.event = f'{run.event}. {stop_error}' if run.event else stop_error
            print(stop_error, file=sys.stderr, flush=True)
        else:
            run.stop_verified = True
            run.final_ticks = stopped.ticks
        run.firmware_fault = bridge.feedback.reason if bridge.feedback else None
    return run.stop_verified and run.event is None and run.firmware_fault is None


def handle_sigterm(_signal, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/serial0')
    parser.add_argument('--rpm', required=True, type=int, choices=LOW_CANDIDATES)
    parser.add_argument('--direction', required=True, choices=('forward', 'reverse'))
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--csv', required=True, type=Path, help='output path for this single run')
    args = parser.parse_args()
    if not 0.5 <= args.duration <= MAX_DURATION_S:
        parser.error(f'--duration must be between 0.5 and {MAX_DURATION_S:g} seconds')
    signed_rpm = args.rpm if args.direction == 'forward' else -args.rpm
    if not confirm_bench(Mode.LOW, signed_rpm):
        return 1
    run = Run(signed_rpm)
    signal.signal(signal.SIGTERM, handle_sigterm)
    try:
        with args.csv.open('x', newline='') as stream:
            try:
                with exclusive_uart(args.port) as uart:
                    bridge = BridgeV31(uart, Mode.LOW, signed_rpm)
                    success = characterize(bridge, args.duration, run)
            except (KeyboardInterrupt, RuntimeError, OSError, ImportError) as exc:
                run.event = str(exc) or 'operator interrupted'
                success = False
            run.write_csv(stream)
    except OSError as exc:
        print(f'CSV error: {exc}', file=sys.stderr)
        return 1
    if success:
        print(f'Stationary STOP verified. CSV saved to {args.csv}')
    else:
        print(f'Run unsuccessful: {run.event or run.firmware_fault}. CSV saved to {args.csv}',
              file=sys.stderr)
    return int(not success)


if __name__ == '__main__':
    sys.exit(main())
