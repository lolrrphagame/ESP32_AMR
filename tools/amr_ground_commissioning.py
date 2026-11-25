#!/usr/bin/env python3
"""Publish one explicitly confirmed AMR commissioning profile and record its telemetry."""
import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path
import sys
import time

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from amr_ground_protocol import (
    SOURCE_TOPICS, STATUS_TOPIC, AuthorityError, DDSStamp, check_dds_publisher, command_qos, graph_publishers,
    metadata_executor, token_valid,
)
from amr_ros2_bridge_v3 import DEADMAN_S


@dataclass(frozen=True)
class Profile:
    mode: str
    linear: float
    angular: float
    duration: float
    source_loss: bool = False


PROFILES = {
    'raised': Profile('raised', .04, 0., 2.),
    'forward-3s': Profile('ground', .04, 0., 3.),
    'forward-5s': Profile('ground', .05, 0., 5.),
    'left': Profile('ground', 0., .25, 2.),
    'right': Profile('ground', 0., -.25, 2.),
    'normal-stop': Profile('raised', .04, 0., 1.),
    'source-loss': Profile('raised', .04, 0., 1., source_loss=True),
}
CSV_FIELDS = (
    'elapsed_s', 'test', 'phase', 'state', 'authority_state', 'session_state',
    'command_source', 'session_time_remaining_s', 'source_age_s', 'adapter_receipt_age_s',
    'feedback_age_s', 'requested_body', 'limited_body', 'requested_rpm', 'limited_rpm',
    'acknowledged_rpm', 'measured_rpm', 'pwm_duty_percent', 'encoder_ticks', 'boot_id',
    'encoder_continuity_valid', 'firmware_run_confirmed', 'firmware_running',
    'firmware_armed', 'firmware_stationary', 'stop_verified', 'stop_reason',
    'fault_reason', 'firmware_fault_reason', 'result',
)


class CommissioningRun:
    def __init__(self, name, session, publish, now=time.monotonic):
        self.name, self.profile, self.session = name, PROFILES[name], session
        self.publish, self.now = publish, now
        self.started = now()
        self.phase = 'WAIT_READY'
        self.phase_at = self.started
        self.status_at = None
        self.status = None
        self.rows = []
        self.motion_confirmed = False
        self.quiet = None
        self.result = None

    def fail(self, reason):
        if self.result is None:
            self.result = reason

    def record(self, status):
        if self.result is not None:
            return
        if status.get('session_id') != self.session:
            return
        if status.get('bridge_mode') != self.profile.mode or status.get('command_source') != 'commissioning':
            self.fail('Wrong bridge mode or source for selected profile')
            return
        self.status, self.status_at = status, self.now()
        self.rows.append(dict(status, elapsed_s=self.now() - self.started, test=self.name, phase=self.phase))
        if self.phase == 'MOTION' and status.get('firmware_run_confirmed') is True:
            self.motion_confirmed = True
        if self.phase == 'WAIT_FAULT' and status.get('state') == 'FAULT_LATCHED':
            reason = status.get('fault_reason') or ''
            if 'arrival gap' not in reason and 'deadman' not in reason:
                self.fail('Expected source loss, got ' + reason)
                return
            ticks = status.get('encoder_ticks')
            quiet = (status.get('authority_state') == 'INHIBITED'
                     and status.get('stop_verified') is True and status.get('firmware_stationary') is True
                     and status.get('firmware_armed') is False and status.get('firmware_running') is False
                     and status.get('encoder_continuity_valid') is True
                     and status.get('pwm_duty_percent') == [0, 0] and ticks is not None)
            if not quiet:
                self.quiet = None
            elif self.quiet is None or self.quiet[1] != ticks:
                self.quiet = self.now(), ticks
            elif self.now() - self.quiet[0] >= .25:
                self.result = 'PASS'
        elif status.get('state') == 'FAULT_LATCHED':
            self.fail(status.get('fault_reason') or 'Bridge fault')
        if (self.phase == 'WAIT_STOP' and status.get('stationary_ready') is True
                and status.get('stop_reason') == 'NORMAL_ZERO' and self.motion_confirmed):
            self.result = 'PASS'

    def tick(self):
        if self.result is not None:
            return
        now = self.now()
        elapsed = now - self.phase_at
        if self.status_at is not None and now - self.status_at >= DEADMAN_S:
            self.fail('Bridge status stream lost')
            return
        if self.phase == 'WAIT_READY':
            if elapsed >= 15:
                self.fail('Timed out waiting for authorized stationary READY')
            elif (self.status and self.status.get('stationary_ready') is True
                  and self.status.get('authority_state') == 'OWNED'
                  and self.status.get('clock_status') == 'VERIFIED'):
                self.phase, self.phase_at = 'NEUTRAL', now
            return
        if self.phase in ('NEUTRAL', 'MOTION') and (
                self.status.get('authority_state') != 'OWNED'
                or self.status.get('clock_status') != 'VERIFIED'):
            self.fail('Bridge authority lost during test')
            return
        if self.phase == 'NEUTRAL':
            if elapsed < .45:
                self.publish(0., 0.)
                return
            self.phase, self.phase_at = 'MOTION', now
            elapsed = 0.
        if self.phase == 'MOTION':
            if elapsed >= .8 and not self.motion_confirmed:
                self.fail('Firmware never confirmed RUN')
            elif elapsed < self.profile.duration:
                self.publish(self.profile.linear, self.profile.angular)
            else:
                self.phase = 'WAIT_FAULT' if self.profile.source_loss else 'WAIT_STOP'
                self.phase_at = now
                if self.phase == 'WAIT_STOP':
                    self.publish(0., 0.)
        elif self.phase in ('WAIT_STOP', 'WAIT_FAULT'):
            if elapsed >= 3:
                self.fail('Final STOP and stationary feedback unverified')
            elif self.phase == 'WAIT_STOP':
                self.publish(0., 0.)

    def write_csv(self, stream):
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(self.rows)
        writer.writerow({'elapsed_s': self.now() - self.started, 'test': self.name,
                         'phase': 'RESULT', 'result': self.result})


def run_ros(name, session, stream):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String

    node = executor = run = None
    rclpy.init(args=[])
    try:
        executor = metadata_executor()
        node = rclpy.create_node('amr_ground_commissioning')
        output = node.create_publisher(Twist, SOURCE_TOPICS['commissioning'], command_qos())

        def publish(linear, angular):
            msg = Twist()
            msg.linear.x, msg.angular.z = linear, angular
            output.publish(msg)

        run = CommissioningRun(name, session, publish)
        status_owner = None

        def on_status(msg, info):
            nonlocal status_owner
            try:
                status = json.loads(msg.data)
                if not isinstance(status, dict) or status.get('session_id') != session:
                    return
                DDSStamp.read(info, time.time_ns(), remote=True)
                publishers = graph_publishers(node, STATUS_TOPIC)
                if len(publishers) != 1 or (status_owner is not None and publishers[0] != status_owner):
                    raise AuthorityError('Bridge status publisher conflict')
                status_owner = publishers[0]
                check_dds_publisher(info, status_owner)
                run.record(status)
            except (ValueError, AuthorityError) as exc:
                run.fail(str(exc))

        executor.subscribe(node, String, STATUS_TOPIC, on_status, command_qos())
        executor.add_node(node)
        next_tick = time.monotonic()
        while rclpy.ok() and run.result is None:
            executor.spin_once(timeout_sec=max(0., next_tick - time.monotonic()))
            if time.monotonic() >= next_tick:
                run.tick()
                next_tick = time.monotonic() + .05
    except (KeyboardInterrupt, ExternalShutdownException):
        if run:
            run.fail('Operator cancelled or ROS shut down')
    except (AuthorityError, RuntimeError, OSError) as exc:
        if run:
            run.fail(str(exc))
        else:
            print(str(exc), file=sys.stderr)
    finally:
        if run:
            run.fail('ROS stopped before verified completion')
            run.write_csv(stream)
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    if run and run.result == 'PASS':
        print('PASS: verified final STOP and stationary feedback.')
        return 0
    if run:
        print(f'Run failed: {run.result}. If stationary STOP is unverified, cut motor power physically.',
              file=sys.stderr, flush=True)
    return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test', choices=PROFILES, required=True)
    parser.add_argument('--session', required=True)
    parser.add_argument('--csv', type=Path, required=True)
    args = parser.parse_args()
    if not token_valid(args.session):
        parser.error('--session must be the 32-character bridge session ID')
    if not sys.stdin.isatty():
        return 1
    profile = PROFILES[args.test]
    token = f'RUN {args.test} {args.session}'
    setup = 'securely raised wheels' if profile.mode == 'raised' else 'a clear floor area'
    try:
        answer = input(f'Confirm {setup} and physical motor cutoff. Type {token}: ')
    except (EOFError, KeyboardInterrupt):
        return 1
    if answer.strip() != token:
        return 1
    try:
        with args.csv.open('x', newline='') as stream:
            return run_ros(args.test, args.session, stream)
    except (OSError, ImportError) as exc:
        print(f'Commissioning tool stopped: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
