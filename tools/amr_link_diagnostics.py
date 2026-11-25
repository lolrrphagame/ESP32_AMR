#!/usr/bin/env python3
"""Run a bounded, no-motion ROS link check on an isolated domain and topic prefix."""
import argparse
import json
from pathlib import Path
import sys
import time

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from amr_ground_protocol import (
    AuthorityError, DDSStamp, check_dds_publisher, command_qos, metadata_executor, token_valid,
)
from amr_link_metrics import LinkMetrics, PeriodicLinkReport, summarize
from amr_ros2_bridge_v3 import LOOP_PERIOD_S, State
from amr_ros2_bridge_v3_2 import BridgeV32, attach_bridge_link
from tools.amr_cmd_vel_adapter import CommandAdapter, attach_adapter


class DiagnosticBridge(BridgeV32):
    """Only exercise the status and authority code. There is no UART transport."""
    def __init__(self, session, metrics, **kwargs):
        super().__init__(None, 'raised', 'commissioning', metrics=metrics, **kwargs)
        self.session = session
        self.state = State.READY
        self.protocol = '3.1'
        self.peer_complete = False

    def control(self, message, info):
        if message.get('kind') != 'diagnostic_done':
            return super().control(message, info)
        if (self.state == State.FAULT_LATCHED or self.activation is None
                or message.get('session') != self.session or message.get('activation') != self.activation
                or message.get('source') != self.source):
            return
        try:
            self.require_authority()
            check_dds_publisher(info, self.owner_endpoints[1])
            DDSStamp.read(info, self.clock.check(), remote=True)
        except AuthorityError as exc:
            self.latch(str(exc))
        else:
            self.peer_complete = True
            self.metrics.latest['clean_peer_completion'] = True

    def latch(self, reason):
        if self.state != State.FAULT_LATCHED:
            self.state = State.FAULT_LATCHED
            self.fault_reason = reason
            self.metrics.latest['fault_reason'] = reason

    def tick(self):
        if self.activation is not None and self.state != State.FAULT_LATCHED:
            try:
                self.require_authority()
            except AuthorityError as exc:
                self.latch(str(exc))

    def command(self, *args):
        raise AssertionError('diagnostic bridge has no motion input')

    def send_velocity(self, *args, **kwargs):
        raise AssertionError('diagnostic bridge cannot send velocity')


def attach_diagnostic(node, executor, role, session, metrics, reporter, pause_at, pause_for):
    from geometry_msgs.msg import TwistStamped
    from rclpy.clock import Clock, ClockType
    from std_msgs.msg import String

    prefix = '/amr_link_diagnostics/s_' + session
    ground, control, status, raw = (f'{prefix}/{topic}'
                                    for topic in ('ground', 'control', 'status', 'no_raw_input'))
    if role == 'bridge':
        model = DiagnosticBridge(session, metrics)
        before_tick, publish = attach_bridge_link(node, executor, model, status_topic=status,
                                                      ground_topic=ground, control_topic=control)
        created = time.monotonic()

        def tick():
            started = time.monotonic()
            metrics.timer(LOOP_PERIOD_S, started)
            before_tick()
            model.tick()
            paused = pause_at is not None and pause_at <= started - created < pause_at + pause_for
            if not paused:
                publish()
            else:
                metrics.event('intentional_status_pause_ticks')
            metrics.sample('timer_execution_ms', (time.monotonic() - started) * 1000)
            reporter.poll()

        node.create_timer(LOOP_PERIOD_S, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        return model, None
    else:
        control_pub = node.create_publisher(String, control, command_qos())
        # Discovery must see the same endpoint type. This publisher is never
        # used and the adapter is wired to an isolated, writerless raw topic.
        node.create_publisher(TwistStamped, ground, command_qos())

        def no_velocity(*args):
            raise AssertionError('diagnostic adapter cannot publish velocity')

        def send_control(data):
            control_pub.publish(String(data=json.dumps(data, separators=(',', ':'))))

        model = CommandAdapter(session, 'commissioning', no_velocity, send_control, metrics=metrics)
        attach_adapter(node, executor, model, raw, status, reporter)
        return model, lambda: send_control(model.control_payload('diagnostic_done'))


def run(args, metrics):
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=[], domain_id=args.domain_id)
    node = executor = None
    try:
        executor = metadata_executor()
        node = rclpy.create_node('amr_link_diagnostics_' + args.role)
        model, finish = attach_diagnostic(node, executor, args.role, args.session, metrics,
                                  PeriodicLinkReport(metrics), args.pause_status_at,
                                  args.pause_status_for)
        executor.add_node(node)
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(.1, max(0., deadline - time.monotonic())))
            if args.role == 'bridge' and model.peer_complete:
                break
        if not rclpy.ok():
            metrics.latest['outcome'] = 'INTERRUPTED'
            return 1
        if args.role == 'bridge':
            metrics.latest['last_clock_proof_age_s'] = model.clock_proof_age()
        healthy = metrics.counts['clock_success'] if args.role == 'bridge' else metrics.counts['status_accepted']
        if args.role == 'adapter':
            healthy = healthy and model.bridge_allows_commands
        metrics.latest['outcome'] = 'PASS' if healthy and not model.fault_reason else 'FAIL'
        if not healthy and not model.fault_reason:
            metrics.latest['completion_reason'] = 'authority was not verified at completion'
        if finish is not None and metrics.latest['outcome'] == 'PASS':
            finish()
            # Keep diagnostic endpoints alive briefly so teardown cannot race
            # the completion message. No motion traffic is published.
            time.sleep(.35)
        return int(metrics.latest['outcome'] != 'PASS')
    except (KeyboardInterrupt, ExternalShutdownException):
        metrics.latest['outcome'] = 'INTERRUPTED'
        return 1
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=('bridge', 'adapter'))
    parser.add_argument('--domain-id', type=int, required=True, help='unused isolated ROS domain')
    parser.add_argument('--session', required=True)
    parser.add_argument('--duration', type=float, default=120)
    parser.add_argument('--pause-status-at', type=float, help='bridge-only elapsed time for deliberate loss')
    parser.add_argument('--pause-status-for', type=float, default=.35)
    parser.add_argument('--json', type=Path, required=True)
    args = parser.parse_args()
    if not (token_valid(args.session) and 1 <= args.domain_id <= 232 and 1 <= args.duration <= 600):
        parser.error('require a 32-character hex session, isolated domain 1–232 and duration 1–600 s')
    if args.pause_status_at is not None and (
            args.role != 'bridge' or not 0 <= args.pause_status_at < args.duration
            or not 0 < args.pause_status_for <= args.duration - args.pause_status_at):
        parser.error('status pause must be within the bridge run')
    metrics = LinkMetrics(args.role)
    metrics.latest['fault_reason'] = None
    for key in (('status_taken', 'status_callbacks', 'status_accepted', 'status_ignored', 'status_rejected')
                if args.role == 'adapter' else ('clock_success', 'clock_failure', 'clock_ignored')):
        metrics.counts[key] = 0
    try:
        with args.json.open('x') as stream:
            try:
                result = run(args, metrics)
            except (RuntimeError, OSError, ImportError) as exc:
                metrics.latest.update(outcome='ERROR', completion_reason=str(exc))
                result = 1
            summary = summarize(metrics.snapshot())
            json.dump(summary, stream, indent=2, allow_nan=False)
            stream.write('\n')
    except OSError as exc:
        print(exc, file=sys.stderr)
        return 1
    print(f'{metrics.latest.get("outcome")}. Summary saved to {args.json}')
    return result


if __name__ == '__main__':
    sys.exit(main())
