#!/usr/bin/env python3
"""Forward a single selected Humble Twist source to a confirmed V3.2 session."""
import argparse
import json
from pathlib import Path
import secrets
import sys
import time

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from amr_ground_protocol import (
    CLOCK_OFFSET_NS, CLOCK_PROOF_S, CONTROL_TOPIC, DEADMAN_NS, DISCOVERY_S, GROUND_TOPIC, NS, SOURCE_TOPICS,
    STATUS_TOPIC, AuthorityError, CommandStamp, DDSAgeError, DDSStamp, WallClockGuard,
    arrival_monotonic, check_dds_publisher, command_qos, inspect_publishers, metadata_executor,
    status_qos, token_valid,
)
from amr_link_metrics import LinkMetrics, PeriodicLinkReport, summarize
from amr_ros2_bridge_v3 import DEADMAN_S, wheel_targets
from amr_ros2_bridge_v3_1 import twist_components


class CommandAdapter:
    def __init__(self, session, source, send_velocity, send_control, *,
                 now=time.monotonic, wall_ns=time.time_ns, activation=None, metrics=None,
                 stationary_check=False):
        if not token_valid(session) or source not in SOURCE_TOPICS:
            raise ValueError('valid bridge session and exactly one source required')
        self.session, self.source = session, source
        self.activation = activation or secrets.token_hex(16)
        self.now = now
        self.metrics = metrics
        self.clock = WallClockGuard(now, wall_ns)
        self.send_velocity, self.send_control = send_velocity, send_control
        self.stationary_check = stationary_check
        self.bridge_gid = None
        self.raw_candidate = self.raw_owner = None
        self.discovered_at = None
        self.raw_cutoff = self.clock.check()
        self.last_status_at = self.last_probe_at = None
        self.last_source = None
        self.last_motion_source = None
        self.verified_idle = False
        self.idle_status_paused = False
        self.bridge_allows_commands = False
        self.fault_reason = None

    def control_payload(self, kind, **fields):
        return dict(kind=kind, session=self.session, source=self.source,
                    activation=self.activation, **fields)

    def fault(self, reason, age=None):
        if self.fault_reason is None:
            self.fault_reason = reason
            self.bridge_allows_commands = False
            if self.metrics:
                latest = self.metrics.latest
                latest.update(fault_reason=reason, timeout_age_s=age,
                              fault_status_sequence=latest.get('status_sequence'),
                              fault_status_timestamps=latest.get('status_timestamps'),
                              fault_last_accepted_dds_received_ns=latest.get('last_accepted_dds_received_ns'))
            self.send_control(self.control_payload('fault', reason=reason))

    def observe_bridge(self, bridge):
        if len(bridge) > 1:
            self.fault('conflicting source or bridge publishers')
        elif self.bridge_gid is not None and (not bridge or bridge[0] != self.bridge_gid):
            self.fault('bridge status owner lost or changed')

    def observe(self, raw, bridge):
        if self.fault_reason:
            return
        self.observe_bridge(bridge)
        if self.fault_reason:
            return
        if len(raw) > 1:
            self.fault('conflicting source or bridge publishers')
            return
        candidate = raw[0] if raw else None
        if self.raw_owner is not None and candidate != self.raw_owner:
            self.fault('selected source owner lost or changed')
        elif candidate != self.raw_candidate:
            self.raw_candidate = candidate
            self.discovered_at = self.now() if candidate is not None else None
        elif (candidate is not None and self.raw_owner is None
              and self.now() - self.discovered_at >= DISCOVERY_S):
            self.raw_owner = candidate
            self.raw_cutoff = self.clock.check()

    def status_result(self, result, reason=None):
        if self.metrics:
            self.metrics.event('status_' + result)
            if reason is not None:
                self.metrics.event('status_reason:' + reason)
                self.metrics.latest['last_status_rejection'] = reason

    def status(self, message, info, publishers):
        self.observe_bridge(publishers)
        if self.fault_reason or len(publishers) != 1 or message.get('session_id') != self.session:
            reason = ('adapter_faulted' if self.fault_reason else
                      'status_publisher_unresolved' if len(publishers) != 1 else 'session_mismatch')
            self.status_result('ignored', reason)
            return
        if self.metrics:
            self.metrics.sequence(message.get('status_sequence'))
        version = message.get('version')
        if (message.get('command_source') != self.source or version not in ('3.2', '3.3')
                or message.get('bridge_mode') not in ('raised', 'ground')):
            self.status_result('rejected', 'mode_or_source')
            self.fault('bridge mode or selected source mismatch')
            return
        expected = message.get('adapter_activation')
        if expected is not None and expected != self.activation:
            self.status_result('rejected', 'activation_mismatch')
            self.fault('bridge is owned by a different adapter activation')
            return
        owned = expected == self.activation and message.get('authority_state') == 'OWNED'
        link_verified = (owned and message.get('clock_status') == 'VERIFIED'
                         and (version == '3.2' or message.get('uart_link_ready') is True))
        verified_idle = (link_verified
                         and message.get('state') == 'READY' and message.get('stationary_ready') is True
                         and message.get('firmware_armed') is False and message.get('firmware_running') is False
                         and message.get('firmware_fault') is False)
        try:
            wall = self.clock.check()
            check_dds_publisher(info, publishers[0])
            if not isinstance(info, dict):
                raise AuthorityError('DDS status timestamps unavailable')
            t2, sent = info.get('received_timestamp'), info.get('source_timestamp')
            if owned or message.get('state') == 'FAULT_LATCHED':
                DDSStamp.read(info, wall, remote=True)
            elif (type(t2) is not int or type(sent) is not int or sent <= 0 or t2 <= 0
                  or not 0 <= wall - t2 < DEADMAN_NS):
                raise AuthorityError('invalid or stale DDS status metadata')
        except AuthorityError as exc:
            if (isinstance(exc, DDSAgeError) and self.source == 'nav2' and self.verified_idle
                    and verified_idle and self.last_motion_source is None):
                # Keep the last valid receipt and probe. The original watchdogs
                # still run while waiting for a fresh stationary status.
                self.idle_status_paused = True
                self.bridge_allows_commands = False
                self.status_result('discarded', 'stale_idle_telemetry')
                if self.status_overdue():
                    self.fault('bridge status stream lost', self.now() - self.last_status_at)
                return
            self.status_result('rejected', str(exc))
            self.fault(str(exc))
            return
        received_at = arrival_monotonic(t2, wall, self.now())
        if self.last_status_at is not None:
            gap = received_at - self.last_status_at
            if gap >= self.status_deadline():
                self.status_result('rejected', 'dds_arrival_gap')
                self.fault('bridge status stream lost', gap)
                return
            if gap <= 0:
                self.status_result('ignored', 'out_of_order_receipt')
                return
            if gap >= DEADMAN_S:
                self.raw_cutoff = wall
                if self.metrics:
                    self.metrics.sample('idle_status_gap_ms', gap * 1000)
        if self.idle_status_paused and verified_idle:
            self.raw_cutoff = wall
            self.idle_status_paused = False
        self.verified_idle = verified_idle
        self.bridge_gid = publishers[0]
        self.last_status_at = received_at
        self.status_result('accepted')
        if self.metrics:
            self.metrics.latest.update(last_accepted_dds_received_ns=t2,
                                       bridge_reported_clock_proof_age_s=message.get('clock_proof_age_s'))
        if message.get('state') == 'FAULT_LATCHED':
            if self.metrics:
                self.metrics.latest['bridge_reported_fault_reason'] = message.get('fault_reason')
            self.fault('bridge latched a fault')
            return
        cutoff = message.get('stationary_source_cutoff_ns')
        if (self.last_motion_source is not None and owned and message.get('stationary_ready') is True
                and type(cutoff) is int and cutoff >= self.last_motion_source):
            # Only verified stationary feedback that covers the last forwarded
            # movement can restore the idle deadline.
            self.last_motion_source = None
        self.bridge_allows_commands = (link_verified and not self.idle_status_paused
                                       and message.get('state') in ('READY', 'STARTING', 'RUNNING'))
        t1 = message.get('clock_probe_ns')
        if type(t1) is not int or t1 <= 0 or not 0 <= sent - t1 < CLOCK_OFFSET_NS:
            if self.metrics:
                self.metrics.event('clock_probe_absent' if t1 is None else 'clock_probe_invalid')
            return
        if self.last_probe_at is None or self.now() - self.last_probe_at >= 0.12:
            self.last_probe_at = self.now()
            if self.metrics:
                self.metrics.event('clock_responses_sent')
            self.send_control(self.control_payload('clock', t1=t1, t2=t2, t3=self.clock.check()))

    def status_deadline(self):
        if self.source == 'nav2' and self.last_motion_source is None:
            return CLOCK_PROOF_S
        return DEADMAN_S

    def status_overdue(self):
        return self.last_status_at is not None and self.now() - self.last_status_at >= self.status_deadline()

    def sample(self, twist, info):
        if self.fault_reason or self.raw_owner is None:
            return
        if self.status_overdue():
            self.fault('bridge status stream lost', self.now() - self.last_status_at)
            return
        if self.idle_status_paused or (self.source == 'nav2' and self.last_status_at is not None
                and self.now() - self.last_status_at >= DEADMAN_S):
            return  # Pause admission while idle status is late.
        try:
            wall = self.clock.check()
            check_dds_publisher(info, self.raw_owner)
            stamp = DDSStamp.read(info, wall)
            if self.source != 'nav2' and self.last_source is not None and wall - self.last_source >= DEADMAN_NS:
                self.fault('selected source arrival gap', (wall - self.last_source) / NS)
                return
            if stamp.source <= self.raw_cutoff or (
                    self.last_source is not None and stamp.source <= self.last_source):
                return
            components = twist_components(twist)
            wheel_targets(components)
        except (ValueError, AuthorityError) as exc:
            self.fault(str(exc))
            return
        self.last_source = stamp.source
        if self.bridge_allows_commands and not self.stationary_check:
            if self.source == 'nav2' and any(components):
                self.last_motion_source = stamp.source
            header = CommandStamp(self.session, self.activation, self.source, stamp.source, stamp.received)
            self.send_velocity(twist, header)

    def tick(self):
        if self.fault_reason:
            return
        try:
            wall = self.clock.check()
            if self.status_overdue():
                self.fault('bridge status stream lost', self.now() - self.last_status_at)
            elif (self.source != 'nav2' and self.last_source is not None
                  and wall - self.last_source >= DEADMAN_NS):
                self.fault('selected source arrival gap', (wall - self.last_source) / NS)
        except AuthorityError as exc:
            self.fault(str(exc))


def attach_adapter(node, executor, adapter, raw_topic, status_topic=STATUS_TOPIC, reporter=None):
    from geometry_msgs.msg import Twist
    from rclpy.clock import Clock, ClockType
    from std_msgs.msg import String

    metrics = adapter.metrics

    def observe():
        raw = inspect_publishers(node, raw_topic, metrics)
        bridge = inspect_publishers(node, status_topic, metrics)
        adapter.observe(raw, bridge)

    def on_status(msg, info):
        if metrics:
            started = time.monotonic()
            metrics.received('status', info)
        try:
            try:
                data = json.loads(msg.data)
            except ValueError:
                data = None
            if isinstance(data, dict):
                publishers = inspect_publishers(node, status_topic, metrics)
                adapter.status(data, info, publishers)
            else:
                adapter.status_result('rejected', 'invalid_json')
                adapter.fault('invalid bridge status JSON')
        finally:
            if metrics:
                metrics.sample('status_callback_ms', (time.monotonic() - started) * 1000)

    def drain_status():
        if not adapter.fault_reason and adapter.status_overdue():
            if metrics:
                metrics.event('pending_status_checks')
            executor.take_pending(status_sub)

    def on_source(msg, info):
        if metrics:
            started = time.monotonic()
            metrics.received('raw_source', info)
        try:
            observe()
            drain_status()
            adapter.sample(msg, info)
        finally:
            if metrics:
                metrics.sample('raw_source_callback_ms', (time.monotonic() - started) * 1000)

    def tick():
        if metrics:
            metrics.timer(.04, time.monotonic())
        drain_status()
        observe()
        adapter.tick()
        if reporter:
            reporter.poll()

    status_sub = executor.subscribe(node, String, status_topic, on_status, status_qos(),
                                    metrics=metrics, channel='status')
    executor.subscribe(node, Twist, raw_topic, on_source, command_qos())
    node.create_timer(.04, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))


def run_ros(session, source, diagnostics=False, *, stationary_check=False):
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String

    node = executor = adapter = None
    rclpy.init(args=[])
    result = 0
    metrics = None
    try:
        executor = metadata_executor()
        node = rclpy.create_node('amr_cmd_vel_adapter')
        stamped_pub = node.create_publisher(TwistStamped, GROUND_TOPIC, command_qos())
        control_pub = node.create_publisher(String, CONTROL_TOPIC, command_qos())

        def send_velocity(twist, stamp):
            msg = TwistStamped()
            msg.twist = twist
            msg.header.frame_id = stamp.frame
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp.receipt_ns, NS)
            stamped_pub.publish(msg)

        def send_control(data):
            if rclpy.ok():
                control_pub.publish(String(data=json.dumps(data, separators=(',', ':'))))
            if data['kind'] == 'fault':
                print(data['reason'], file=sys.stderr, flush=True)

        metrics = LinkMetrics('adapter') if diagnostics else None
        adapter = CommandAdapter(session, source, send_velocity, send_control, metrics=metrics,
                                 stationary_check=stationary_check)
        attach_adapter(node, executor, adapter, SOURCE_TOPICS[source],
                       reporter=PeriodicLinkReport(metrics) if metrics else None)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (AuthorityError, RuntimeError, OSError) as exc:
        print(f'Adapter stopped: {exc}', file=sys.stderr, flush=True)
        result = 1
    finally:
        if adapter is not None:
            result = int(bool(result or adapter.fault_reason))
            adapter.fault('operator cancelled adapter')
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        if metrics:
            print(json.dumps(summarize(metrics.snapshot())), file=sys.stderr, flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, help='session ID printed by the Pi bridge')
    parser.add_argument('--source', required=True, choices=SOURCE_TOPICS)
    parser.add_argument('--link-diagnostics', action='store_true', help='emit bounded link summaries')
    parser.add_argument('--stationary-check', action='store_true',
                        help='exercise ownership and clock proof with velocity forwarding disabled')
    args = parser.parse_args()
    if not token_valid(args.session):
        parser.error('--session must be the 32-character bridge session ID')
    if not sys.stdin.isatty():
        return 1
    try:
        received = input(f'Confirm single local {args.source} publisher, isolated trusted LAN and '
                         f'synchronized Pi/laptop clocks. Type ACTIVATE {args.session}: ')
    except (EOFError, KeyboardInterrupt):
        return 1
    if received.strip() != f'ACTIVATE {args.session}':
        return 1
    try:
        return run_ros(args.session, args.source, args.link_diagnostics,
                       stationary_check=args.stationary_check)
    except ImportError as exc:
        print(f'Adapter dependency unavailable: {exc}', file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
