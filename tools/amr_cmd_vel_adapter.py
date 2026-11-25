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
    CLOCK_OFFSET_NS, CONTROL_TOPIC, DEADMAN_NS, DISCOVERY_S, GROUND_TOPIC, NS, SOURCE_TOPICS,
    STATUS_TOPIC, AuthorityError, CommandStamp, DDSStamp, WallClockGuard, check_dds_publisher, command_qos,
    graph_publishers, metadata_executor, token_valid,
)
from amr_ros2_bridge_v3 import DEADMAN_S, wheel_targets
from amr_ros2_bridge_v3_1 import twist_components


class CommandAdapter:
    def __init__(self, session, source, send_velocity, send_control, *,
                 now=time.monotonic, wall_ns=time.time_ns, activation=None):
        if not token_valid(session) or source not in SOURCE_TOPICS:
            raise ValueError('valid bridge session and exactly one source required')
        self.session, self.source = session, source
        self.activation = activation or secrets.token_hex(16)
        self.now = now
        self.clock = WallClockGuard(now, wall_ns)
        self.send_velocity, self.send_control = send_velocity, send_control
        self.bridge_gid = None
        self.raw_candidate = self.raw_owner = None
        self.discovered_at = None
        self.raw_cutoff = self.clock.check()
        self.last_status_at = self.last_probe_at = None
        self.last_source = None
        self.bridge_allows_commands = False
        self.fault_reason = None

    def control_payload(self, kind, **fields):
        return dict(kind=kind, session=self.session, source=self.source,
                    activation=self.activation, **fields)

    def fault(self, reason):
        if self.fault_reason is None:
            self.fault_reason = reason
            self.bridge_allows_commands = False
            self.send_control(self.control_payload('fault', reason=reason))

    def observe(self, raw, bridge):
        if self.fault_reason:
            return
        if len(raw) > 1 or len(bridge) > 1:
            self.fault('conflicting source or bridge publishers')
            return
        status_gid = bridge[0] if bridge else None
        candidate = raw[0] if raw else None
        if self.bridge_gid is not None and status_gid != self.bridge_gid:
            self.fault('bridge status owner lost or changed')
        elif self.raw_owner is not None and candidate != self.raw_owner:
            self.fault('selected source owner lost or changed')
        elif candidate != self.raw_candidate:
            self.raw_candidate = candidate
            self.discovered_at = self.now() if candidate is not None else None
        elif (candidate is not None and self.raw_owner is None
              and self.now() - self.discovered_at >= DISCOVERY_S):
            self.raw_owner = candidate
            self.raw_cutoff = self.clock.check()

    def status(self, message, info, publishers):
        if self.fault_reason or len(publishers) != 1 or message.get('session_id') != self.session:
            return
        if (message.get('command_source') != self.source or message.get('version') != '3.2'
                or message.get('bridge_mode') not in ('raised', 'ground')):
            self.fault('bridge mode or selected source mismatch')
            return
        if message.get('state') == 'FAULT_LATCHED':
            self.fault('bridge latched a fault')
            return
        t1 = message.get('clock_probe_ns')
        if type(t1) is not int or t1 <= 0:
            return
        try:
            wall = self.clock.check()
            check_dds_publisher(info, publishers[0])
            # The exchange must be possible before clock offset is known.
            # Use only the local receipt time and Pi-side publication ordering here.
            if not isinstance(info, dict):
                raise AuthorityError('DDS status timestamps unavailable')
            t2, sent = info.get('received_timestamp'), info.get('source_timestamp')
            if (type(t2) is not int or type(sent) is not int
                    or not 0 <= wall - t2 < DEADMAN_NS
                    or not 0 <= sent - t1 < CLOCK_OFFSET_NS):
                raise AuthorityError('invalid DDS bridge clock probe metadata')
        except AuthorityError as exc:
            self.fault(str(exc))
            return
        if self.bridge_gid is None:
            self.bridge_gid = publishers[0]
        self.last_status_at = self.now()
        expected = message.get('adapter_activation')
        if expected is not None and expected != self.activation:
            self.fault('bridge is owned by a different adapter activation')
            return
        if (expected == self.activation and message.get('authority_state') == 'OWNED'
                and message.get('clock_status') == 'VERIFIED'):
            try:
                DDSStamp.read(info, wall, remote=True)
            except AuthorityError as exc:
                self.fault(str(exc))
                return
            self.bridge_allows_commands = message.get('state') in ('READY', 'STARTING', 'RUNNING')
        else:
            self.bridge_allows_commands = False
        if self.last_probe_at is None or self.now() - self.last_probe_at >= 0.12:
            self.last_probe_at = self.now()
            self.send_control(self.control_payload('clock', t1=t1, t2=t2, t3=self.clock.check()))

    def sample(self, twist, info):
        if self.fault_reason or self.raw_owner is None:
            return
        try:
            wall = self.clock.check()
            check_dds_publisher(info, self.raw_owner)
            stamp = DDSStamp.read(info, wall)
            if self.last_source is not None and wall - self.last_source >= DEADMAN_NS:
                raise AuthorityError('selected source arrival gap')
            if stamp.source <= self.raw_cutoff or (
                    self.last_source is not None and stamp.source <= self.last_source):
                return
            wheel_targets(twist_components(twist))
        except (ValueError, AuthorityError) as exc:
            self.fault(str(exc))
            return
        self.last_source = stamp.source
        if self.bridge_allows_commands:
            header = CommandStamp(self.session, self.activation, self.source, stamp.source, stamp.received)
            self.send_velocity(twist, header)

    def tick(self):
        if self.fault_reason:
            return
        try:
            wall = self.clock.check()
            if self.last_status_at is not None and self.now() - self.last_status_at >= DEADMAN_S:
                raise AuthorityError('bridge status stream lost')
            if self.last_source is not None and wall - self.last_source >= DEADMAN_NS:
                raise AuthorityError('selected source arrival gap')
        except AuthorityError as exc:
            self.fault(str(exc))


def run_ros(session, source):
    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String

    node = executor = adapter = None
    rclpy.init(args=[])
    result = 0
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

        adapter = CommandAdapter(session, source, send_velocity, send_control)
        raw_topic = SOURCE_TOPICS[source]

        def observe():
            raw = graph_publishers(node, raw_topic)
            bridge = graph_publishers(node, STATUS_TOPIC)
            adapter.observe(raw, bridge)
            return bridge

        def on_status(msg, info):
            bridge = observe()
            try:
                data = json.loads(msg.data)
            except ValueError:
                adapter.fault('invalid bridge status JSON')
                return
            if isinstance(data, dict):
                adapter.status(data, info, bridge)

        def on_source(msg, info):
            observe()
            adapter.sample(msg, info)

        def tick():
            observe()
            adapter.tick()

        executor.subscribe(node, String, STATUS_TOPIC, on_status, command_qos())
        executor.subscribe(node, Twist, raw_topic, on_source, command_qos())
        node.create_timer(0.04, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
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
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, help='session ID printed by the Pi bridge')
    parser.add_argument('--source', required=True, choices=SOURCE_TOPICS)
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
        return run_ros(args.session, args.source)
    except ImportError as exc:
        print(f'Adapter dependency unavailable: {exc}', file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
