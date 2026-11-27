"""Shared commissioning topics, clock checks and Humble DDS metadata access."""
from dataclasses import dataclass
import re
import time

from amr_ros2_bridge_v3 import DEADMAN_S

GROUND_TOPIC = '/amr_bridge/ground_cmd_vel'
CONTROL_TOPIC = '/amr_bridge/ground_control'
STATUS_TOPIC = '/amr_bridge/status'
SOURCE_TOPICS = {
    'teleop': '/amr/teleop_cmd_vel',
    'nav2': '/amr/nav2_cmd_vel',
    'commissioning': '/amr/commissioning_cmd_vel',
}

NS = 1_000_000_000
DEADMAN_NS = round(DEADMAN_S * NS)
CLOCK_OFFSET_NS = 20_000_000
CLOCK_PROOF_S = 1.0
DISCOVERY_S = 0.25
TOKEN = re.compile(r'[0-9a-f]{32}')


def token_valid(value):
    return isinstance(value, str) and TOKEN.fullmatch(value) is not None


class AuthorityError(RuntimeError):
    pass


class DDSAgeError(AuthorityError):
    """DDS timestamps passed validity checks but exceeded the freshness limit."""


class WallClockGuard:
    def __init__(self, now=time.monotonic, wall_ns=time.time_ns):
        self.now, self.wall_ns = now, wall_ns
        self.initial_offset = wall_ns() - round(now() * NS)

    def check(self):
        wall = self.wall_ns()
        offset = wall - round(self.now() * NS)
        if abs(offset - self.initial_offset) > CLOCK_OFFSET_NS:
            raise AuthorityError('system clock changed during commissioning')
        return wall


@dataclass(frozen=True)
class DDSStamp:
    source: int
    received: int

    @classmethod
    def read(cls, info, wall, *, remote=False):
        if not isinstance(info, dict):
            raise AuthorityError('DDS timestamps unavailable')
        source, received = info.get('source_timestamp'), info.get('received_timestamp')
        if (type(source) is not int or type(received) is not int
                or source <= 0 or received <= 0):
            raise AuthorityError('DDS source or receipt timestamp unavailable')
        skew = CLOCK_OFFSET_NS if remote else 0
        receipt_age = wall - received
        if receipt_age < 0:
            raise AuthorityError('stale or invalid DDS receipt timestamp')
        if received - source < -skew:
            raise AuthorityError('stale DDS source timestamp or clock mismatch')
        if receipt_age >= DEADMAN_NS:
            raise DDSAgeError('stale or invalid DDS receipt timestamp')
        if wall - source + skew >= DEADMAN_NS:
            raise DDSAgeError('stale DDS source timestamp or clock mismatch')
        return cls(source, received)


def clock_proven(t1, t2, t3, t4):
    """All offsets consistent with nonnegative network delays must be safe."""
    if not all(type(ns) is int and ns > 0 for ns in (t1, t2, t3, t4)):
        return False
    if t3 < t2 or t4 < t1:
        return False
    lower, upper = t3 - t4, t2 - t1
    return -CLOCK_OFFSET_NS <= lower <= upper <= CLOCK_OFFSET_NS


@dataclass(frozen=True)
class CommandStamp:
    session: str
    activation: str
    source: str
    source_ns: int
    receipt_ns: int
    health_until_ns: int | None = None

    @property
    def frame(self):
        if self.health_until_ns is not None:
            return f'amr-pi/{self.session}/{self.activation}/nav2/{self.source_ns}/{self.health_until_ns}'
        return f'amr-v3.2/{self.session}/{self.activation}/{self.source}/{self.source_ns}'

    @classmethod
    def from_header(cls, header):
        parts = header.frame_id.split('/')
        health_until = None
        if len(parts) == 6 and parts[0] == 'amr-pi' and parts[3] == 'nav2' and re.fullmatch(r'[0-9]{1,19}', parts[5]):
            health_until = int(parts.pop())
            parts[0] = 'amr-v3.2'
        if (len(parts) != 5 or parts[0] != 'amr-v3.2'
                or not token_valid(parts[1]) or not token_valid(parts[2])
                or parts[3] not in SOURCE_TOPICS or not re.fullmatch(r'[1-9][0-9]{0,18}', parts[4])):
            raise AuthorityError('invalid ground command header')
        stamp = header.stamp
        if stamp.sec <= 0 or not 0 <= stamp.nanosec < NS:
            raise AuthorityError('invalid adapter receipt stamp')
        return cls(parts[1], parts[2], parts[3], int(parts[4]), stamp.sec * NS + stamp.nanosec, health_until)


def check_dds_publisher(info, expected):
    # Humble 3.3.22 may only return timestamps. Pin observed graph endpoints
    # regardless, and check per-sample identity on bindings that expose it.
    if isinstance(info, dict) and 'publisher_gid' in info:
        try:
            matches = tuple(info['publisher_gid']) == expected
        except TypeError:
            matches = False
        if not matches:
            raise AuthorityError('DDS sample publisher differs from pinned endpoint')


def graph_publishers(node, topic):
    return tuple(tuple(info.endpoint_gid) for info in node.get_publishers_info_by_topic(topic))


def command_qos():
    from rclpy.duration import Duration
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE, lifespan=Duration(seconds=DEADMAN_S))


def status_qos(*, publisher=False):
    # Keep the live QoS policies. A reliable writer is compatible with the
    # best-effort reader, but that match does not guarantee delivery.
    return 10 if publisher else command_qos()


def telemetry_qos():
    from rclpy.qos import ReliabilityPolicy

    qos = command_qos()
    qos.reliability = ReliabilityPolicy.RELIABLE
    return qos


def arrival_monotonic(received_ns, wall_ns, now):
    return now - (wall_ns - received_ns) / NS


def inspect_publishers(node, topic, metrics=None):
    started = time.monotonic() if metrics else None
    endpoints = graph_publishers(node, topic)
    if metrics:
        metrics.sample('graph_inspection_ms', (time.monotonic() - started) * 1000)
    return endpoints


def metadata_executor():
    """Humble 3.3.x takes (message, info) but normally discards info.

    Keep the override scoped to explicitly registered subscriptions. Missing
    metadata or an incompatible executor fails closed in the authority path.
    """
    from rclpy.executors import SingleThreadedExecutor

    if not all(hasattr(SingleThreadedExecutor, name)
               for name in ('_take_subscription', '_execute_subscription')):
        raise AuthorityError('installed executor cannot expose DDS metadata')

    class MetadataExecutor(SingleThreadedExecutor):
        def __init__(self):
            super().__init__()
            self.with_metadata = {}

        def subscribe(self, node, msg_type, topic, callback, qos, *, metrics=None, channel=None):
            sub = node.create_subscription(msg_type, topic, callback, qos)
            self.with_metadata[sub] = (metrics, channel)
            return sub

        def _take_subscription(self, sub):
            if sub not in self.with_metadata:
                return super()._take_subscription(sub)
            with sub.handle:
                result = sub.handle.take_message(sub.msg_type, sub.raw)
            metrics, channel = self.with_metadata[sub]
            if result is not None and metrics:
                metrics.event(channel + '_taken')
            return result

        def take_pending(self, sub):
            """Take at most one pending sample before a timer declares timeout."""
            result = self._take_subscription(sub)
            if result is not None:
                self._dispatch(sub, result)

        def _dispatch(self, sub, result):
            sub.callback(result[0], result[1] if len(result) == 2 else None)

        async def _execute_subscription(self, sub, result):
            if sub not in self.with_metadata:
                await super()._execute_subscription(sub, result)
            elif result is not None:
                self._dispatch(sub, result)

    return MetadataExecutor()
