#!/usr/bin/env python3
"""ROS 2 Humble bench bridge and offline Nav2 Twist monitor for Phase 4L.6A."""
import argparse
from contextlib import contextmanager
from dataclasses import asdict, dataclass, replace
from enum import Enum
import json
import math
import sys
import time

from amr_ros2_bridge_v3 import (
    BENCH_TOPIC, CPR, DEADMAN_S, LOOP_PERIOD_S, MAX_WHEEL_RPM, MIN_ACTIVE_RPM,
    SESSION_MAX_S, SETTLE_S, SETTLE_TIMEOUT_S, STATIONARY_RPM, UART_BAUD,
    VERSION_REPLY, WHEEL_RADIUS_M, ZERO, Feedback, GuardedBridge, ProtocolError,
    State, Uart, directions, wheel_targets,
)

LOW_TOPIC = '/amr_bridge/low_speed_cmd_vel'
LOW_CANDIDATES = (2, 3, 4, 5)
LOW_CAPABILITIES = 'CAPS PV=3.1 CAP=ARM_LOW,LOW_FIXED_PAIR,LOW_SESSION_15S'
CAPABILITIES_DISABLED = 'CAPS PV=3.1 CAP=NONE'
SYNC_MARGIN_RPM = 1.0
MPS_PER_RPM = 2 * math.pi * WHEEL_RADIUS_M / 60


class Mode(str, Enum):
    READ_ONLY = 'read-only'
    CONTINUOUS = 'continuous-bench'
    LOW = 'low-speed-bench'


class Transition(str, Enum):
    STARTING = 'STARTING'


def twist_components(twist):
    return (twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z)


@dataclass(frozen=True)
class Preview:
    disposition: str
    requested_rpm: tuple | None = None
    limited_rpm: tuple | None = None
    effective_abs_rpm_upper_bound: tuple | None = None
    reason: str | None = None


class Nav2Monitor:
    """Pure evaluation of an unstamped Twist with no hardware or ROS subscription."""

    @staticmethod
    def evaluate(twist, previous_rpm=ZERO):
        return Nav2Monitor.evaluate_components(twist_components(twist), previous_rpm)

    @staticmethod
    def evaluate_components(components, previous_rpm=ZERO):
        try:
            requested, limited = wheel_targets(components)
        except ValueError as exc:
            return Preview('INHIBITED', reason=str(exc))
        if not any(components):
            return Preview('STOP', ZERO, ZERO, ZERO)
        if not any(limited):
            return Preview('INHIBITED', requested, limited, reason='nonzero command underflowed')
        # Preserve curvature while reserving firmware trim if the final pair
        # could satisfy the forward sync condition after rounding.
        sync_possible = (min(limited) > 0 and max(limited) >= 6.99999
                         and abs(limited[0] - limited[1]) < 0.05001)
        margin = SYNC_MARGIN_RPM if sync_possible else 0.0
        peak = max(map(abs, limited))
        reserve_scale = min(1.0, (MAX_WHEEL_RPM - margin) / peak)
        limited = tuple(rpm * reserve_scale for rpm in limited)
        upper_bound = tuple(abs(rpm) + margin for rpm in limited)
        if any(0 < abs(rpm) < MIN_ACTIVE_RPM for rpm in limited):
            return Preview('INHIBITED', requested, limited, upper_bound,
                           'wheel target below the normal 5 RPM minimum')
        if previous_rpm != ZERO and directions(previous_rpm) != directions(limited):
            return Preview('STOP_BEFORE_CHANGE', requested, limited, upper_bound,
                           'requires stationary feedback and a new command')
        return Preview('ELIGIBLE_FOR_REVIEW', requested, limited, upper_bound)


@contextmanager
def exclusive_uart(port_name):
    import fcntl
    import termios
    import serial

    port = serial.Serial(port_name, UART_BAUD, timeout=0.004, write_timeout=0.025,
                         exclusive=True)
    try:
        fcntl.ioctl(port.fileno(), termios.TIOCEXCL)
        try:
            yield Uart(port)
        finally:
            # Release the kernel TTY claim before the next explicitly staged session.
            # pyserial still holds its advisory lock until close.
            fcntl.ioctl(port.fileno(), termios.TIOCNXCL)
    finally:
        port.close()


class BridgeV31(GuardedBridge):
    extended_version = '3.1'

    def __init__(self, uart, mode=Mode.READ_ONLY, low_rpm=None, **kwargs):
        self.mode = Mode(mode)
        if ((self.mode == Mode.LOW and (type(low_rpm) is not int or abs(low_rpm) not in LOW_CANDIDATES))
                or (self.mode != Mode.LOW and low_rpm is not None)):
            raise ValueError('a signed candidate of 2, 3, 4 or 5 RPM is required only in low mode')
        self.low_rpm = low_rpm
        super().__init__(uart, bench=self.mode != Mode.READ_ONLY, **kwargs)
        self.extended_status = False
        self.low_capable = False
        self.low_run_consumed = False
        self.feedback_version = None
        self.sync_correction = None
        self.acknowledged = None
        self.acknowledged_at = None
        self.stop_verified = False

    def query(self):
        return self._parse_feedback(self.uart.exchange('Q', 'STATUS ', prefix=True))

    def _parse_feedback(self, reply):
        tokens = reply.split()
        versions = [token[3:] for token in tokens if token.startswith('PV=')]
        corrections = [token[3:] for token in tokens if token.startswith('SC=')]
        if versions not in ([], [self.extended_version]) or len(versions) != len(corrections):
            raise ProtocolError('invalid firmware version or sync fields in Q')
        try:
            correction = float(corrections[0]) if corrections else None
        except ValueError as exc:
            raise ProtocolError('invalid straight-sync correction') from exc
        if correction is not None and not -SYNC_MARGIN_RPM <= correction <= SYNC_MARGIN_RPM:
            raise ProtocolError('straight-sync correction exceeds advertised bound')
        is_low = 'MODE=LOW' in tokens
        if is_low:
            if versions != [self.extended_version] or correction != 0:
                raise ProtocolError('invalid low-mode feedback')
            # V3 validates all shared fields and the armed/mode invariants.
            tokens[tokens.index('MODE=LOW')] = 'MODE=CONT'
        feedback = Feedback.parse(' '.join(tokens))
        if versions and (feedback.boot is None or None in feedback.duty):
            raise ProtocolError('extended Q is missing boot identity or PWM feedback')
        self.feedback = replace(feedback, mode='LOW') if is_low else feedback
        self.feedback_at = self.now()
        self.feedback_version = versions[0] if versions else None
        self.sync_correction = correction
        return self.feedback

    def start(self):
        try:
            feedback = self.query()
            if feedback.boot is not None:
                self.uart.exchange('VER', VERSION_REPLY)
                self.protocol = 3
                if self.feedback_version == '3.1':
                    capabilities = self.uart.exchange('CAPS', 'CAPS ', prefix=True)
                    if capabilities not in (LOW_CAPABILITIES, CAPABILITIES_DISABLED):
                        raise ProtocolError('unrecognized firmware capability reply')
                    self.protocol = '3.1'
                    self.extended_status = True
                    self.low_capable = capabilities == LOW_CAPABILITIES
            if self.mode == Mode.LOW and not self.low_capable:
                raise ProtocolError('experimental low-speed firmware capability missing')
            if self.bench and self.protocol is None:
                raise ProtocolError('continuous-mode firmware capability missing')
            self.baseline = feedback.boot, feedback.counters
            self.check_feedback(armed=False, running=False)
            if self.bench:
                self.begin_stop()
        except (RuntimeError, OSError) as exc:
            self.latch(str(exc))

    def check_feedback(self, *, armed=None, running=None):
        feedback = self.feedback
        if feedback.fault:
            raise ProtocolError(f'firmware fault {feedback.reason}')
        if (feedback.boot, feedback.counters) != self.baseline:
            raise ProtocolError('firmware reset or fault counters changed')
        if (self.feedback_version == self.extended_version) != self.extended_status:
            raise ProtocolError('firmware status version changed')
        if armed is not None and feedback.armed != armed:
            raise ProtocolError('unexpected firmware arm state')
        if running is not None and feedback.running != running:
            raise ProtocolError('unexpected firmware run state')
        expected_mode = 'LOW' if self.mode == Mode.LOW else 'CONT'
        if self.bench and feedback.armed and feedback.mode != expected_mode:
            raise ProtocolError('firmware armed in unexpected mode')

    def verify_stop(self):
        self.stop_verified = False
        super().verify_stop()
        if any(duty not in (0.0, None) for duty in self.feedback.duty):
            raise ProtocolError('firmware reports nonzero PWM after STOP')
        self.stop_verified = True

    def command(self, components, source_ns):
        active = self.state in (Transition.STARTING, State.RUNNING)
        if self.state != State.READY and not active:
            return
        try:
            self.check_session()
            if active:
                self.require_fresh()
            age = (self.wall_ns() - source_ns) / 1e9
            if source_ns <= max(self.source_cutoff, self.last_source) or not 0 <= age < DEADMAN_S:
                return
            self.last_source = source_ns
            self.requested = self.limited = ZERO
            self.requested, self.limited = wheel_targets(components)
            is_stop = not any(components)
            if not is_stop:
                if self.mode == Mode.LOW:
                    # Tolerance covers only the conversion from m/s to RPM.
                    if components[-1] != 0 or not all(
                            math.isclose(rpm, self.low_rpm, abs_tol=1e-9, rel_tol=0)
                            for rpm in self.limited):
                        raise ValueError('unsupported low-speed request for selected RPM and direction')
                    self.limited = (self.low_rpm, self.low_rpm)
                    if self.low_run_consumed and not active:
                        return
                elif self.limited == ZERO or any(0 < abs(rpm) < MIN_ACTIVE_RPM for rpm in self.limited):
                    raise ValueError('unsupported wheel RPM below 5 RPM after limiting')
            if is_stop and not active and self.command_at is None:
                return
            if is_stop or (active and directions(self.sent) != directions(self.limited)):
                self.begin_stop()
                return
            self.command_at = self.now() - age
        except (ValueError, RuntimeError, OSError) as exc:
            self.latch(str(exc))

    def send_velocity(self, starting):
        if self.mode == Mode.LOW:
            self.check_session()
            self.require_fresh()
            self.uart.exchange(f'V {self.low_rpm} {self.low_rpm}', 'OK V')
            self.sent = (self.low_rpm, self.low_rpm)
            self.low_run_consumed = True
        else:
            super().send_velocity(starting)
        self.acknowledged = self.sent
        self.acknowledged_at = self.now()
        self.state = Transition.STARTING if starting else State.RUNNING

    def tick(self):
        try:
            if self.state != State.FAULT_LATCHED:
                self.check_session()
            active = self.state in (Transition.STARTING, State.RUNNING)
            if active:
                self.require_fresh()
            self.query()
            if self.state == State.FAULT_LATCHED:
                return
            self.check_feedback(armed=active, running=active)
            if self.state == State.STOPPING:
                self.settle()
            elif active:
                self.send_velocity(starting=False)
            elif self.state == State.READY and self.command_at is not None:
                self.require_fresh()
                if not self.feedback.stationary:
                    raise ProtocolError('wheels moved before arming')
                self.check_session()
                if self.session_deadline is None:
                    self.session_deadline = self.now() + SESSION_MAX_S
                self.stop_verified = False
                if self.mode == Mode.LOW:
                    self.uart.exchange(f'ARM_LOW {self.low_rpm} WHEELS_UP', 'OK ARMED LOW')
                else:
                    self.uart.exchange('ARM_CONT', 'OK ARMED CONT')
                self.query()
                self.check_feedback(armed=True, running=False)
                self.send_velocity(starting=True)
        except (RuntimeError, OSError) as exc:
            self.latch(str(exc))

    def status(self):
        status = super().status()
        age = status['feedback_age_s']
        status.update({
            'version': '3.1',
            'bridge_mode': self.mode.value,
            'acknowledged_rpm': self.acknowledged,
            'acknowledgement_age_s': (self.now() - self.acknowledged_at
                                       if self.acknowledged_at is not None else None),
            'firmware_run_confirmed': self.state == State.RUNNING and bool(
                self.feedback and self.feedback.running and age < DEADMAN_S),
            'firmware_stationary': self.feedback.stationary if self.feedback else None,
            'stationary_ready': self.state == State.READY and self.stop_verified and bool(
                self.feedback and self.feedback.stationary and age < DEADMAN_S),
            'stop_verified': self.stop_verified,
            'encoder_ticks': self.feedback.ticks if self.feedback else None,
            'boot_id': self.feedback.boot if self.feedback else None,
            'straight_sync_correction_rpm': self.sync_correction,
            'low_speed_capable': self.low_capable,
            'low_rpm': self.low_rpm,
            'low_run_consumed': self.low_run_consumed,
        })
        return status

    def verify_stationary_stop(self, sleep=time.sleep):
        """Repeat STOP verification and observe quiet feedback even after a fault."""
        if self.state != State.FAULT_LATCHED:
            self.state = State.STOPPING
            self.command_at = None
        self.verify_stop()
        boot = self.feedback.boot
        if boot is None or (self.baseline is not None and boot != self.baseline[0]):
            raise ProtocolError('firmware boot identity unavailable or changed before final stop')
        deadline = self.now() + SETTLE_TIMEOUT_S
        quiet_at = ticks = None
        while self.now() < deadline:
            feedback = self.query()
            now = self.now()
            if feedback.boot != boot or feedback.armed or feedback.running or feedback.duty != ZERO:
                raise ProtocolError('reset, activity or invalid PWM during final stationary verification')
            quiet = feedback.stationary and all(abs(rpm) < STATIONARY_RPM for rpm in feedback.measured)
            if not quiet:
                quiet_at = ticks = None
            elif ticks != feedback.ticks:
                quiet_at, ticks = now, feedback.ticks
            elif now - quiet_at >= SETTLE_S:
                if self.state != State.FAULT_LATCHED:
                    self.source_cutoff = self.wall_ns()
                    self.state = State.READY
                return feedback
            sleep(LOOP_PERIOD_S)
        raise ProtocolError('final wheel stationary verification timed out')


def confirm_bench(mode, signed_rpm=None):
    if not sys.stdin.isatty():
        print('Bench mode requires an interactive operator confirmation.', file=sys.stderr)
        return False
    token = 'WHEELS_UP_LOW_SPEED' if mode == Mode.LOW else 'WHEELS_UP'
    test = f' ({signed_rpm:+d} RPM per wheel)' if mode == Mode.LOW else ''
    try:
        answer = input(f'Secure both raised wheels and verify the physical motor cutoff{test}. '
                       f'Type {token} to proceed: ')
    except (EOFError, KeyboardInterrupt):
        return False
    return answer.strip() == token


def run_ros(port_name, mode, low_rpm=None):
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.clock import Clock, ClockType
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    result = 0
    node = None
    rclpy.init()
    try:
        node = rclpy.create_node('amr_serial_bridge')
        joint_pub = node.create_publisher(JointState, '/joint_states', 10)
        status_pub = node.create_publisher(String, '/amr_bridge/status', 10)
        with exclusive_uart(port_name) as uart:
            bridge = BridgeV31(uart, mode, low_rpm)
            try:
                bridge.start()

                def loop():
                    previous = bridge.feedback
                    bridge.tick()
                    feedback = bridge.feedback
                    if feedback is not None and feedback is not previous:
                        msg = JointState()
                        msg.header.stamp = node.get_clock().now().to_msg()
                        msg.name = ['left_wheel_joint', 'right_wheel_joint']
                        msg.position = [ticks * 2 * math.pi / CPR for ticks in feedback.ticks]
                        msg.velocity = [rpm * 2 * math.pi / 60 for rpm in feedback.measured]
                        joint_pub.publish(msg)
                    status_pub.publish(String(data=json.dumps(bridge.status(), allow_nan=False)))

                def on_command(msg):
                    stamp = msg.header.stamp
                    if stamp.nanosec < 10**9:
                        bridge.command(twist_components(msg.twist),
                                       stamp.sec * 10**9 + stamp.nanosec)

                if mode != Mode.READ_ONLY:
                    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE,
                                     lifespan=Duration(seconds=DEADMAN_S))
                    topic = LOW_TOPIC if mode == Mode.LOW else BENCH_TOPIC
                    node.create_subscription(TwistStamped, topic, on_command, qos)
                node.create_timer(LOOP_PERIOD_S, loop, clock=Clock(clock_type=ClockType.STEADY_TIME))
                rclpy.spin(node)
            finally:
                if not bridge.shutdown():
                    result = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (RuntimeError, OSError, ImportError) as exc:
        print(f'Bridge stopped: {exc}', file=sys.stderr)
        result = 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/serial0')
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument('--continuous-bench', action='store_true')
    actions.add_argument('--low-speed-bench', action='store_true')
    actions.add_argument('--monitor-twist', nargs=2, type=float, metavar=('VX', 'WZ'),
                         help='evaluate one candidate Twist offline and exit')
    parser.add_argument('--rpm', type=int, choices=LOW_CANDIDATES)
    parser.add_argument('--direction', choices=('forward', 'reverse'))
    args = parser.parse_args()
    if args.low_speed_bench and (args.rpm is None or args.direction is None):
        parser.error('--low-speed-bench requires --rpm and --direction')
    if not args.low_speed_bench and (args.rpm is not None or args.direction is not None):
        parser.error('--rpm and --direction require --low-speed-bench')
    if args.monitor_twist is not None:
        vx, wz = args.monitor_twist
        preview = Nav2Monitor.evaluate_components((vx, 0., 0., 0., 0., wz))
        print(json.dumps(asdict(preview), allow_nan=False))
        return int(preview.disposition == 'INHIBITED')
    mode = (Mode.LOW if args.low_speed_bench else
            Mode.CONTINUOUS if args.continuous_bench else Mode.READ_ONLY)
    rpm = args.rpm * (-1 if args.direction == 'reverse' else 1) if mode == Mode.LOW else None
    if mode != Mode.READ_ONLY and not confirm_bench(mode, rpm):
        return 1
    try:
        return run_ros(args.port, mode, rpm)
    except ImportError as exc:
        print(f'Bridge dependency unavailable: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
