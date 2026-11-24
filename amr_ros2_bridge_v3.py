#!/usr/bin/env python3
"""Phase 4L.1 UART bridge for ROS 2 Humble and supervised raised-wheel testing."""
import argparse
from dataclasses import dataclass
from enum import Enum
import json
import math
import re
import secrets
import sys
import time

CPR = 3960.0
WHEEL_RADIUS_M = 0.0435
WHEEL_SEPARATION_M = 0.300
UART_BAUD = 57600
UART_TIMEOUT_S = 0.075
LOOP_PERIOD_S = 0.06
DEADMAN_S = 0.23
SESSION_MAX_S = 15.0
SESSION_STOP_MARGIN_S = 0.40  # UART deadline, 300 ms firmware watchdog and a control tick.
MAX_LINEAR_MPS = 0.06
MAX_ANGULAR_RPS = 0.35
MAX_WHEEL_RPM = 15.0
MIN_ACTIVE_RPM = 5.0
RAMP_RPM_PER_S = 15.0
SETTLE_S = 0.25
SETTLE_TIMEOUT_S = 2.0
STATIONARY_RPM = 1.0
BENCH_TOPIC = '/amr_bridge/bench_cmd_vel'
VERSION_REPLY = 'VERSION PV=3 CAP=ARM_CONT,STICKY_FAULT,STATIONARY,BOOT_ID'
ZERO = (0.0, 0.0)


class State(str, Enum):
    READ_ONLY = 'READ_ONLY'
    READY = 'READY'
    RUNNING = 'RUNNING'
    STOPPING = 'STOPPING'
    FAULT_LATCHED = 'FAULT_LATCHED'


class ProtocolError(RuntimeError):
    pass


class Uart:
    def __init__(self, serial_port, monotonic=time.monotonic):
        self.port = serial_port
        self.monotonic = monotonic
        self.buffer = bytearray()

    def exchange(self, command, expected, *, prefix=False, drain=False):
        deadline = self.monotonic() + UART_TIMEOUT_S
        frame = (command + '\r').encode('ascii')
        if self.port.write(frame) != len(frame):
            raise ProtocolError(f'incomplete UART write for {command!r}')
        while self.monotonic() < deadline:
            end = self.buffer.find(b'\n')
            if end < 0:
                if len(self.buffer) >= 256:
                    self.buffer.clear()
                    raise ProtocolError('oversized UART reply')
                self.buffer.extend(self.port.read(max(1, min(self.port.in_waiting, 256))))
                continue
            line = bytes(self.buffer[:end]).removesuffix(b'\r')
            del self.buffer[:end + 1]
            if not line or len(line) >= 256 or not all(32 <= value <= 126 for value in line):
                if drain:
                    continue
                raise ProtocolError('malformed UART reply')
            reply = line.decode('ascii')
            if (reply.startswith(expected) if prefix else reply == expected):
                return reply
            if not drain:
                raise ProtocolError(f'{command!r} returned {reply!r}, expected {expected!r}')
        # The next transaction can discard a delayed fragment at its delimiter.
        raise ProtocolError(f'UART timeout waiting for {expected!r} after {command!r}')


@dataclass(frozen=True)
class Feedback:
    armed: bool
    running: bool
    measured: tuple
    duty: tuple
    ticks: tuple
    counters: tuple
    mode: str | None
    fault: bool | None
    reason: str | None
    stationary: bool | None
    boot: str | None

    @classmethod
    def parse(cls, reply):
        def flag(value):
            if value not in ('0', '1'):
                raise ValueError('invalid bit')
            return value == '1'

        try:
            tokens = reply.split()[1:]
            fields = dict(token.split('=', 1) for token in tokens)
            if len(tokens) != len(fields):
                raise ValueError('duplicate status fields')
            armed, running = flag(fields['A']), flag(fields['RUN'])
            if running and not armed:
                raise ValueError('running while disarmed')
            wheel = [tuple(float(value) for value in fields[key].split('/')) for key in ('L', 'R')]
            if any(len(pair) != 2 or not all(map(math.isfinite, pair)) for pair in wheel):
                raise ValueError('invalid wheel feedback')
            duty = tuple(float(fields[key]) if key in fields else None for key in ('DL', 'DR'))
            if any(value is not None and not 0.0 <= value <= 45.0 for value in duty):
                raise ValueError('invalid PWM duty')
            ticks = tuple(int(fields[key]) for key in ('TL', 'TR'))
            counters = tuple(int(fields[key]) for key in ('UE', 'CF'))
            if (any(not 0 <= n <= 0xFFFFFFFF for n in counters)
                    or any(not -(2**31) <= n < 2**31 for n in ticks)):
                raise ValueError('invalid fault or encoder counter')
            extras = {'MODE', 'FL', 'FR', 'ST', 'BOOT'}
            if not extras.intersection(fields):
                return cls(armed, running, (wheel[0][1], wheel[1][1]), duty, ticks,
                           counters, None, None, None, None, None)
            mode, fault, stationary = fields['MODE'], flag(fields['FL']), flag(fields['ST'])
            reason, boot = fields['FR'], fields['BOOT']
            if (mode not in ('IDLE', 'LEGACY', 'CONT') or armed != (mode != 'IDLE')
                    or (fault and (armed or running)) or (stationary and running)
                    or fault != (reason != 'NONE') or not re.fullmatch(r'[0-9a-f]{8}', boot)):
                raise ValueError('inconsistent protocol 3 status')
            return cls(armed, running, (wheel[0][1], wheel[1][1]), duty, ticks,
                       counters, mode, fault, reason if fault else None, stationary, boot)
        except (KeyError, ValueError) as exc:
            raise ProtocolError(f'invalid Q status: {reply!r}') from exc


def wheel_targets(components):
    vx, lateral, vertical, roll, pitch, wz = components
    if not all(map(math.isfinite, components)):
        raise ValueError('non-finite Twist')
    if any((lateral, vertical, roll, pitch)):
        raise ValueError('unsupported lateral or 3D Twist')
    scale = 60.0 / (2.0 * math.pi * WHEEL_RADIUS_M)
    turn = wz * WHEEL_SEPARATION_M / 2.0
    requested = ((vx - turn) * scale, (vx + turn) * scale)
    if not all(map(math.isfinite, requested)):
        raise ValueError('Twist overflows wheel RPM calculation')
    # Leave room at the body-speed boundaries for UART and firmware float rounding.
    ratio = max(1.0, abs(vx) / (MAX_LINEAR_MPS * (1 - 1e-7)),
                abs(wz) / (MAX_ANGULAR_RPS * (1 - 1e-7)),
                *(abs(rpm) / MAX_WHEEL_RPM for rpm in requested))
    limited = tuple(rpm / ratio for rpm in requested)
    if any(rpm and not bounded for rpm, bounded in zip(requested, limited)):
        raise ValueError('unsupported wheel RPM below 5 RPM after limiting')
    return requested, limited


def directions(target):
    return tuple((rpm > 0) - (rpm < 0) for rpm in target)


class GuardedBridge:
    def __init__(self, uart, bench=False, monotonic=time.monotonic, wall_ns=time.time_ns,
                 report=lambda message: print(message, file=sys.stderr, flush=True)):
        self.uart = uart
        self.bench = bench
        self.now = monotonic
        self.wall_ns = wall_ns
        self.report = report
        self.state = State.STOPPING if bench else State.READ_ONLY
        self.protocol = None
        self.feedback = None
        self.feedback_at = None
        self.baseline = None
        self.fault_reason = None
        self.requested = self.limited = self.sent = ZERO
        self.command_at = None
        self.source_cutoff = self.last_source = 0
        self.session_deadline = None
        self.stop_deadline = None
        self.quiet_at = None
        self.quiet_ticks = None
        self.last_send = None
        self.sequence = secrets.randbelow(1_000_000)

    def query(self):
        self.feedback = Feedback.parse(self.uart.exchange('Q', 'STATUS ', prefix=True))
        self.feedback_at = self.now()
        return self.feedback

    def check_feedback(self, *, armed=None, running=None):
        feedback = self.feedback
        if feedback.fault:
            raise ProtocolError(f'firmware fault {feedback.reason}')
        identity = feedback.boot, feedback.counters
        if identity != self.baseline:
            raise ProtocolError('firmware reset or fault counters changed')
        if armed is not None and feedback.armed != armed:
            raise ProtocolError('unexpected firmware arm state')
        if running is not None and feedback.running != running:
            raise ProtocolError('unexpected firmware run state')
        if self.bench and feedback.armed and feedback.mode != 'CONT':
            raise ProtocolError('firmware armed in unexpected mode')

    def verify_stop(self):
        ack_error = None
        try:
            self.uart.exchange('STOP', 'OK STOP', drain=True)
        except (RuntimeError, OSError):
            # A partial outbound frame can consume the first STOP delimiter.
            try:
                self.uart.exchange('STOP', 'OK STOP', drain=True)
            except (RuntimeError, OSError) as exc:
                ack_error = exc
        self.sequence = (self.sequence + 1) % 1_000_000
        self.uart.exchange(f'T {self.sequence:06d}', f'ACK {self.sequence:06d}', drain=True)
        feedback = self.query()
        if feedback.armed or feedback.running:
            raise ProtocolError('firmware still armed or running after STOP')
        if ack_error is not None:
            raise ProtocolError(f'Q reports idle but STOP acknowledgement failed: {ack_error}')
        self.sent = ZERO

    def start(self):
        try:
            feedback = self.query()
            if feedback.boot is not None:
                self.uart.exchange('VER', VERSION_REPLY)
                self.protocol = 3
            if self.bench and self.protocol != 3:
                raise ProtocolError('continuous-mode firmware capability missing')
            self.baseline = feedback.boot, feedback.counters
            self.check_feedback(armed=False, running=False)
            if self.bench:
                self.begin_stop()
        except (RuntimeError, OSError) as exc:
            self.latch(str(exc))

    def latch(self, reason):
        if self.state == State.FAULT_LATCHED:
            return
        self.state = State.FAULT_LATCHED
        self.fault_reason = reason
        self.report(f'FAULT_LATCHED: {reason}')
        try:
            self.verify_stop()
        except (RuntimeError, OSError) as exc:
            self.report(f'STOP verification failed: {exc}. CUT MOTOR POWER PHYSICALLY.')

    def begin_stop(self):
        self.state = State.STOPPING
        self.command_at = None
        self.quiet_at = self.quiet_ticks = None
        self.stop_deadline = self.now() + SETTLE_TIMEOUT_S
        self.verify_stop()
        self.check_feedback(armed=False, running=False)

    def command(self, components, source_ns):
        if self.state not in (State.READY, State.RUNNING):
            return
        try:
            self.check_session()
            if self.state == State.RUNNING:
                self.require_fresh()
            # The publisher's stamp cuts off messages sent before READY, even if the
            # executor takes them after the blocking STOP/settle transition.
            received_wall = self.wall_ns()
            age = (received_wall - source_ns) / 1e9
            if source_ns <= max(self.source_cutoff, self.last_source) or not 0 <= age < DEADMAN_S:
                return
            self.last_source = source_ns
            self.requested = self.limited = ZERO
            self.requested, self.limited = wheel_targets(components)
            if any(0 < abs(rpm) < MIN_ACTIVE_RPM for rpm in self.limited):
                raise ValueError('unsupported wheel RPM below 5 RPM after limiting')
            if self.limited == ZERO and self.state == State.READY and self.command_at is None:
                return
            if (self.limited == ZERO or
                    (self.state == State.RUNNING and directions(self.sent) != directions(self.limited))):
                self.begin_stop()
                return
            self.command_at = self.now() - age
        except (ValueError, RuntimeError, OSError) as exc:
            self.latch(str(exc))

    def check_session(self):
        if (self.session_deadline is not None and
                self.now() + SESSION_STOP_MARGIN_S >= self.session_deadline):
            raise ProtocolError('15-second bench session limit reached')

    def require_fresh(self):
        if self.command_at is None or self.now() - self.command_at >= DEADMAN_S:
            raise ProtocolError('bench command deadman expired')

    def settle(self):
        now = self.now()
        if now >= self.stop_deadline:
            raise ProtocolError('wheel stationary verification timed out')
        feedback = self.feedback
        if not feedback.stationary or any(abs(rpm) >= STATIONARY_RPM for rpm in feedback.measured):
            self.quiet_at = self.quiet_ticks = None
            return
        if self.quiet_ticks != feedback.ticks:
            self.quiet_at, self.quiet_ticks = now, feedback.ticks
        elif now - self.quiet_at >= SETTLE_S:
            self.source_cutoff = self.wall_ns()
            self.state = State.READY

    def send_velocity(self, starting):
        self.check_session()
        self.require_fresh()
        now = self.now()
        if starting:
            target = tuple(direction * MIN_ACTIVE_RPM for direction in directions(self.limited))
        else:
            deltas = tuple(goal - current for current, goal in zip(self.sent, self.limited))
            step = RAMP_RPM_PER_S * min(LOOP_PERIOD_S, now - self.last_send)
            largest = max(map(abs, deltas))
            fraction = min(1.0, step / largest) if largest else 1.0
            target = tuple(current + fraction * delta for current, delta in zip(self.sent, deltas))
        formatted = tuple(f'{rpm:.9f}' for rpm in target)
        self.uart.exchange(f'V {formatted[0]} {formatted[1]}', 'OK V')
        self.sent = tuple(map(float, formatted))
        self.last_send = now
        self.state = State.RUNNING

    def tick(self):
        try:
            if self.state != State.FAULT_LATCHED:
                self.check_session()
            if self.state == State.RUNNING:
                self.require_fresh()
            self.query()
            if self.state == State.FAULT_LATCHED:
                return
            running = self.state == State.RUNNING
            self.check_feedback(armed=running, running=running)
            if self.state == State.STOPPING:
                self.settle()
            elif running:
                self.send_velocity(starting=False)
            elif self.state == State.READY and self.command_at is not None:
                self.require_fresh()
                if not self.feedback.stationary:
                    raise ProtocolError('wheels moved before arming')
                self.check_session()
                if self.session_deadline is None:
                    self.session_deadline = self.now() + SESSION_MAX_S
                self.uart.exchange('ARM_CONT', 'OK ARMED CONT')
                self.query()
                self.check_feedback(armed=True, running=False)
                self.send_velocity(starting=True)
        except (RuntimeError, OSError) as exc:
            self.latch(str(exc))

    def status(self):
        feedback = self.feedback
        return {
            'version': '3.0',
            'firmware_protocol_version': self.protocol,
            'state': self.state.value,
            'requested_rpm': self.requested,
            'limited_rpm': self.limited,
            'sent_rpm': self.sent,
            'measured_rpm': feedback.measured if feedback else None,
            'pwm_duty_percent': feedback.duty if feedback else None,
            'command_age_s': self.now() - self.command_at if self.command_at is not None else None,
            'feedback_age_s': self.now() - self.feedback_at if feedback else None,
            'firmware_armed': feedback.armed if feedback else None,
            'firmware_running': feedback.running if feedback else None,
            'firmware_mode': feedback.mode if feedback else None,
            'firmware_fault': feedback.fault if feedback else None,
            'firmware_fault_reason': feedback.reason if feedback else None,
            'fault_latched': self.state == State.FAULT_LATCHED,
            'fault_reason': self.fault_reason,
            'uart_errors': feedback.counters[0] if feedback else None,
            'control_faults': feedback.counters[1] if feedback else None,
        }

    def shutdown(self):
        try:
            self.verify_stop()
        except (RuntimeError, OSError) as exc:
            self.report(f'FINAL STOP UNVERIFIED: {exc}. CUT MOTOR POWER PHYSICALLY.')
            return False
        self.report('Final STOP confirmed by Q: A=0 RUN=0')
        return True


def run_ros(port_name, bench):
    import fcntl
    import termios

    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.clock import Clock, ClockType
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    import serial
    from std_msgs.msg import String

    port = node = bridge = None
    result = 0
    rclpy.init()
    try:
        node = rclpy.create_node('amr_serial_bridge')
        joint_pub = node.create_publisher(JointState, '/joint_states', 10)
        status_pub = node.create_publisher(String, '/amr_bridge/status', 10)
        port = serial.Serial(port_name, UART_BAUD, timeout=0.004, write_timeout=0.025,
                             exclusive=True)
        fcntl.ioctl(port.fileno(), termios.TIOCEXCL)
        bridge = GuardedBridge(Uart(port), bench=bench)
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
                msg.velocity = [rpm * 2 * math.pi / 60.0 for rpm in feedback.measured]
                joint_pub.publish(msg)
            status_pub.publish(String(data=json.dumps(bridge.status(), allow_nan=False)))

        def on_command(msg):
            twist = msg.twist
            components = (twist.linear.x, twist.linear.y, twist.linear.z,
                          twist.angular.x, twist.angular.y, twist.angular.z)
            stamp = msg.header.stamp
            if stamp.nanosec < 10**9:
                bridge.command(components, stamp.sec * 10**9 + stamp.nanosec)

        if bench:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             lifespan=Duration(seconds=DEADMAN_S))
            node.create_subscription(TwistStamped, BENCH_TOPIC, on_command, qos)
        node.create_timer(LOOP_PERIOD_S, loop, clock=Clock(clock_type=ClockType.STEADY_TIME))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (RuntimeError, OSError) as exc:
        print(f'Bridge stopped: {exc}', file=sys.stderr, flush=True)
        result = 1
    finally:
        try:
            if bridge is not None and not bridge.shutdown():
                result = 1
        finally:
            if port is not None:
                port.close()
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/serial0')
    parser.add_argument('--continuous-bench', action='store_true',
                        help='enable supervised motion with both wheels securely raised')
    args = parser.parse_args()
    if args.continuous_bench:
        if not sys.stdin.isatty():
            print('Bench mode requires an interactive operator confirmation.', file=sys.stderr)
            return 1
        try:
            answer = input('Securely raise both wheels and verify the physical motor cutoff. '
                           'Type WHEELS_UP to enable this bench session: ')
        except (EOFError, KeyboardInterrupt):
            return 1
        if answer.strip() != 'WHEELS_UP':
            return 1
    try:
        return run_ros(args.port, args.continuous_bench)
    except ImportError as exc:
        print(f'Bridge dependency unavailable: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
