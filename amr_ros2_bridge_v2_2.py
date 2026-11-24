#!/usr/bin/env python3
"""AMR ROS 2 / ESP32-S3 UART Bridge V2.2 (ROS 2 Humble).

DEFAULT: read-only. /cmd_vel cannot start motors.
--bench-drive: one <=2 s wheels-raised motion, then latched STOP.
--ground-test: one <=1 s floor motion, then latched STOP (legacy V2.1).
--ground-session: one <=3 s floor session, then latched STOP.

The ESP32 firmware has a HARD 5-second ARM window: a repeated V command does
NOT extend it. Therefore this bridge NEVER runs past 3 seconds per session.
This is NOT continuous/Nav2 control or a safety-rated device. A tested physical
motor power cut-off/E-stop and fuse are REQUIRED. Only one UART owner allowed.
"""
import argparse
import json
import math
import sys
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

CPR = 3960.0
WHEEL_RADIUS_M = 0.0435
WHEEL_SEPARATION_M = 0.300
UART_BAUD = 57600
LOOP_PERIOD_S = 0.10
DEADMAN_S = 0.23
MAX_WHEEL_RPM = 20.0
MIN_ACTIVE_RPM = 5.0  # Current ESP32 Phase4 firmware rejects nonzero <5 RPM.
MAX_LINEAR_MPS = 0.08
MAX_ANGULAR_RPS = 0.45
SESSION_MAX_S = 3.0  # Safely below ESP32 ARM_WINDOW_US=5,000,000
RPM_RAMP_STEP = 2.0    # per 100 ms, after minimum 5 RPM initial step


class AMRBridgeV2(Node):
    def __init__(self, port: str, mode: str):
        super().__init__('amr_serial_bridge')
        self.mode = mode
        self.duration_limit = {
            'read-only': 0.0, 'bench': 2.0,
            'ground': 1.0, 'ground-session': SESSION_MAX_S,
        }[mode]
        # These per-session limits are more conservative than V2.1.
        self.rpm_limit = 15.0 if mode == 'ground-session' else MAX_WHEEL_RPM
        self.linear_limit = 0.06 if mode == 'ground-session' else MAX_LINEAR_MPS
        self.angular_limit = 0.35 if mode == 'ground-session' else MAX_ANGULAR_RPS
        self.ser = serial.Serial(port, UART_BAUD, timeout=0.15, write_timeout=0.15)
        self.ser.reset_input_buffer()
        self.latched = False
        self.running = False
        self.start_time = 0.0
        self.last_cmd_time = 0.0
        self.last_query_time = 0.0
        self.target = (0.0, 0.0)
        self.sent = (0.0, 0.0)
        self.fault_baseline = None
        self.uart_baseline = None
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.bridge_pub = self.create_publisher(String, '/amr_bridge/status', 10)
        # Depth 1 prevents building up stale /cmd_vel commands in the subscriber queue.
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 1)

        self.exchange('STOP', 'OK STOP')
        self.exchange('T 000001', 'ACK 000001')
        startup = self.exchange('Q', 'STATUS ')
        idle = self.parse_status(startup)
        if idle['A'] != '0' or idle['RUN'] != '0':
            raise RuntimeError('Startup refused: firmware is not safely idle')
        self.publish_status(startup)
        self.publish_bridge_state()
        self.get_logger().info('Connected to ESP32-S3. Mode: ' + mode)
        if mode == 'read-only':
            self.get_logger().info('READ-ONLY: /cmd_vel cannot move motors')
        else:
            self.get_logger().warning(
                f'GUARDED {mode.upper()}: single motion, {self.duration_limit:.1f}s max; '
                'a STOP requires restarting this bridge')
        self.timer = self.create_timer(LOOP_PERIOD_S, self.loop)

    def exchange(self, command: str, prefix: str) -> str:
        """Synchronous UART request/reply; tolerate stale Q telemetry before STOP.

        The firmware speaks CR/LF-delimited lines. A Q reply can still be in the
        input buffer during shutdown; reading only one line can mistake STATUS
        for the acknowledgement of STOP. Never treat STATUS itself as STOP ACK.
        """
        self.ser.write((command + '\r').encode('ascii'))
        self.ser.flush()
        deadline = time.monotonic() + 0.65
        last_response = None
        while time.monotonic() < deadline:
            response = self.ser.readline().decode('ascii', errors='replace').strip()
            if not response:
                continue
            if response.startswith(prefix):
                return response
            last_response = response
            if command == 'STOP' and response.startswith('STATUS '):
                continue  # delayed Q reply; keep waiting for actual OK STOP
            raise RuntimeError(
                f'UART {command!r}: expected {prefix!r}, received {response!r}'
            )
        raise RuntimeError(
            f'UART {command!r}: no {prefix!r} acknowledgement within timeout; '
            f'last={last_response!r}'
        )

    def stop_and_latch(self, reason: str) -> None:
        if self.latched:
            return
        self.latched = True
        self.running = False
        self.target = (0.0, 0.0)
        self.sent = (0.0, 0.0)
        self.get_logger().warning('LATCHED STOP: ' + reason)
        try:
            self.exchange('STOP', 'OK STOP')
        except Exception as exc:
            self.get_logger().error(f'STOP unconfirmed: {exc}; CUT MOTOR POWER PHYSICALLY')

    @staticmethod
    def is_zero(value: float) -> bool:
        return abs(value) < 1e-6

    def on_cmd_vel(self, msg: Twist) -> None:
        if self.mode == 'read-only' or self.latched:
            return
        components = [
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        ]
        if not all(math.isfinite(float(v)) for v in components):
            self.stop_and_latch('non-finite Twist')
            return
        if any(abs(float(v)) > 1e-4 for v in
               (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)):
            self.stop_and_latch('unsupported lateral/3D Twist')
            return
        vx, wz = float(msg.linear.x), float(msg.angular.z)
        if abs(vx) > self.linear_limit or abs(wz) > self.angular_limit:
            self.stop_and_latch('Twist exceeds test limits')
            return
        if self.is_zero(vx) and self.is_zero(wz):
            if self.running or self.last_cmd_time:
                self.stop_and_latch('zero /cmd_vel')
            return

        left_mps = vx - wz * WHEEL_SEPARATION_M / 2.0
        right_mps = vx + wz * WHEEL_SEPARATION_M / 2.0
        scale = 60.0 / (2.0 * math.pi * WHEEL_RADIUS_M)
        requested = (left_mps * scale, right_mps * scale)
        for rpm in requested:
            if abs(rpm) > self.rpm_limit:
                self.stop_and_latch('wheel RPM exceeds test limit')
                return
            if 0.1 < abs(rpm) < MIN_ACTIVE_RPM:
                self.stop_and_latch('unsupported wheel RPM below firmware minimum')
                return
        # Firmware requires STOP before reversing an active wheel.
        if self.running and any(a * b < -0.01 for a, b in zip(self.sent, requested)):
            self.stop_and_latch('direction reversal while moving')
            return
        self.target = tuple(0.0 if abs(x) < 0.1 else x for x in requested)
        self.last_cmd_time = time.monotonic()

    def publish_bridge_state(self) -> None:
        msg = String()
        msg.data = json.dumps({
            'version': '2.2',
            'mode': self.mode,
            'running': self.running,
            'latched_stop': self.latched,
            'max_motion_seconds': self.duration_limit,
            'target_rpm': [round(float(v), 2) for v in self.target],
            'sent_rpm': [round(float(v), 2) for v in self.sent],
        }, separators=(',', ':'))
        self.bridge_pub.publish(msg)

    @staticmethod
    def parse_status(response: str) -> dict:
        parts = {}
        for token in response.split()[1:]:
            if '=' in token:
                k, v = token.split('=', 1)
                parts[k] = v
        for required in ('A', 'RUN', 'L', 'R', 'TL', 'TR', 'UE', 'CF'):
            if required not in parts:
                raise RuntimeError('Malformed status, missing ' + required)
        return parts

    def publish_status(self, response: str) -> None:
        fields = self.parse_status(response)
        if self.fault_baseline is None:
            self.fault_baseline = int(fields['CF'])
            self.uart_baseline = int(fields['UE'])
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [
            int(fields['TL']) * 2 * math.pi / CPR,
            int(fields['TR']) * 2 * math.pi / CPR,
        ]
        msg.velocity = [
            float(fields['L'].split('/')[1]) * 2 * math.pi / 60.0,
            float(fields['R'].split('/')[1]) * 2 * math.pi / 60.0,
        ]
        self.joint_pub.publish(msg)
        if self.running:
            if fields['A'] != '1' or fields['RUN'] != '1':
                self.stop_and_latch('ESP disarmed/inactive during motion')
            elif (int(fields['CF']) > self.fault_baseline or
                  int(fields['UE']) > self.uart_baseline):
                self.stop_and_latch('new ESP UART/control fault')

    @staticmethod
    def ramp(current: float, wanted: float) -> float:
        if abs(wanted) < 0.1:
            return 0.0
        if abs(current) < 0.1:
            return math.copysign(MIN_ACTIVE_RPM, wanted)
        delta = max(-RPM_RAMP_STEP, min(RPM_RAMP_STEP, wanted - current))
        result = current + delta
        if 0.1 < abs(result) < MIN_ACTIVE_RPM:
            result = math.copysign(MIN_ACTIVE_RPM, wanted)
        return result

    def loop(self) -> None:
        try:
            now = time.monotonic()
            if self.mode != 'read-only' and not self.latched:
                if self.running:
                    if now - self.start_time >= self.duration_limit:
                        self.stop_and_latch('motion time limit reached')
                    elif now - self.last_cmd_time > DEADMAN_S:
                        self.stop_and_latch('/cmd_vel timeout (deadman)')
                elif self.last_cmd_time and now - self.last_cmd_time <= DEADMAN_S:
                    self.exchange('ARM', 'OK ARMED 5S')
                    self.running = True
                    self.start_time = time.monotonic()
                    self.sent = (0.0, 0.0)
                    self.get_logger().warning('MOTOR TEST STARTED')
                if self.running:
                    nxt = tuple(self.ramp(a, b) for a, b in zip(self.sent, self.target))
                    self.exchange(f'V {nxt[0]:.2f} {nxt[1]:.2f}', 'OK V')
                    self.sent = nxt

            if now - self.last_query_time >= 0.18:
                self.last_query_time = now
                self.publish_status(self.exchange('Q', 'STATUS '))
            self.publish_bridge_state()
        except Exception as exc:
            self.get_logger().error(f'Bridge loop error: {exc}')
            self.stop_and_latch('UART/feedback failure')

    def close(self) -> None:
        # rclpy may have shut down its context after Ctrl+C, so avoid ROS logging
        # here; printing to stderr is safe even when the logger is invalid.
        try:
            if self.ser.is_open:
                try:
                    self.exchange('STOP', 'OK STOP')
                    fields = self.parse_status(self.exchange('Q', 'STATUS '))
                    if fields['A'] != '0' or fields['RUN'] != '0':
                        raise RuntimeError('ESP32 reports non-idle state after STOP')
                    print('Final STOP confirmed: A=0 RUN=0', flush=True)
                except Exception as exc:
                    print(
                        f'FINAL STOP NOT VERIFIED: {exc}; '
                        'CUT MOTOR POWER WITH PHYSICAL E-STOP',
                        file=sys.stderr,
                        flush=True,
                    )
                finally:
                    self.ser.close()
        except Exception as exc:
            print(f'UART close failure: {exc}', file=sys.stderr, flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description='Guarded ROS 2 ESP32 AMR bridge')
    parser.add_argument('--port', default='/dev/serial0')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--bench-drive', action='store_true')
    mode.add_argument('--ground-test', action='store_true')
    mode.add_argument('--ground-session', action='store_true',
                      help='One <=3-second guarded ground motion; NOT for Nav2')
    args = parser.parse_args()
    selected = ('bench' if args.bench_drive else
                'ground' if args.ground_test else
                'ground-session' if args.ground_session else 'read-only')
    if selected != 'read-only':
        print(f'{selected.upper()} TEST: one short run only; not intended for Nav2')
        print('Ensure physical E-stop, fuse and clear area; no other UART owner.')
        if selected == 'bench':
            print('Both wheels MUST be securely raised from the floor.')
            expected = 'WHEELS_UP'
        else:
            print('Robot on clear, level floor. Low speed only.')
            expected = 'GROUND_SESSION_READY' if selected == 'ground-session' else 'GROUND_READY'
        if input(f'Type exactly {expected} to arm bridge: ').strip() != expected:
            print('Cancelled; no motors enabled')
            return 1
    rclpy.init()
    node = None
    try:
        node = AMRBridgeV2(args.port, selected)
        rclpy.spin(node)
    except (KeyboardInterrupt, serial.SerialException, RuntimeError, OSError) as exc:
        print('Bridge stopped:', repr(exc))
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
