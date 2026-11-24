"""Offline bridge tests. Fake UART exchanges real protocol frames without a device."""
import contextlib
import io
import math
import sys
import types
import unittest
from unittest.mock import patch

from amr_ros2_bridge_v3 import (
    BENCH_TOPIC, DEADMAN_S, LOOP_PERIOD_S, MAX_ANGULAR_RPS, MAX_LINEAR_MPS, MAX_WHEEL_RPM,
    SESSION_MAX_S, SESSION_STOP_MARGIN_S, VERSION_REPLY, WHEEL_RADIUS_M, WHEEL_SEPARATION_M, ZERO,
    GuardedBridge, State, Uart, main, run_ros,
)


FORWARD = (0.05, 0, 0, 0, 0, 0)
REVERSE = (-0.05, 0, 0, 0, 0, 0)
STOPPED = (0, 0, 0, 0, 0, 0)


class Clock:
    def __init__(self):
        self.seconds = 1.0

    def now(self):
        return self.seconds

    def wall(self):
        return 1_700_000_000_000_000_000 + round(self.seconds * 1e9)


class FakeSerial:
    def __init__(self, clock, legacy=False):
        self.clock = clock
        self.legacy = legacy
        self.rx = bytearray()
        self.log = []
        self.override = {}
        self.armed = self.running = False
        self.still = True
        self.fault = None
        self.boot = 'aaccee01'
        self.ue = self.cf = 0
        self.speed = ZERO
        self.ticks = (0, 0)
        self.arm_at = self.last_v = None

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, size):
        if not self.rx:
            self.clock.seconds += 0.004
        result = bytes(self.rx[:size])
        del self.rx[:size]
        return result

    def expire(self):
        if self.running and self.clock.now() - self.last_v >= 0.30:
            self.trip('HEARTBEAT')
        elif self.armed and (self.legacy or not self.running) and self.clock.now() - self.arm_at >= 5:
            if self.running:
                self.trip('ARM_EXPIRED')
            else:
                self.armed = False

    def trip(self, reason):
        self.fault = reason
        self.cf += 1
        self.armed = self.running = False

    def reply(self, line):
        if line == 'Q':
            status = (f'STATUS A={int(self.armed)} RUN={int(self.running)} '
                      f'L=+0.0/{self.speed[0]} R=+0.0/{self.speed[1]} '
                      f'TL={self.ticks[0]} TR={self.ticks[1]} UE={self.ue} CF={self.cf}')
            if not self.legacy:
                mode = 'CONT' if self.armed else 'IDLE'
                status += (f' DL=0 DR=0 MODE={mode} FL={int(bool(self.fault))} '
                           f'FR={self.fault or "NONE"} ST={int(self.still and not self.running)} '
                           f'BOOT={self.boot}')
            return status
        if line == 'VER':
            return 'ERR COMMAND' if self.legacy else VERSION_REPLY
        if line == 'STOP':
            self.running = self.armed = False
            return 'OK STOP'
        if line.startswith('T '):
            return 'ACK ' + line[2:]
        if line == 'ARM_CONT':
            if self.legacy or self.fault or not self.still:
                return 'ERR ARM'
            self.armed = True
            self.arm_at = self.clock.now()
            return 'OK ARMED CONT'
        if line.startswith('V ') and self.armed:
            self.running = True
            self.last_v = self.clock.now()
            return 'OK V'
        return 'ERR COMMAND'

    def write(self, frame):
        self.expire()
        line = frame.decode('ascii').removesuffix('\r')
        self.log.append(line)
        if line in self.override:
            response = self.override[line]
        else:
            response = self.reply(line)
        if response is not None:
            self.rx.extend((response + '\r\n').encode('ascii'))
        return len(frame)

    def fileno(self):
        return -1

    def close(self):
        self.closed = True


class BridgeTest(unittest.TestCase):
    def make_bridge(self, bench=True, legacy=False):
        self.clock = Clock()
        self.serial = FakeSerial(self.clock, legacy=legacy)
        self.messages = []
        self.bridge = GuardedBridge(Uart(self.serial, self.clock.now), bench,
                                    self.clock.now, self.clock.wall, self.messages.append)
        self.bridge.start()
        if bench and not legacy:
            self.settle()

    def advance(self, seconds=LOOP_PERIOD_S):
        self.clock.seconds += seconds
        self.bridge.tick()

    def settle(self):
        for _ in range(8):
            self.advance()
        self.assertEqual(self.bridge.state, State.READY)

    def command(self, components=FORWARD, stamp=None):
        self.clock.seconds += 0.001
        self.bridge.command(components, self.clock.wall() if stamp is None else stamp)

    def start_motion(self, components=FORWARD):
        self.command(components)
        self.advance()
        self.assertEqual(self.bridge.state, State.RUNNING)

    def velocities(self):
        return [tuple(map(float, line.split()[1:])) for line in self.serial.log if line.startswith('V ')]

    def test_default_read_only_never_arms(self):
        self.make_bridge(bench=False, legacy=True)
        self.command()
        self.advance()
        self.assertEqual(self.bridge.state, State.READ_ONLY)
        self.assertEqual(self.serial.log, ['Q', 'Q'])
        self.assertEqual(self.bridge.status()['pwm_duty_percent'], (None, None))
        self.assertIsNone(self.bridge.status()['firmware_protocol_version'])

    def test_old_firmware_cannot_arm_continuous(self):
        self.make_bridge(legacy=True)
        self.command()
        self.advance()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertIn('capability', self.bridge.fault_reason)
        self.assertNotIn('ARM_CONT', self.serial.log)
        self.assertIn('STOP', self.serial.log)

    def test_repeated_commands_maintain_motion_past_five_seconds(self):
        self.make_bridge()
        self.start_motion()
        for _ in range(100):
            self.command()
            self.advance()
        self.assertGreater(self.clock.now() - self.serial.arm_at, 5)
        self.assertEqual(self.bridge.state, State.RUNNING)
        self.assertEqual(self.serial.log.count('ARM_CONT'), 1)
        self.assertTrue(self.serial.running)

    def test_zero_stops_and_waits_for_wheels(self):
        self.make_bridge()
        self.start_motion()
        self.serial.still = False
        self.serial.speed = (4., -4.)
        self.command(STOPPED)
        self.assertEqual(self.bridge.state, State.STOPPING)
        self.assertEqual(self.serial.log[-1], 'Q')
        self.assertFalse(self.serial.armed)
        for _ in range(8):
            self.advance()
        self.assertEqual(self.bridge.state, State.STOPPING)
        self.serial.still = True
        self.serial.speed = ZERO
        self.settle()

    def test_fresh_command_restarts_after_confirmed_stop(self):
        self.make_bridge()
        self.start_motion()
        deadline = self.bridge.session_deadline
        self.command(STOPPED)
        self.settle()
        self.start_motion()
        self.assertEqual(self.serial.log.count('ARM_CONT'), 2)
        self.assertEqual(self.bridge.session_deadline, deadline)

    def test_missing_commands_latch_before_firmware_heartbeat_expires(self):
        self.make_bridge()
        self.start_motion()
        self.advance(DEADMAN_S + .001)
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertIn('deadman', self.bridge.fault_reason)
        self.assertFalse(self.serial.running)
        self.assertFalse(self.serial.fault)
        self.command()
        self.advance()
        self.assertEqual(self.serial.log.count('ARM_CONT'), 1)

    def test_independent_heartbeat_can_trip_while_bridge_is_absent(self):
        self.make_bridge()
        self.start_motion()
        self.clock.seconds += 0.31
        self.serial.expire()
        self.assertEqual(self.serial.fault, 'HEARTBEAT')
        self.bridge.tick()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertEqual(self.bridge.status()['firmware_fault_reason'], 'HEARTBEAT')

    def test_uart_timeout_latches_and_requests_stop(self):
        self.make_bridge()
        self.start_motion()
        self.serial.override['Q'] = None
        start = self.clock.now()
        self.advance()
        self.assertLess(self.clock.now() - start, 0.25)
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertIn('UART timeout', self.bridge.fault_reason)
        self.assertFalse(self.serial.running)
        self.assertTrue(any('CUT MOTOR POWER' in line for line in self.messages))

    def test_bad_ack_latches(self):
        self.make_bridge()
        self.serial.override['ARM_CONT'] = 'OK ARMED CONT extra'
        self.command()
        self.advance()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertFalse(self.velocities())

    def test_firmware_fault_or_counter_change_while_idle_blocks_rearm(self):
        for field in ('fault', 'ue', 'boot'):
            with self.subTest(field=field):
                self.make_bridge()
                if field == 'fault':
                    self.serial.trip('ENCODER')
                else:
                    setattr(self.serial, field, 1 if field == 'ue' else 'aaccee02')
                self.advance()
                self.command()
                self.advance()
                self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
                self.assertNotIn('ARM_CONT', self.serial.log)
                if field == 'fault':
                    self.assertEqual(self.serial.fault, 'ENCODER')

    def test_unexpected_disarm_during_motion_latches_before_next_v(self):
        self.make_bridge()
        self.start_motion()
        self.serial.armed = self.serial.running = False
        self.command()
        self.advance()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertEqual(len(self.velocities()), 1)

    def test_invalid_twists_stop_motion(self):
        for components in ((0, 0, 0, 0, 0, math.inf), (0, 0, math.nan, 0, 0, 0),
                           (.05, 0.01, 0, 0, 0, 0)):
            with self.subTest(components=components):
                self.make_bridge()
                self.start_motion()
                self.command(components)
                self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
                self.assertFalse(self.serial.armed)

    def test_wheel_and_body_limits_with_proportional_saturation_and_slew(self):
        self.make_bridge()
        saturated = (0.09, 0, 0, 0, 0, 0.24)
        self.start_motion(saturated)
        for _ in range(20):
            self.command(saturated)
            self.advance()
        status = self.bridge.status()
        self.assertAlmostEqual(max(map(abs, status['limited_rpm'])), MAX_WHEEL_RPM)
        self.assertAlmostEqual(status['limited_rpm'][0] / status['limited_rpm'][1],
                               status['requested_rpm'][0] / status['requested_rpm'][1])
        history = self.velocities()
        speed_scale = 2 * math.pi * WHEEL_RADIUS_M / 60
        for left, right in history:
            self.assertLessEqual(max(abs(left), abs(right)), MAX_WHEEL_RPM)
            self.assertLessEqual(abs(left + right) * speed_scale / 2, MAX_LINEAR_MPS)
            self.assertLessEqual(abs(right - left) * speed_scale / WHEEL_SEPARATION_M,
                                 MAX_ANGULAR_RPS)
        for before, after in zip(history, history[1:]):
            self.assertLessEqual(max(abs(a - b) for a, b in zip(before, after)), .900001)

    def test_reversal_waits_for_stationary_and_fresh_command(self):
        self.make_bridge()
        self.start_motion()
        self.serial.still = False
        self.command(REVERSE)
        self.assertEqual(self.bridge.state, State.STOPPING)
        self.assertFalse(self.serial.running)
        self.command(REVERSE)
        stale_reverse = self.clock.wall()
        self.advance()
        self.serial.still = True
        self.settle()
        self.command(REVERSE, stamp=stale_reverse)
        self.advance()
        self.assertEqual(self.bridge.state, State.READY)
        self.start_motion(REVERSE)
        self.assertEqual(self.velocities()[-1], (-5.0, -5.0))

    def test_low_speed_is_reported_without_rounding_up(self):
        for vx in (0.001, 1e-17):
            self.make_bridge()
            self.command((vx, 0, 0, 0, 0, 0))
            self.advance()
            self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
            self.assertIn('below 5 RPM', self.bridge.fault_reason)
            self.assertNotIn('ARM_CONT', self.serial.log)

    def test_cumulative_session_limit(self):
        self.make_bridge()
        self.start_motion()
        start = self.serial.arm_at
        while self.clock.now() < start + 10:
            self.command()
            self.advance()
        self.command(STOPPED)
        self.settle()
        self.start_motion()
        while self.clock.now() < start + SESSION_MAX_S:
            self.command()
            self.advance()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertIn('session limit', self.bridge.fault_reason)
        self.assertFalse(self.serial.running)
        self.assertLessEqual(self.serial.last_v, start + SESSION_MAX_S - SESSION_STOP_MARGIN_S)

    def test_shutdown_requires_ack_and_actual_idle_q(self):
        self.make_bridge()
        self.start_motion()
        self.assertTrue(self.bridge.shutdown())
        self.assertIn('confirmed by Q', self.messages[-1])
        self.assertEqual(self.serial.log[-3], 'STOP')
        self.assertEqual(self.serial.log[-1], 'Q')

        self.serial.running = self.serial.armed = True
        wrong_state = self.serial.reply('Q')
        self.serial.override['Q'] = wrong_state
        self.assertFalse(self.bridge.shutdown())
        self.assertIn('CUT MOTOR POWER', self.messages[-1])

        del self.serial.override['Q']
        self.serial.override['STOP'] = 'NO STOP ACK'
        self.assertFalse(self.bridge.shutdown())
        self.assertEqual(self.serial.log[-1], 'Q')

    def test_stale_or_queued_command_cannot_restart(self):
        self.make_bridge()
        self.start_motion()
        self.command(STOPPED)
        for _ in range(5):
            self.advance()
        queued_before_ready = self.clock.wall()
        self.settle()
        self.command(stamp=queued_before_ready)
        self.command(stamp=self.clock.wall() - round(DEADMAN_S * 2e9))
        self.command(stamp=self.clock.wall() + 1_000_000_000)
        self.advance()
        self.assertEqual(self.bridge.state, State.READY)
        self.assertEqual(self.serial.log.count('ARM_CONT'), 1)

    def test_freshness_rechecked_after_arm_transaction(self):
        self.make_bridge()
        original = self.serial.reply

        def delayed_arm(line):
            if line == 'ARM_CONT':
                self.clock.seconds += .06
            return original(line)

        self.serial.reply = delayed_arm
        self.advance(0.15)
        self.command(stamp=self.clock.wall() - 130_000_000)
        self.advance()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertFalse(self.velocities())

    def test_no_encoder_quiet_latches_on_settle_timeout(self):
        self.make_bridge()
        self.start_motion()
        self.serial.still = False
        self.command(STOPPED)
        self.advance(2.01)
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertIn('stationary', self.bridge.fault_reason)


class EntryPointTest(unittest.TestCase):
    def test_bench_requires_exact_interactive_confirmation(self):
        for terminal, reply, expected in ((False, 'WHEELS_UP', 1), (True, 'YES', 1),
                                          (True, 'WHEELS_UP', 0)):
            with self.subTest(terminal=terminal, reply=reply):
                with (patch.object(sys, 'argv', ['bridge', '--continuous-bench']),
                      patch.object(sys.stdin, 'isatty', return_value=terminal),
                      patch('builtins.input', return_value=reply),
                      patch('amr_ros2_bridge_v3.run_ros', return_value=0) as run,
                      contextlib.redirect_stderr(io.StringIO())):
                    self.assertEqual(main(), expected)
                    if expected:
                        run.assert_not_called()
                    else:
                        run.assert_called_once_with('/dev/serial0', True)

    def test_humble_adapter_closes_after_interrupt_or_ros_shutdown(self):
        try:
            import rclpy
            from geometry_msgs.msg import TwistStamped
            from rclpy.executors import ExternalShutdownException
        except ImportError as exc:
            self.skipTest(f'ROS 2 Humble environment unavailable: {exc}')
        for bench, shutdown in ((False, KeyboardInterrupt), (True, ExternalShutdownException)):
            with self.subTest(bench=bench):
                mock_serial = FakeSerial(Clock())
                serial_module = types.SimpleNamespace(Serial=lambda *args, **kwargs: mock_serial)

                def spin(node):
                    subscriptions = [(sub.topic_name, sub.msg_type) for sub in node.subscriptions]
                    self.assertNotIn('/cmd_vel', [topic for topic, _ in subscriptions])
                    self.assertEqual(
                        (BENCH_TOPIC, TwistStamped) in subscriptions, bench)
                    topics = {publisher.topic_name for publisher in node.publishers}
                    self.assertTrue({'/joint_states', '/amr_bridge/status'} <= topics)
                    rclpy.spin_once(node, timeout_sec=0.4)
                    raise shutdown

                with (patch.dict(sys.modules, {'serial': serial_module}),
                      patch.dict('os.environ', {'ROS_LOCALHOST_ONLY': '1', 'ROS_DOMAIN_ID': '174'}),
                      patch('fcntl.ioctl') as exclusive, patch('rclpy.spin', side_effect=spin),
                      contextlib.redirect_stderr(io.StringIO()) as stderr):
                    self.assertEqual(run_ros('mock-port', bench), 0)
                    exclusive.assert_called_once()
                self.assertTrue(mock_serial.closed)
                self.assertEqual(mock_serial.log[-3], 'STOP')
                self.assertEqual(mock_serial.log[-1], 'Q')
                self.assertNotIn('ARM_CONT', mock_serial.log)
                self.assertIn('Final STOP confirmed by Q', stderr.getvalue())


if __name__ == '__main__':
    unittest.main()
