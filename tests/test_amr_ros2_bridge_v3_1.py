"""Offline V3.1 checks with byte-level UART mocks and a fake clock."""
import contextlib
import csv
import io
import math
import sys
import types
import unittest
from unittest.mock import patch

from amr_ros2_bridge_v3 import (
    BENCH_TOPIC, DEADMAN_S, LOOP_PERIOD_S, MAX_ANGULAR_RPS, MAX_LINEAR_MPS,
    MAX_WHEEL_RPM, SESSION_MAX_S, SESSION_STOP_MARGIN_S, VERSION_REPLY,
    WHEEL_SEPARATION_M, GuardedBridge, State, Uart, ZERO,
)
from amr_ros2_bridge_v3_1 import (
    CAPABILITIES_DISABLED, LOW_CAPABILITIES, LOW_TOPIC, MPS_PER_RPM, BridgeV31,
    Mode, Nav2Monitor, Transition, main, run_ros,
)
from test_amr_ros2_bridge_v3 import Clock, FakeSerial, FORWARD, REVERSE, STOPPED
from tools.amr_v3_low_speed_test import Run, characterize


def straight(rpm):
    return (rpm * MPS_PER_RPM, 0., 0., 0., 0., 0.)


class Firmware31(FakeSerial):
    def __init__(self, clock):
        super().__init__(clock)
        self.low = None
        self.erase_run = False
        self.freeze = False
        self.capabilities = LOW_CAPABILITIES

    def expire(self):
        super().expire()
        if self.armed and self.low is not None and self.clock.now() - self.arm_at >= 15.0:
            self.trip('LOW_SESSION_EXPIRED')

    def reply(self, line):
        if line == 'CAPS':
            return self.capabilities
        if line.startswith('ARM_LOW '):
            fields = line.split()
            candidate = int(fields[1])
            if len(fields) != 3 or fields[2] != 'WHEELS_UP' or abs(candidate) not in (2, 3, 4, 5):
                self.trip('BAD_ARM_LOW')
                return 'ERR BAD_ARM_LOW'
            if self.armed or self.fault or not self.still:
                return 'ERR LOW ARM'
            self.low = candidate
            self.armed = True
            self.arm_at = self.clock.now()
            return 'OK ARMED LOW'
        if line == 'STOP':
            self.low = None
            self.speed = ZERO
        if line.startswith('V ') and self.armed:
            targets = tuple(map(float, line.split()[1:]))
            if (self.low is None and any(0 < abs(v) < 5 for v in targets)) or (
                    self.low is not None and targets != ZERO and targets != (self.low, self.low)):
                self.trip('LOW_SETPOINT')
                return 'ERR LOW_SETPOINT'
            if targets == ZERO:
                self.armed = self.running = False
                self.low = None
                return 'OK V'
            self.speed = targets if not self.freeze else ZERO
        response = super().reply(line)
        if line == 'Q':
            if self.armed and self.low is not None:
                response = response.replace('MODE=CONT', 'MODE=LOW')
            if self.running:
                if self.erase_run:
                    response = response.replace('RUN=1', 'RUN=0')
                if not self.freeze:
                    self.ticks = tuple(n + math.copysign(4, v) if v else n
                                       for n, v in zip(self.ticks, self.speed))
                    self.ticks = tuple(map(int, self.ticks))
                response = response.replace(' DL=0 DR=0 ', ' DL=15.2 DR=14.0 ')
            return f'{response} PV=3.1 SC=+0.00'
        return response


class Bridge31Test(unittest.TestCase):
    def setup_bridge(self, mode=Mode.LOW, candidate=2, firmware=Firmware31, preflight=True):
        self.clock = Clock()
        self.serial = firmware(self.clock)
        self.messages = []
        self.bridge = BridgeV31(Uart(self.serial, self.clock.now), mode,
                                candidate if mode == Mode.LOW else None,
                                monotonic=self.clock.now, wall_ns=self.clock.wall,
                                report=self.messages.append)
        if preflight:
            self.bridge.start()
            if self.bridge.state == State.STOPPING:
                self.settle()

    def settle(self):
        for _ in range(8):
            self.tick()
        self.assertEqual(self.bridge.state, State.READY)

    def tick(self, seconds=LOOP_PERIOD_S):
        self.clock.seconds += seconds
        self.bridge.tick()

    def command(self, components, stamp=None):
        self.clock.seconds += .001
        self.bridge.command(components, self.clock.wall() if stamp is None else stamp)

    def velocity_log(self):
        return [line for line in self.serial.log if line.startswith('V ')]

    def motion(self, components=None):
        if components is None:
            components = straight(self.bridge.low_rpm)
        self.command(components)
        self.tick()
        self.assertEqual(self.bridge.state, Transition.STARTING)
        self.tick()
        self.assertEqual(self.bridge.state, State.RUNNING)

    def test_default_read_only_and_legacy_read_only_never_arm(self):
        for firmware in (Firmware31, FakeSerial, lambda clock: FakeSerial(clock, legacy=True)):
            with self.subTest(firmware=firmware):
                self.setup_bridge(Mode.READ_ONLY, firmware=firmware)
                self.command(straight(2))
                self.tick()
                self.assertEqual(self.bridge.state, State.READ_ONLY)
                self.assertTrue(set(self.serial.log) <= {'Q', 'VER', 'CAPS'})

    def test_v3_bridge_accepts_extended_firmware_in_normal_mode(self):
        clock = Clock()
        serial = Firmware31(clock)
        baseline = GuardedBridge(Uart(serial, clock.now), True, clock.now, clock.wall)
        baseline.start()
        for _ in range(8):
            clock.seconds += LOOP_PERIOD_S
            baseline.tick()
        clock.seconds += .001
        baseline.command(FORWARD, clock.wall())
        baseline.tick()
        self.assertEqual(baseline.state, State.RUNNING)
        self.assertIn('VER', serial.log)
        self.assertIn('ARM_CONT', serial.log)
        self.assertNotIn('CAPS', serial.log)

    def test_v31_continuous_mode_negotiates_old_v3_and_keeps_five_rpm(self):
        self.setup_bridge(Mode.CONTINUOUS, firmware=FakeSerial)
        self.assertEqual(self.bridge.protocol, 3)
        self.motion(FORWARD)
        self.assertNotIn('CAPS', self.serial.log)
        self.assertEqual(self.velocity_log()[0], 'V 5.000000000 5.000000000')
        self.command(straight(4))
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)

    def test_low_requires_explicit_capabilities_and_does_not_probe_old_firmware(self):
        self.setup_bridge(firmware=FakeSerial)
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertNotIn('CAPS', self.serial.log)
        self.assertFalse(any(line.startswith('ARM') for line in self.serial.log))

        self.setup_bridge(preflight=False)
        self.serial.capabilities = CAPABILITIES_DISABLED
        self.bridge.start()
        self.assertEqual(self.bridge.protocol, '3.1')
        self.assertFalse(self.bridge.low_capable)
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)

        self.setup_bridge(preflight=False)
        self.serial.capabilities += ',UNEXPECTED'
        self.bridge.start()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)

    def test_all_candidates_in_both_directions_and_single_run_interlock(self):
        for rpm in (2, 3, 4, 5, -2, -3, -4, -5):
            with self.subTest(rpm=rpm):
                self.setup_bridge(candidate=rpm)
                self.assertTrue(self.bridge.low_capable)
                self.motion()
                self.assertIn(f'ARM_LOW {rpm} WHEELS_UP', self.serial.log)
                self.assertEqual(set(self.velocity_log()), {f'V {rpm} {rpm}'})
                self.assertEqual(self.bridge.status()['firmware_mode'], 'LOW')
                self.command(STOPPED)
                self.settle()
                self.command(straight(rpm))
                self.tick()
                self.assertEqual(self.bridge.state, State.READY)
                self.assertTrue(self.bridge.status()['stationary_ready'])
                self.assertEqual(self.serial.log.count(f'ARM_LOW {rpm} WHEELS_UP'), 1)

    def test_unsupported_low_target_is_inhibited_and_fault_prevents_motion(self):
        for invalid in (straight(1e-16), straight(1), straight(1.99999), straight(3), straight(-2),
                        (2 * MPS_PER_RPM, 0., 0., 0., 0., .02),
                        (2 * MPS_PER_RPM, 0., 0., math.inf, 0., 0.)):
            with self.subTest(invalid=invalid):
                self.setup_bridge()
                self.command(invalid)
                self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
                self.command(straight(2))
                self.tick()
                self.assertEqual(self.velocity_log(), [])

    def test_requested_ack_measured_and_run_confirmation_are_distinct(self):
        self.setup_bridge()
        self.command(straight(2))
        self.tick()
        first = self.bridge.status()
        self.assertEqual(first['state'], 'STARTING')
        self.assertEqual(first['requested_rpm'], (2., 2.))
        self.assertEqual(first['limited_rpm'], (2, 2))
        self.assertEqual(first['acknowledged_rpm'], (2, 2))
        self.assertFalse(first['firmware_run_confirmed'])
        self.assertFalse(first['firmware_running'])
        self.assertEqual(first['measured_rpm'], ZERO)
        self.serial.speed = (1.6, 2.2)
        self.tick()
        status = self.bridge.status()
        self.assertEqual(status['state'], 'RUNNING')
        self.assertTrue(status['firmware_run_confirmed'])
        self.assertEqual(status['acknowledged_rpm'], (2, 2))
        self.assertEqual(status['measured_rpm'], (1.6, 2.2))
        self.assertEqual(status['pwm_duty_percent'], (15.2, 14.0))
        self.assertEqual(status['straight_sync_correction_rpm'], 0.0)

    def test_ack_without_fresh_run_confirmation_latches(self):
        self.setup_bridge()
        self.command(straight(2))
        self.tick()
        self.serial.erase_run = True
        self.tick()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertEqual(len(self.velocity_log()), 1)
        self.assertFalse(self.bridge.status()['firmware_run_confirmed'])

    def test_stale_and_future_stamps_cannot_start(self):
        self.setup_bridge()
        stamp = self.clock.wall()
        self.tick(.25)
        self.command(straight(2), stamp)
        self.command(straight(2), self.clock.wall() + 100_000_000)
        self.tick()
        self.assertEqual(self.bridge.state, State.READY)
        self.assertEqual(self.velocity_log(), [])

    def test_reversal_requires_stop_and_fresh_command_in_continuous_mode(self):
        self.setup_bridge(Mode.CONTINUOUS)
        self.motion(FORWARD)
        self.serial.still = False
        self.command(REVERSE)
        self.assertEqual(self.bridge.state, State.STOPPING)
        self.assertEqual(self.serial.log[-1], 'Q')
        self.command(REVERSE)
        stale = self.clock.wall()
        self.tick()
        self.serial.still = True
        self.settle()
        self.command(REVERSE, stale)
        self.tick()
        self.assertEqual(self.bridge.state, State.READY)
        self.motion(REVERSE)
        self.assertTrue(self.velocity_log()[-1].startswith('V -'))

    def test_low_direction_change_stops_and_latches(self):
        self.setup_bridge()
        self.motion()
        self.command(straight(-2))
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertTrue(self.bridge.status()['stop_verified'])
        self.assertFalse(self.serial.armed)

    def test_deadman_and_firmware_heartbeat(self):
        for elapsed, firmware_fault in ((DEADMAN_S + .002, None), (.31, 'HEARTBEAT')):
            with self.subTest(elapsed=elapsed):
                self.setup_bridge()
                self.motion()
                self.clock.seconds += elapsed
                self.serial.expire()
                self.bridge.tick()
                self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
                self.assertEqual(self.serial.fault, firmware_fault)
                self.command(straight(2))
                self.tick()
                self.assertEqual(self.serial.log.count('ARM_LOW 2 WHEELS_UP'), 1)

    def test_fifteen_second_low_session_and_watchdog_reserve(self):
        self.setup_bridge()
        self.motion()
        end = self.bridge.session_deadline
        while self.bridge.state == State.RUNNING:
            self.command(straight(2))
            self.tick()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertIn('15-second', self.bridge.fault_reason)
        self.assertLessEqual(self.serial.last_v, end - SESSION_STOP_MARGIN_S)
        self.assertAlmostEqual(end - self.serial.arm_at, SESSION_MAX_S)

    def test_reset_counters_and_wrong_mode_latch(self):
        for change in ('boot', 'ue', 'mode'):
            with self.subTest(change=change):
                self.setup_bridge()
                self.motion()
                if change == 'boot':
                    self.serial.boot = 'beef9012'
                elif change == 'ue':
                    self.serial.ue += 1
                else:
                    self.serial.low = None
                self.command(straight(2))
                before = len(self.velocity_log())
                self.tick()
                self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
                self.assertEqual(len(self.velocity_log()), before)

    def test_characterization_csv_for_both_wheels_and_stationary_stop(self):
        self.setup_bridge(candidate=-3, preflight=False)
        run = Run(-3)
        self.assertTrue(characterize(self.bridge, .5, run, self.advance_clock))
        output = io.StringIO()
        run.write_csv(output)
        rows = list(csv.DictReader(io.StringIO(output.getvalue())))
        summary = [row for row in rows if row['record'] == 'summary']
        self.assertEqual([row['wheel'] for row in summary], ['left', 'right'])
        for row in summary:
            self.assertEqual(row['stationary_stop_verified'], 'True')
            self.assertLess(int(row['encoder_delta']), 0)
            self.assertLess(float(row['time_to_start_s']), .5)
            self.assertEqual(float(row['max_abs_error_rpm']), 0.)
        self.assertEqual(self.serial.log[-1], 'Q')
        self.assertFalse(self.serial.armed)

    def test_characterization_preserves_fault_and_still_verifies_stationary(self):
        self.setup_bridge(preflight=False)
        run = Run(2)

        def fault_after_start(seconds):
            if self.serial.running:
                self.serial.trip('STALL')
            self.advance_clock(seconds)

        self.assertFalse(characterize(self.bridge, 1., run, fault_after_start))
        self.assertTrue(run.stop_verified)
        self.assertEqual(run.firmware_fault, 'STALL')
        self.assertIn('STALL', run.event)
        self.assertEqual(self.serial.log.count('ARM_LOW 2 WHEELS_UP'), 1)

    def test_characterization_fails_if_stationary_stop_cannot_be_verified(self):
        self.setup_bridge(preflight=False)
        run = Run(2)

        def never_settle_after_start(seconds):
            if self.serial.running:
                self.serial.still = False
            self.advance_clock(seconds)

        with contextlib.redirect_stderr(io.StringIO()) as stderr:
            self.assertFalse(characterize(self.bridge, .5, run, never_settle_after_start))
        self.assertFalse(run.stop_verified)
        self.assertIn('CUT MOTOR POWER', stderr.getvalue())

    def advance_clock(self, seconds):
        self.clock.seconds += seconds


class MonitorTest(unittest.TestCase):
    @staticmethod
    def twist(vx=0., wz=0., lateral=0.):
        return types.SimpleNamespace(linear=types.SimpleNamespace(x=vx, y=lateral, z=0.),
                                     angular=types.SimpleNamespace(x=0., y=0., z=wz))

    def test_unstamped_twist_validation_and_near_zero_policy(self):
        self.assertEqual(Nav2Monitor.evaluate(self.twist()).disposition, 'STOP')
        for twist in (self.twist(vx=1e-20), self.twist(vx=.035, wz=.15),
                      self.twist(vx=math.inf), self.twist(lateral=.01), self.twist(wz=math.nan)):
            self.assertEqual(Nav2Monitor.evaluate(twist).disposition, 'INHIBITED')
        wz = .30
        pivot = Nav2Monitor.evaluate(self.twist(vx=wz * WHEEL_SEPARATION_M / 2, wz=wz))
        self.assertEqual(pivot.disposition, 'ELIGIBLE_FOR_REVIEW')
        self.assertEqual(pivot.limited_rpm[0], 0.)
        self.assertEqual(Nav2Monitor.evaluate(self.twist(vx=-.055), (8., 8.)).disposition,
                         'STOP_BEFORE_CHANGE')

    def test_proportional_saturation_and_sync_headroom(self):
        for vx, wz in ((.08, .25), (.055, 0.), (-.055, 0.), (0., .55)):
            preview = Nav2Monitor.evaluate(self.twist(vx, wz))
            self.assertEqual(preview.disposition, 'ELIGIBLE_FOR_REVIEW')
            l, r = preview.limited_rpm
            left, right = preview.requested_rpm
            self.assertAlmostEqual(l / left, r / right)
            self.assertLessEqual(max(preview.effective_abs_rpm_upper_bound), MAX_WHEEL_RPM)
            self.assertLessEqual(abs(l + r) * MPS_PER_RPM / 2, MAX_LINEAR_MPS)
            self.assertLessEqual(abs(r - l) * MPS_PER_RPM / WHEEL_SEPARATION_M, MAX_ANGULAR_RPS)
            expected_margin = 1. if vx > 0 and not wz else 0.
            self.assertAlmostEqual(preview.effective_abs_rpm_upper_bound[0] - abs(l), expected_margin)

    def test_monitor_cli_has_no_ros_or_serial_path(self):
        with (patch.object(sys, 'argv', ['bridge', '--monitor-twist', '.055', '0']),
              patch('amr_ros2_bridge_v3_1.run_ros') as run,
              contextlib.redirect_stdout(io.StringIO()) as output):
            self.assertEqual(main(), 0)
            run.assert_not_called()
            self.assertIn('ELIGIBLE_FOR_REVIEW', output.getvalue())

    def test_low_cli_requires_its_own_confirmation(self):
        with (patch.object(sys, 'argv', ['bridge', '--low-speed-bench', '--rpm', '2',
                                         '--direction', 'reverse']),
              patch.object(sys.stdin, 'isatty', return_value=True),
              patch('builtins.input', side_effect=['WHEELS_UP', 'WHEELS_UP_LOW_SPEED']),
              patch('amr_ros2_bridge_v3_1.run_ros', return_value=0) as run):
            self.assertEqual(main(), 1)
            run.assert_not_called()
            self.assertEqual(main(), 0)
            run.assert_called_once_with('/dev/serial0', Mode.LOW, -2)

    def test_ros_subscribes_only_to_the_selected_stamped_bench_topic(self):
        try:
            import rclpy
            from geometry_msgs.msg import Twist, TwistStamped
            from rclpy.executors import ExternalShutdownException
        except ImportError as exc:
            self.skipTest(f'ROS 2 Humble environment unavailable: {exc}')
        unstamped = Twist()
        unstamped.linear.x = .05
        self.assertEqual(Nav2Monitor.evaluate(unstamped).disposition, 'ELIGIBLE_FOR_REVIEW')
        for mode, expected in ((Mode.READ_ONLY, None), (Mode.CONTINUOUS, BENCH_TOPIC),
                               (Mode.LOW, LOW_TOPIC)):
            with self.subTest(mode=mode):
                serial = Firmware31(Clock())

                def spin(node):
                    subscriptions = {sub.topic_name: sub.msg_type for sub in node.subscriptions}
                    self.assertNotIn('/cmd_vel', subscriptions)
                    for topic in (BENCH_TOPIC, LOW_TOPIC):
                        self.assertEqual(subscriptions.get(topic), TwistStamped if expected == topic else None)
                    rclpy.spin_once(node, timeout_sec=.35)
                    raise ExternalShutdownException

                with (patch.dict(sys.modules, {'serial': types.SimpleNamespace(Serial=lambda *a, **k: serial)}),
                      patch.dict('os.environ', {'ROS_LOCALHOST_ONLY': '1', 'ROS_DOMAIN_ID': '175'}),
                      patch('fcntl.ioctl') as exclusive, patch('rclpy.spin', side_effect=spin),
                      contextlib.redirect_stderr(io.StringIO())):
                    self.assertEqual(run_ros('mock-port', mode, 2 if mode == Mode.LOW else None), 0)
                exclusive.assert_called_once()
                self.assertEqual(serial.log[-1], 'Q')
                self.assertTrue(serial.closed)


if __name__ == '__main__':
    unittest.main()
