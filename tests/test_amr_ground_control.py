"""Offline authority, motion, shutdown and publisher tests for Phase 4L.6C."""
import contextlib
import csv
import io
import math
import os
import sys
import time
import types
import unittest
from unittest.mock import patch

from amr_ground_protocol import (
    CLOCK_OFFSET_NS, NS, AuthorityError, CommandStamp,
    DDSStamp, clock_proven, command_qos, metadata_executor,
)
from amr_ros2_bridge_v3 import DEADMAN_S, LOOP_PERIOD_S, SESSION_STOP_MARGIN_S, State, Uart
from amr_ros2_bridge_v3_1 import MPS_PER_RPM, Transition
from amr_ros2_bridge_v3_2 import (
    MIN_START_ANGULAR, MIN_START_LINEAR, BridgeV32, Limits, main, run_ros,
)
from test_amr_ros2_bridge_v3 import Clock, FakeSerial, FORWARD, REVERSE, STOPPED
from test_amr_ros2_bridge_v3_1 import Firmware31
from tools.amr_cmd_vel_adapter import CommandAdapter
from tools.amr_ground_commissioning import CommissioningRun, PROFILES

ACTIVATION = '1' * 32
COMMAND_GID, CONTROL_GID, RAW_GID, STATUS_GID = (1,), (2,), (3,), (4,)


def dds(source, received=None, gid=None):
    data = {'source_timestamp': source, 'received_timestamp': source if received is None else received}
    if gid is not None:
        data['publisher_gid'] = gid
    return data


def twist(vx=.04, wz=0., lateral=0.):
    return types.SimpleNamespace(linear=types.SimpleNamespace(x=vx, y=lateral, z=0.),
                                 angular=types.SimpleNamespace(x=0., y=0., z=wz))


def wheel_speed(rpm):
    return (MPS_PER_RPM * rpm, 0., 0., 0., 0., 0.)


class GroundTest(unittest.TestCase):
    def setup(self, mode='ground', source='commissioning', firmware=Firmware31, duration=None):
        self.clock = Clock()
        self.serial = firmware(self.clock)
        self.log = []
        self.bridge = BridgeV32(Uart(self.serial, self.clock.now), mode, source, duration,
                                monotonic=self.clock.now, wall_ns=self.clock.wall,
                                report=self.log.append)
        self.bridge.start()
        if self.bridge.state == State.STOPPING:
            self.settle()

    def tick(self, dt=LOOP_PERIOD_S, renew=False):
        self.clock.seconds += dt
        if renew:
            self.prove()
        self.bridge.tick()

    def settle(self, renew=False):
        for _ in range(7):
            self.tick(renew=renew)
        self.assertEqual(self.bridge.state, State.READY)

    def prove(self, offset=0):
        t1 = self.bridge.issue_challenge()
        if t1 is None:
            return
        t2 = t3 = t1 + offset + 1_000_000
        self.clock.seconds += .002
        reply = dict(session=self.bridge.session, source=self.bridge.source, activation=ACTIVATION,
                     kind='clock', t1=t1, t2=t2, t3=t3)
        self.bridge.control(reply, dds(t3, self.clock.wall(), CONTROL_GID))

    def authorize(self):
        self.bridge.observe_publishers((COMMAND_GID,), (CONTROL_GID,))
        self.clock.seconds += .26
        self.prove()
        self.clock.seconds += .03
        self.assertEqual(self.bridge.activation, ACTIVATION)

    def command(self, components=FORWARD, *, source=None, session=None, sent=None, gid=COMMAND_GID):
        self.clock.seconds += .003
        wall = self.clock.wall()
        stamp = CommandStamp(session or self.bridge.session, ACTIVATION, source or self.bridge.source,
                             wall - 2_000_000 if sent is None else sent, wall - 1_000_000)
        self.bridge.command(components, stamp, dds(wall, gid=gid))

    def motion(self, components=FORWARD):
        self.command(components)
        self.tick()
        self.assertEqual(self.bridge.state, Transition.STARTING)
        self.assertFalse(self.bridge.status()['firmware_run_confirmed'])
        self.tick()
        self.assertEqual(self.bridge.state, State.RUNNING)

    def v_frames(self):
        return [line for line in self.serial.log if line.startswith('V ')]

    def test_default_read_only_and_no_ownership_means_no_arm(self):
        self.setup(None, None)
        self.command()
        self.tick()
        self.assertEqual(self.bridge.state, State.READ_ONLY)
        self.assertFalse(self.v_frames())
        self.assertEqual(self.bridge.status()['authority_state'], 'DISABLED')
        self.setup()
        self.command()
        self.tick()
        self.assertEqual(self.bridge.state, State.READY)
        self.assertEqual(self.bridge.status()['authority_state'], 'WAITING_FOR_OWNER')
        self.assertFalse(self.v_frames())

    def test_ground_rejects_older_protocol_raised_has_same_path(self):
        self.setup(firmware=FakeSerial)
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertIn('requires firmware V3.1', self.bridge.fault_reason)
        self.setup('raised')
        self.assertEqual(self.bridge.duration, 15)
        self.authorize()
        self.motion()
        self.assertIn('ARM_CONT', self.serial.log)
        self.assertFalse(any(cmd.startswith('ARM_LOW') for cmd in self.serial.log))

    def test_cli_confirmation_and_budget_bounds(self):
        argv = ['bridge', '--ground-control', '--source', 'teleop', '--session-seconds', '120']
        with (patch.object(sys, 'argv', argv), patch.object(sys.stdin, 'isatty', return_value=True),
              patch('builtins.input', side_effect=['READY', 'CLEAR_AREA_CUTOFF_READY']),
              patch('amr_ros2_bridge_v3_2.run_ros', return_value=0) as run):
            self.assertEqual(main(), 1)
            run.assert_not_called()
            self.assertEqual(main(), 0)
            self.assertEqual(run.call_args.args[1:4], ('ground', 'teleop', 120.))
        with self.assertRaises(ValueError):
            BridgeV32(None, 'ground', 'teleop', 121.)
        with self.assertRaises(ValueError):
            BridgeV32(None, 'raised', 'teleop', 10.)

    def test_session_budget_does_not_renew_after_stop_or_new_commands(self):
        self.setup()
        self.assertEqual(self.bridge.duration, 10)
        self.authorize()
        self.motion()
        deadline = self.bridge.session_deadline
        self.assertAlmostEqual(deadline - self.serial.arm_at, 10)
        self.command(STOPPED)
        self.assertEqual(self.bridge.state, State.STOPPING)
        self.assertTrue(self.bridge.stop_verified)
        self.settle(renew=True)
        self.motion()
        self.assertEqual(self.bridge.session_deadline, deadline)
        while self.bridge.state == State.RUNNING:
            self.command()
            self.tick(renew=True)
        self.assertIn('budget expired', self.bridge.fault_reason)
        self.assertLessEqual(self.serial.last_v, deadline - SESSION_STOP_MARGIN_S)
        self.assertFalse(self.serial.running)
        self.command()
        self.tick()
        self.assertEqual(self.serial.log.count('ARM_CONT'), 2)

    def test_one_wheel_below_min_stops_and_requires_reentry_dwell(self):
        self.setup()
        self.authorize()
        self.motion()
        # One wheel ~4 RPM, the other is supported.
        self.command((.04, 0., 0., 0., 0., .15))
        self.assertEqual(self.bridge.state, State.STOPPING)
        self.assertTrue(self.bridge.low_inhibited)
        self.assertEqual(self.bridge.stop_reason, 'LOW_SPEED_INHIBITED')
        self.assertIsNone(self.bridge.fault_reason)
        self.settle(renew=True)
        for _ in range(5):
            self.command(wheel_speed(5.2))
            self.tick(renew=True)
        self.assertEqual(self.serial.log.count('ARM_CONT'), 1)
        self.command(wheel_speed(6))
        self.tick()
        self.command(wheel_speed(4.99))
        self.tick()
        self.command(wheel_speed(6))
        self.tick()
        self.command(wheel_speed(6))
        self.tick()
        self.assertEqual(self.bridge.state, State.READY)
        self.command(wheel_speed(6))
        self.tick(renew=True)
        self.assertEqual(self.bridge.state, Transition.STARTING)
        self.assertEqual(self.v_frames()[-1], 'V 5.000000000 5.000000000')

    def test_tiny_input_is_normal_inhibition_and_zero_opens_exact_min_dwell(self):
        self.setup()
        self.authorize()
        self.command(wheel_speed(1e-14))
        self.tick()
        self.assertTrue(self.bridge.low_inhibited)
        self.assertFalse(self.v_frames())
        self.command(STOPPED)
        for _ in range(3):
            self.tick(renew=True)
            self.command(wheel_speed(5.1))
        self.tick()
        self.assertEqual(self.bridge.state, Transition.STARTING)

    def test_limit_saturation_reserves_sync_headroom_and_respects_slew(self):
        limits = Limits(.055, .25, 12.)
        request, bounded, body = limits.targets((.20, 0., 0., 0., 0., .40))
        self.assertAlmostEqual(bounded[0] / request[0], bounded[1] / request[1])
        self.assertLessEqual(max(bounded) + 1, 12.)
        self.assertLessEqual(body[0], .055)
        self.assertLessEqual(body[1], .25)
        self.setup()
        self.authorize()
        self.motion()
        for _ in range(18):
            self.command((.08, 0., 0., 0., 0., .14))
            self.tick(renew=True)
        targets = [tuple(map(float, line.split()[1:])) for line in self.v_frames()]
        for before, after in zip(targets, targets[1:]):
            self.assertLessEqual(max(abs(a - b) for a, b in zip(before, after)), .900001)
        self.assertLessEqual(max(max(target) for target in targets) + 1, 15)

    def test_minimum_commissioning_limits_can_represent_exact_supported_targets(self):
        limits = Limits(MIN_START_LINEAR, MIN_START_ANGULAR, 5.0)
        self.assertEqual(limits.targets((MIN_START_LINEAR, 0, 0, 0, 0, 0))[1], (5., 5.))
        self.assertEqual(limits.targets((0, 0, 0, 0, 0, MIN_START_ANGULAR))[1], (-5., 5.))

    def test_reversal_and_old_source_samples_need_post_ready_input(self):
        self.setup()
        self.authorize()
        self.motion()
        self.command(REVERSE)
        self.assertEqual(self.bridge.state, State.STOPPING)
        stale = self.clock.wall()
        self.settle(renew=True)
        self.command(REVERSE, sent=stale)
        self.command(REVERSE, session='2' * 32)
        self.tick()
        self.assertEqual(self.bridge.state, State.READY)
        self.motion(REVERSE)
        self.assertIn('V -5.000000000 -5.000000000', self.v_frames())

    def test_invalid_input_and_owner_loss_latch(self):
        for failure in ('invalid', 'source-choice', 'disappeared', 'other-gid', 'multiple'):
            with self.subTest(failure=failure):
                self.setup()
                self.authorize()
                self.motion()
                if failure == 'invalid':
                    self.command((math.nan, 0, 0, 0, 0, 0))
                    self.assertIsNone(self.bridge.status()['requested_body'])
                elif failure == 'source-choice':
                    self.command(source='teleop')
                elif failure == 'disappeared':
                    self.bridge.observe_publishers((), (CONTROL_GID,))
                elif failure == 'other-gid':
                    self.command(gid=RAW_GID)
                else:
                    self.bridge.observe_publishers((COMMAND_GID, RAW_GID), (CONTROL_GID,))
                self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
                self.assertFalse(self.serial.running)
                self.command()
                self.tick()
                self.assertEqual(self.serial.log.count('ARM_CONT'), 1)

    def test_stale_command_and_firmware_watchdog(self):
        for age, firmware_fault in ((DEADMAN_S + .005, None), (.32, 'HEARTBEAT')):
            self.setup()
            self.authorize()
            self.motion()
            self.clock.seconds += age
            self.serial.expire()
            self.bridge.tick()
            self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
            self.assertEqual(self.serial.fault, firmware_fault)
            self.assertTrue(self.bridge.stop_verified)

    def test_stale_original_source_without_fresh_restamp_latches(self):
        self.setup()
        self.authorize()
        self.clock.seconds += .25
        self.command(sent=self.clock.wall() - 240_000_000)
        # The stamps predate authorization, so the buffered input is discarded.
        self.assertFalse(self.v_frames())
        self.clock.seconds += .24
        self.command(sent=self.clock.wall() - 235_000_000)
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)

    def test_clock_mismatch_or_expired_proof_inhibits(self):
        self.setup()
        self.bridge.observe_publishers((COMMAND_GID,), (CONTROL_GID,))
        self.clock.seconds += .26
        self.prove(offset=2 * CLOCK_OFFSET_NS)
        self.assertEqual(self.bridge.clock_status, 'CLOCK_UNVERIFIED')
        self.command()
        self.tick()
        self.assertFalse(self.v_frames())
        self.prove()
        self.clock.seconds += .03
        self.motion()
        self.tick(1.01)
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertIn('clock proof', self.bridge.fault_reason)

    def test_firmware_reset_and_counter_jump_block_odometry_continuity(self):
        for change in ('boot', 'count'):
            self.setup()
            self.authorize()
            self.motion()
            if change == 'boot':
                self.serial.boot = 'ffeeddaa'
            else:
                self.serial.ticks = (2**30, 0)
            self.command()
            self.tick()
            self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
            self.assertFalse(self.bridge.status()['encoder_continuity_valid'])
            self.assertFalse(self.bridge.encoder_valid)

    def test_adapter_clock_handshake_and_velocity_end_to_end_with_mock_uart(self):
        for source in ('teleop', 'nav2'):
            with self.subTest(source=source):
                self.check_adapter_motion(source)

    def check_adapter_motion(self, source):
        self.setup(source=source)
        clock, bridge = self.clock, self.bridge
        laptop = lambda: clock.wall() + 5_000_000

        def send_control(message):
            sent = laptop()
            clock.seconds += .003
            bridge.control(message, dds(sent, clock.wall(), CONTROL_GID))

        def send_velocity(value, stamp):
            sent = laptop()
            clock.seconds += .003
            components = (value.linear.x, 0, 0, 0, 0, value.angular.z)
            bridge.command(components, stamp, dds(sent, clock.wall(), COMMAND_GID))

        adapter = CommandAdapter(bridge.session, source, send_velocity, send_control,
                                 now=clock.now, wall_ns=laptop, activation=ACTIVATION)
        adapter.observe((RAW_GID,), (STATUS_GID,))
        bridge.observe_publishers((COMMAND_GID,), (CONTROL_GID,))
        clock.seconds += .27
        adapter.observe((RAW_GID,), (STATUS_GID,))

        def status():
            t1 = bridge.issue_challenge()
            payload = dict(bridge.status(), clock_probe_ns=t1)
            sent = self.clock.wall()
            clock.seconds += .003
            adapter.status(payload, dds(sent, laptop(), STATUS_GID), (STATUS_GID,))

        status()
        self.assertEqual(bridge.activation, ACTIVATION)
        status()
        self.assertTrue(adapter.bridge_allows_commands)
        self.assertFalse(self.v_frames())
        clock.seconds += .03
        received = laptop()
        adapter.sample(twist(), dds(received - 1_000_000, received, RAW_GID))
        self.tick()
        self.assertEqual(bridge.state, Transition.STARTING)
        self.tick()
        self.assertEqual(bridge.state, State.RUNNING)
        self.assertTrue(bridge.status()['firmware_run_confirmed'])
        self.assertGreater(bridge.status()['source_age_s'], bridge.status()['adapter_receipt_age_s'])
        if source == 'teleop':
            clock.seconds += DEADMAN_S
            adapter.tick()
            self.assertEqual(bridge.state, State.FAULT_LATCHED)
            self.assertFalse(self.serial.running)
            return

        adapter.sample(twist(vx=0.), dds(laptop(), gid=RAW_GID))
        self.assertEqual(bridge.state, State.STOPPING)
        for _ in range(8):
            self.tick()
            status()
            adapter.tick()
        self.assertEqual(bridge.state, State.READY)
        self.assertIsNone(adapter.last_motion_source)
        self.assertIsNone(adapter.fault_reason)
        self.assertTrue(bridge.stop_verified)
        self.assertFalse(self.serial.armed)

        adapter.sample(twist(), dds(laptop(), gid=RAW_GID))
        self.tick()
        self.assertEqual(bridge.state, Transition.STARTING)
        clock.seconds += DEADMAN_S
        bridge.tick()  # Source stops without a terminal zero.
        self.assertEqual(bridge.state, State.FAULT_LATCHED)
        self.assertIn('deadman expired', bridge.fault_reason)
        self.assertTrue(bridge.stop_verified)
        self.assertFalse(self.serial.armed)
        self.assertFalse(self.serial.running)

    def test_shutdown_checks_stop_and_stationarity(self):
        for fail in (False, True):
            self.setup()
            self.authorize()
            self.motion()
            if fail:
                self.serial.override['STOP'] = None
            original = self.bridge.verify_stationary_stop
            with patch.object(self.bridge, 'verify_stationary_stop',
                              side_effect=lambda: original(lambda dt: setattr(
                                  self.clock, 'seconds', self.clock.seconds + dt))):
                self.assertEqual(self.bridge.shutdown(), not fail)
            self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
            self.assertEqual(self.serial.log[-1], 'Q')
            if fail:
                self.assertTrue(any('CUT MOTOR POWER' in text for text in self.log))


class AdapterTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.session = 'a' * 32
        self.out, self.controls = [], []
        self.adapter = CommandAdapter(self.session, 'teleop', lambda *sample: self.out.append(sample),
                                      self.controls.append, now=self.clock.now, wall_ns=self.clock.wall,
                                      activation=ACTIVATION)
        self.adapter.observe((RAW_GID,), (STATUS_GID,))
        self.clock.seconds += .26
        self.adapter.observe((RAW_GID,), (STATUS_GID,))
        self.clock.seconds += .025

    def status(self, allowed=True):
        now = self.clock.wall()
        sample = dict(session_id=self.session, command_source='teleop', version='3.2',
                      bridge_mode='ground', state='READY', clock_probe_ns=now - 1_000_000,
                      authority_state='OWNED' if allowed else 'WAITING_FOR_OWNER',
                      clock_status='VERIFIED', adapter_activation=ACTIVATION if allowed else None)
        self.adapter.status(sample, dds(now, gid=STATUS_GID), (STATUS_GID,))

    def test_no_replay_single_emit_and_source_gap(self):
        value = twist()
        original = dds(self.clock.wall() - 1_000_000, self.clock.wall())
        self.adapter.sample(value, original)
        self.status()
        self.adapter.tick()
        self.assertFalse(self.out)
        self.clock.seconds += .02
        now = self.clock.wall()
        info = dds(now - 2_000_000, now - 1_000_000, RAW_GID)
        self.adapter.sample(value, info)
        self.adapter.sample(value, info)
        self.adapter.tick()
        self.assertEqual(len(self.out), 1)
        self.assertEqual(self.out[0][1].source_ns, info['source_timestamp'])
        self.assertEqual(self.out[0][1].receipt_ns, info['received_timestamp'])
        self.clock.seconds += .12
        self.status()
        self.clock.seconds += DEADMAN_S + .01 - .12
        self.adapter.tick()
        self.assertIn('arrival gap', self.adapter.fault_reason)
        self.adapter.sample(value, dds(self.clock.wall()))
        self.assertEqual(len(self.out), 1)

    def test_nav2_idle_status_gap_pauses_forwarding_and_active_gap_faults(self):
        self.adapter.source = 'nav2'

        def ready(**fields):
            wall = self.clock.wall()
            packet = dict(session_id=self.session, command_source='nav2', version='3.3',
                          bridge_mode='ground', state='READY', uart_link_ready=True,
                          authority_state='OWNED', clock_status='VERIFIED',
                          adapter_activation=ACTIVATION, stationary_ready=True) | fields
            self.adapter.status(packet, dds(wall, gid=STATUS_GID), (STATUS_GID,))

        ready()
        self.clock.seconds += .383
        self.adapter.tick()
        self.assertIsNone(self.adapter.fault_reason)
        original = dds(self.clock.wall(), gid=RAW_GID)
        self.adapter.sample(twist(), original)
        ready()
        self.adapter.sample(twist(), original)
        self.assertFalse(self.out)
        self.clock.seconds += .01
        source_ns = self.clock.wall()
        self.adapter.sample(twist(), dds(source_ns, gid=RAW_GID))
        self.assertEqual(len(self.out), 1)
        self.clock.seconds += .06
        ready(stationary_source_cutoff_ns=source_ns - 1)
        self.assertEqual(self.adapter.last_motion_source, source_ns)
        self.clock.seconds += .24
        self.adapter.tick()
        self.assertEqual(self.adapter.fault_reason, 'bridge status stream lost')

    def test_nav2_stationary_check_never_forwards_velocity(self):
        self.adapter.source = 'nav2'
        self.adapter.stationary_check = True
        self.adapter.bridge_allows_commands = True
        self.adapter.sample(twist(), dds(self.clock.wall(), gid=RAW_GID))
        self.assertFalse(self.out)
        self.assertIsNone(self.adapter.last_motion_source)

    def test_fresh_input_after_missed_deadline_cannot_renew_adapter(self):
        self.status()
        self.adapter.sample(twist(), dds(self.clock.wall()))
        self.clock.seconds += .12
        self.status()
        self.clock.seconds += DEADMAN_S + .01 - .12
        self.adapter.sample(twist(), dds(self.clock.wall()))
        self.assertEqual(len(self.out), 1)
        self.assertIn('arrival gap', self.adapter.fault_reason)

    def test_missing_stale_future_or_conflicting_metadata_fails_closed(self):
        for mode in ('missing', 'stale', 'future', 'conflict', 'switch', 'invalid'):
            with self.subTest(mode=mode):
                self.setUp()
                self.status()
                wall = self.clock.wall()
                if mode == 'conflict':
                    self.adapter.observe((RAW_GID, COMMAND_GID), (STATUS_GID,))
                elif mode == 'switch':
                    self.adapter.observe((COMMAND_GID,), (STATUS_GID,))
                elif mode == 'invalid':
                    self.adapter.sample(twist(lateral=.01), dds(wall))
                else:
                    info = {'missing': {}, 'stale': dds(wall - 300_000_000, wall),
                            'future': dds(wall + 1, wall)}[mode]
                    self.adapter.sample(twist(), info)
                self.assertEqual(self.controls[-1]['kind'], 'fault')
                self.assertFalse(self.out)

    def test_four_timestamp_clock_bound(self):
        n = NS * 2
        self.assertTrue(clock_proven(n, n + 3, n + 4, n + 5))
        self.assertFalse(clock_proven(n, n + 25_000_000, n + 25_000_001, n + 5_000_000))
        self.assertFalse(clock_proven(n, n + 3, n + 1, n + 5))
        with self.assertRaises(AuthorityError):
            DDSStamp.read(dds(n + 20_000_001, n), n, remote=True)

    def test_header_round_trip(self):
        sample = CommandStamp(self.session, ACTIVATION, 'teleop', NS + 1, NS + 2)
        header = types.SimpleNamespace(frame_id=sample.frame,
                                       stamp=types.SimpleNamespace(sec=1, nanosec=2))
        self.assertEqual(CommandStamp.from_header(header), sample)


class CommissioningTest(unittest.TestCase):
    def test_single_normal_stop_and_csv(self):
        clock = Clock()
        outputs = []
        run = CommissioningRun('forward-3s', 'f' * 32, lambda *pair: outputs.append(pair), clock.now)
        status = dict(session_id=run.session, bridge_mode='ground', command_source='commissioning',
                      stationary_ready=True, authority_state='OWNED', clock_status='VERIFIED',
                      state='READY', encoder_ticks=[0, 0])
        for _ in range(90):
            run.record(status)
            run.tick()
            if run.phase == 'MOTION':
                status.update(state='RUNNING', stationary_ready=False, firmware_run_confirmed=True)
            elif run.phase == 'WAIT_STOP':
                status.update(state='READY', stationary_ready=True, stop_reason='NORMAL_ZERO',
                              encoder_ticks=[800, 810])
            clock.seconds += .05
        self.assertEqual(run.result, 'PASS')
        nonzero = [pair for pair in outputs if any(pair)]
        self.assertGreaterEqual(len(nonzero), 59)
        self.assertTrue(all(pair == (.04, 0.) for pair in nonzero))
        self.assertEqual(outputs[-1], (0., 0.))
        stream = io.StringIO()
        run.write_csv(stream)
        rows = list(csv.DictReader(io.StringIO(stream.getvalue())))
        self.assertEqual(rows[-1]['result'], 'PASS')
        self.assertIn('[800, 810]', [row['encoder_ticks'] for row in rows])
        self.assertEqual(len(PROFILES), 7)

    def test_loss_profile_sends_nothing_after_motion(self):
        clock = Clock()
        outputs = []
        run = CommissioningRun('source-loss', 'f' * 32, lambda *pair: outputs.append(pair), clock.now)
        status = dict(session_id=run.session, bridge_mode='raised', command_source='commissioning',
                      stationary_ready=True, authority_state='OWNED', clock_status='VERIFIED')
        for _ in range(50):
            run.record(status)
            run.tick()
            if run.phase == 'MOTION':
                status.update(state='RUNNING', stationary_ready=False, firmware_run_confirmed=True)
            elif run.phase == 'WAIT_FAULT':
                status.update(state='FAULT_LATCHED', authority_state='INHIBITED',
                              firmware_stationary=True, stop_verified=True,
                              fault_reason='adapter fault: selected source arrival gap',
                              firmware_running=False, firmware_armed=False,
                              pwm_duty_percent=[0, 0], encoder_continuity_valid=True, encoder_ticks=[40, 40])
            clock.seconds += .05
        self.assertEqual(run.result, 'PASS')
        self.assertEqual(outputs[-1], (.04, 0.))


class HumbleMetadataTest(unittest.TestCase):
    def test_ros_bridge_only_subscribes_to_the_dedicated_authority_topics(self):
        try:
            import rclpy
            from geometry_msgs.msg import TwistStamped
            from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
            from std_msgs.msg import String
        except ImportError as exc:
            self.skipTest(str(exc))
        for mode in (None, 'raised'):
            serial = Firmware31(Clock())

            def spin(executor):
                node, = executor.get_nodes()
                subscriptions = {sub.topic_name: sub.msg_type for sub in node.subscriptions}
                self.assertNotIn('/cmd_vel', subscriptions)
                self.assertFalse(any(topic.startswith('/amr/') for topic in subscriptions))
                self.assertEqual(subscriptions.get('/amr_bridge/ground_cmd_vel'),
                                 TwistStamped if mode else None)
                self.assertEqual(subscriptions.get('/amr_bridge/ground_control'),
                                 String if mode else None)
                raise ExternalShutdownException

            with (patch.dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='177'),
                  patch('amr_ros2_bridge_v3_2.exclusive_uart',
                        return_value=contextlib.nullcontext(Uart(serial))),
                  patch.object(SingleThreadedExecutor, 'spin', spin),
                  contextlib.redirect_stdout(io.StringIO()),
                  contextlib.redirect_stderr(io.StringIO())):
                self.assertEqual(run_ros('mock-uart', mode, 'teleop' if mode else None,
                                         None, Limits()), 0)
            self.assertFalse(any(line.startswith('ARM') for line in serial.log))
            self.assertEqual(serial.log[-1], 'Q')

    def test_real_humble_dds_metadata_on_isolated_non_motion_topic(self):
        try:
            import rclpy
            from std_msgs.msg import String
        except ImportError as exc:
            self.skipTest(str(exc))
        with patch.dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='177'):
            rclpy.init(args=[])
            node = rclpy.create_node('amr_offline_metadata_regression')
            executor = metadata_executor()
            values = []
            topic = '/amr_offline_metadata_regression'
            executor.subscribe(node, String, topic, lambda msg, info: values.append((msg.data, info)),
                               command_qos())
            publisher = node.create_publisher(String, topic, command_qos())
            executor.add_node(node)
            try:
                for _ in range(100):
                    publisher.publish(String(data='isolated DDS metadata'))
                    executor.spin_once(timeout_sec=.01)
                    if values:
                        break
                self.assertTrue(values)
                self.assertEqual(values[0][0], 'isolated DDS metadata')
                self.assertGreater(DDSStamp.read(values[0][1], time.time_ns()).source, 0)
            finally:
                executor.shutdown()
                node.destroy_node()
                rclpy.try_shutdown()


if __name__ == '__main__':
    unittest.main()
