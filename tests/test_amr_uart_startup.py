"""Offline V3.3 regression tests using a byte-level fake UART and fake clock."""
from collections import Counter
import unittest

from amr_ros2_bridge_v3 import ProtocolError, State, Uart, ZERO
from amr_ros2_bridge_v3_2 import BridgeV32
from amr_ros2_bridge_v3_3 import (
    CAPS, LOW_CAPS, STARTUP_TIMEOUT_S, UART_EVENTS, VERSION, VERSION_READ_ONLY, BridgeV33, UartDiagnostics,
)
import test_amr_ground_control as ground
from test_amr_ros2_bridge_v3 import Clock
from test_amr_ros2_bridge_v3_1 import Firmware31


class Firmware33(Firmware31):
    def __init__(self, clock):
        super().__init__(clock)
        self.capabilities = LOW_CAPS
        self.link = 'WAIT_LINK'
        self.established = False
        self.startup = Counter()
        self.operational = Counter()
        self.last_event = 'NONE'
        self.last_us = 0
        self.attempts = 0
        self.challenge = None
        self.acknowledge_stop = True
        self.interrupt = False
        self.continuous_noise = False
        self.replay_begin = False
        self.replay_proof = False
        self.corrupt_diag = False

    def trip(self, reason):
        super().trip(reason)
        self.link = 'FAULT_LATCHED'

    def uart_error(self, event, reason='UART_HW'):
        counts = self.operational if self.established else self.startup
        counts[event] += 1
        self.last_event = event
        self.last_us = round(self.clock.now() * 1_000_000)
        self.challenge = None
        self.armed = self.running = False
        self.speed = ZERO
        if self.established:
            self.fault = self.fault or reason
            self.ue += 1
            self.link = 'FAULT_LATCHED'
        elif not self.fault:
            self.link = 'WAIT_LINK'

    def expire(self):
        super().expire()
        if self.link == 'HANDSHAKE' and self.clock.now() - self.begin_at >= 2:
            self.link = 'WAIT_LINK'
            self.challenge = None

    def reply(self, line):
        if line == 'VER':
            return VERSION
        if line == 'Q':
            return (super().reply(line).replace('PV=3.1 ', 'PV=3.3 ')
                    + f' LS={self.link} SU={self.startup.total()}')
        if line == 'UART_DIAG':
            vectors = ['-1' if self.corrupt_diag else ','.join(str(group[event]) for event in UART_EVENTS)
                       for group in (self.startup, self.operational)]
            return (f'UART_DIAG BOOT={self.boot} LS={self.link} READY={int(self.link == "LINK_READY")}'
                    f' SU={self.startup.total()} UE={self.ue} LAST={self.last_event} US={self.last_us}'
                    f' S={vectors[0]} O={vectors[1]}')
        if line == 'STOP':
            response = super().reply(line)
            if self.link == 'HANDSHAKE':
                self.stopped = self.acknowledge_stop
                self.proofs = 0
            return response if self.acknowledge_stop else None
        if line.startswith('LINK_BEGIN '):
            if self.fault:
                return 'ERR FAULT'
            super().reply('STOP')
            self.link = 'HANDSHAKE'
            self.begin_at = self.clock.now()
            self.attempts += 1
            self.challenge = f'{self.attempts:016x}'
            self.stopped = False
            self.proofs = 0
            client = '0' * 16 if self.replay_begin else line.split()[1]
            return f'LINK_BEGIN {self.boot} {client} {self.challenge}'
        if line.startswith(('LINK_STEP ', 'LINK_COMMIT ')):
            command, boot, token, *steps = line.split()
            if self.fault:
                return 'ERR FAULT'
            if (self.link != 'HANDSHAKE' or boot != self.boot or token != self.challenge
                    or not self.stopped):
                return 'ERR LINK_PROOF'
            if command == 'LINK_STEP':
                if int(steps[0]) != self.proofs + 1:
                    return 'ERR LINK_PROOF'
                if self.continuous_noise or (self.interrupt and self.attempts == 1 and self.proofs == 2):
                    self.uart_error('UART_FRAME_ERR')
                    return None
                self.proofs += 1
                if self.replay_proof:
                    return f'OK LINK_STEP {self.boot} {"0" * 16} {self.proofs}'
            elif (self.proofs < 5 or self.clock.now() - self.begin_at < 0.5
                  or not self.still or self.armed or self.running):
                return 'ERR LINK_PROOF'
            else:
                self.link = 'LINK_READY'
                self.established = True
                self.challenge = None
            return 'OK ' + line
        if line in ('ARM', 'ARM_CONT') or line.startswith(('ARM_LOW ', 'V ')):
            if self.link != 'LINK_READY':
                return 'ERR FAULT' if self.fault else 'ERR LINK_NOT_READY'
        return super().reply(line)


class Bridge33Test(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.serial = Firmware33(self.clock)

    def make_bridge(self, commissioning=None, bridge_class=BridgeV33):
        args = dict(monotonic=self.clock.now, wall_ns=self.clock.wall, report=lambda _: None)
        if bridge_class == BridgeV33:
            args['sleep'] = self.advance
        source = 'commissioning' if commissioning else None
        self.bridge = bridge_class(Uart(self.serial, self.clock.now), commissioning, source, **args)
        return self.bridge

    def advance(self, seconds):
        self.clock.seconds += seconds

    tick = ground.GroundTest.tick
    settle = ground.GroundTest.settle
    prove = ground.GroundTest.prove
    authorize = ground.GroundTest.authorize
    command = ground.GroundTest.command
    motion = ground.GroundTest.motion
    v_frames = ground.GroundTest.v_frames

    def assert_no_motion_commands(self):
        self.assertFalse(any(cmd.startswith(('ARM', 'V ')) for cmd in self.serial.log))

    def assert_link_ready(self):
        status = self.bridge.status()
        self.assertEqual(status['uart_startup_state'], 'LINK_READY')
        self.assertTrue(status['uart_link_ready'])
        self.assertEqual(status['uart_operational_errors'], 0)
        self.assertEqual(self.bridge.baseline, (self.serial.boot, (0, 0)))

    def test_clean_cold_boot_is_read_only_with_five_confirmed_proofs(self):
        self.make_bridge().start()
        self.assert_link_ready()
        self.assertEqual(self.bridge.state, State.READ_ONLY)
        self.assertEqual(self.bridge.status()['uart_startup_errors'], 0)
        self.assertTrue(self.bridge.stop_verified)
        self.assertEqual(sum(cmd.startswith('LINK_STEP ') for cmd in self.serial.log), 5)
        self.assertGreaterEqual(self.clock.now() - self.serial.begin_at, 0.5)
        self.assert_no_motion_commands()

    def test_break_frame_and_multiple_startup_errors_remain_visible(self):
        for event in ('UART_BREAK', 'UART_FRAME_ERR', 'UART_BREAK', 'UART_BUFFER_FULL'):
            self.serial.uart_error(event)
        self.make_bridge('raised').start()
        self.assert_link_ready()
        self.settle()
        self.assertEqual(self.bridge.status()['uart_startup_errors'], 4)
        self.assertEqual(self.bridge.status()['uart_last_error_event'], 'UART_BUFFER_FULL')
        self.assertEqual(self.bridge.uart_diag.startup_counts['UART_BREAK'], 2)
        self.assertEqual(self.bridge.uart_diag.startup_counts['UART_FRAME_ERR'], 1)
        self.assert_no_motion_commands()
        self.make_bridge().start()
        self.assert_link_ready()
        self.assertEqual(self.bridge.status()['uart_startup_errors'], 4)

    def test_interrupted_handshake_requires_new_challenge(self):
        self.serial.interrupt = True
        self.make_bridge().start()
        self.assert_link_ready()
        self.assertEqual(self.serial.attempts, 2)
        self.assertEqual(self.bridge.status()['uart_startup_errors'], 1)
        proof_tokens = {cmd.split()[2] for cmd in self.serial.log if cmd.startswith('LINK_STEP ')}
        self.assertEqual(len(proof_tokens), 2)

    def test_continuous_noise_leaves_motor_disabled_and_attempts_stop(self):
        self.serial.continuous_noise = True
        start = self.clock.now()
        self.make_bridge('ground').start()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertIsNone(self.bridge.baseline)
        self.assertFalse(self.bridge.status()['uart_link_ready'])
        self.assertFalse(self.serial.armed or self.serial.running)
        self.assertLess(self.clock.now() - start, STARTUP_TIMEOUT_S + 0.5)
        self.assertGreater(self.serial.attempts, 1)
        self.assertGreater(self.bridge.status()['uart_startup_errors'], 1)
        self.assertIsNone(self.bridge.issue_challenge())
        self.assert_no_motion_commands()

    def test_firmware_refuses_motion_before_ready_and_stop_is_always_available(self):
        uart = Uart(self.serial, self.clock.now)
        for state in ('WAIT_LINK', 'HANDSHAKE', 'LINK_READY', 'FAULT_LATCHED'):
            with self.subTest(state=state):
                self.serial.link = state
                if state == 'HANDSHAKE':
                    self.serial.begin_at = self.clock.now()
                if state == 'FAULT_LATCHED':
                    self.serial.trip('ENCODER')
                if state != 'LINK_READY':
                    error = 'ERR FAULT' if state == 'FAULT_LATCHED' else 'ERR LINK_NOT_READY'
                    for cmd in ('ARM', 'ARM_CONT', 'ARM_LOW 3 WHEELS_UP', 'V 5 5'):
                        uart.exchange(cmd, error)
                else:
                    uart.exchange('ARM_CONT', 'OK ARMED CONT')
                    uart.exchange('V 5 5', 'OK V')
                uart.exchange('STOP', 'OK STOP')
                self.assertFalse(self.serial.armed or self.serial.running)
                self.assertEqual(self.serial.link, state)

    def test_replayed_client_nonce_or_proof_cannot_establish_baseline(self):
        for flag in ('replay_begin', 'replay_proof'):
            with self.subTest(flag=flag):
                self.serial = Firmware33(self.clock)
                setattr(self.serial, flag, True)
                self.make_bridge('raised').start()
                self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
                self.assertIsNone(self.bridge.baseline)
                self.assertFalse(any(cmd.startswith('LINK_COMMIT ') for cmd in self.serial.log))
                self.assert_no_motion_commands()

    def test_missing_stop_or_stationary_evidence_fails_closed(self):
        for flag in ('acknowledge_stop', 'still'):
            with self.subTest(flag=flag):
                self.serial = Firmware33(self.clock)
                setattr(self.serial, flag, False)
                self.make_bridge('ground').start()
                self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
                self.assertIsNone(self.bridge.baseline)
                self.assertEqual(self.serial.link, 'WAIT_LINK' if flag == 'acknowledge_stop' else 'HANDSHAKE')
                self.assert_no_motion_commands()

    def test_operational_uart_error_while_idle_cannot_be_cleared_by_reconnect(self):
        self.make_bridge().start()
        baseline = self.bridge.baseline
        self.serial.uart_error('UART_BREAK')
        self.tick()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertEqual(self.bridge.baseline, baseline)
        self.assertEqual(self.bridge.status()['uart_operational_errors'], 1)
        self.assertEqual(self.bridge.status()['uart_last_error_event'], 'UART_BREAK')
        self.bridge.start()
        self.assertEqual(self.bridge.baseline, baseline)
        self.make_bridge('ground').start()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertIsNone(self.bridge.baseline)
        self.assertEqual(self.serial.fault, 'UART_HW')
        self.assert_no_motion_commands()

    def test_operational_error_while_running_inhibits_dds_and_sends_stop(self):
        self.make_bridge('ground').start()
        self.settle()
        self.authorize()
        self.motion()
        self.serial.uart_error('UART_FRAME_ERR')
        self.assertFalse(self.serial.running)
        self.tick()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertTrue(self.bridge.stop_verified)
        self.assertEqual(self.bridge.status()['authority_state'], 'INHIBITED')
        self.assertEqual(self.bridge.status()['firmware_fault_reason'], 'UART_HW')
        self.assertIn('STOP', self.serial.log[-4:])
        sent = self.v_frames()
        self.command()
        self.tick()
        self.assertEqual(self.v_frames(), sent)

    def test_firmware_reset_while_running_invalidates_encoder_and_control(self):
        self.make_bridge('raised').start()
        self.settle()
        self.authorize()
        self.motion()
        sent = self.v_frames()
        self.serial.__init__(self.clock)
        self.serial.boot = '11223344'
        self.tick()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertFalse(self.bridge.encoder_valid)
        self.assertFalse(self.bridge.status()['uart_link_ready'])
        self.assertEqual(self.bridge.status()['authority_state'], 'INHIBITED')
        self.assertEqual(self.bridge.baseline[0], 'aaccee01')
        self.command()
        self.tick()
        self.assertFalse(self.v_frames())
        self.assertTrue(sent)
        self.assertFalse(any(cmd.startswith('LINK_BEGIN ') for cmd in self.serial.log))

    def test_old_firmware_or_missing_safe_startup_capability_rejected(self):
        for firmware, capabilities in ((Firmware31, None), (Firmware33, 'CAPS PV=3.3 CAP=UART_DIAG'),
                                       (Firmware33, CAPS + ',UNKNOWN')):
            with self.subTest(firmware=firmware, capabilities=capabilities):
                self.serial = firmware(self.clock)
                if capabilities:
                    self.serial.capabilities = capabilities
                self.make_bridge('ground').start()
                self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
                self.assertIsNone(self.bridge.baseline)
                self.assert_no_motion_commands()

    def test_read_only_firmware_allows_diagnostics_and_refuses_commissioning(self):
        self.serial.override['VER'] = VERSION_READ_ONLY
        self.serial.capabilities = CAPS
        self.make_bridge().start()
        self.assert_link_ready()
        self.make_bridge('raised').start()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertIsNone(self.bridge.baseline)
        self.assert_no_motion_commands()

    def test_legacy_bridge_cannot_negotiate_new_firmware_or_arm(self):
        self.make_bridge('raised', BridgeV32).start()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertEqual(self.serial.link, 'WAIT_LINK')
        self.assert_no_motion_commands()

    def test_diagnostic_counter_validation_and_large_reply(self):
        self.serial.startup.update({event: 4294967295 for event in UART_EVENTS})
        self.serial.operational.update({event: 4294967295 for event in UART_EVENTS})
        self.serial.ue = 4294967295
        self.serial.last_event = 'UART_BREAK'
        self.serial.link = 'FAULT_LATCHED'
        reply = self.serial.reply('UART_DIAG').replace(f'SU={self.serial.startup.total()}', 'SU=4294967295')
        self.assertGreater(len(reply), 256)
        self.serial.override['UART_DIAG'] = reply
        bridge = self.make_bridge()
        diag = bridge.read_uart_diag()
        self.assertEqual(diag.startup_errors, 4294967295)
        for bad in (reply + ' UE=1', reply.replace('READY=0', 'READY=1'),
                    reply.replace('SU=4294967295', 'SU=0'), reply.replace('LAST=UART_BREAK', 'LAST=NONE')):
            with self.subTest(bad=bad):
                with self.assertRaises(ProtocolError):
                    UartDiagnostics.parse(bad)

    def test_malformed_diag_never_seeds_baseline(self):
        self.serial.corrupt_diag = True
        self.make_bridge('raised').start()
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertIsNone(self.bridge.baseline)
        self.assert_no_motion_commands()


if __name__ == '__main__':
    unittest.main()
