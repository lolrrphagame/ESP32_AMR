"""Pi-local ground authority, remote DDS origin and independent watchdogs."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from amr_ground_protocol import AuthorityError, CLOCK_OFFSET_NS, CommandStamp, NS
from amr_nav2_peer import PeerSocket, PiIngress, public_domain
from amr_ros2_bridge_v3 import State
import test_amr_ground_control as ground
from tools.amr_cmd_vel_adapter import CommandAdapter


class PiNav2Test(unittest.TestCase):
    gid = tuple(range(24))
    writer = bytes(gid).hex()

    def setUp(self):
        driver = self.driver = ground.GroundTest()
        driver.setup(source='nav2')
        driver.authorize()
        self.clock = driver.clock
        self.bridge = driver.bridge
        self.bridge.pi_nav2 = True
        self.adapter = CommandAdapter(self.bridge.session, 'nav2', self.forward,
                                      lambda data: self.bridge.control(data, self.info(ground.CONTROL_GID)),
                                      activation=ground.ACTIVATION, now=self.clock.now,
                                      wall_ns=self.clock.wall, remote_nav2=True)
        self.adapter.observe((self.gid,), (ground.STATUS_GID,))
        self.clock.seconds += .26
        self.adapter.observe((self.gid,), (ground.STATUS_GID,))
        self.clock.seconds += .04
        self.adapter.status(self.bridge.status(), self.info(ground.STATUS_GID), (ground.STATUS_GID,))
        self.assertTrue(self.adapter.bridge_allows_commands)
        self.ingress = PiIngress(Mock(), self.adapter)

    def info(self, gid):
        return ground.dds(self.clock.wall(), gid=gid)

    def forward(self, twist, stamp):
        sec, nanosec = divmod(stamp.receipt_ns, NS)
        header = SimpleNamespace(frame_id=stamp.frame, stamp=SimpleNamespace(sec=sec, nanosec=nanosec))
        self.bridge.command((twist.linear.x, 0., 0., 0., 0., twist.angular.z),
                            CommandStamp.from_header(header), self.info(ground.COMMAND_GID))

    def sample(self, values, stamp, until):
        self.adapter.sample(ground.twist(values[0], values[5]), {'publisher_gid': self.gid},
                            remote_stamp=stamp, health_until=until)

    def packet(self, *, offset=6_000_000, source=None, health=400_000_000):
        # Laptop is ahead of Pi. Its original receipt may legitimately follow Pi publication.
        t1 = self.clock.wall()
        self.ingress.pending['challenge'] = (t1, self.clock.now())
        self.clock.seconds += .005
        t2 = t1 + 1_000_000 + offset
        return ['cmd', 'challenge', 'instance', self.writer, t2, t2 + 1_000_000,
                t2 if source is None else source, t2 + 1, [.04, 0., 0., 0., 0., 0.], t1 + health]

    def test_local_bridge_preserves_both_remote_stamps_and_discards_reordered_motion(self):
        cmd = self.packet()
        self.ingress.accept(cmd, self.sample)
        self.assertEqual(self.bridge.last_stamp.source_ns, cmd[6])
        self.assertEqual(self.bridge.last_stamp.receipt_ns, cmd[7])
        self.assertGreater(cmd[7], self.clock.wall())
        admitted = self.bridge.command_at
        self.clock.seconds += .04
        self.ingress.accept(cmd, self.sample)
        self.ingress.accept(self.packet(source=cmd[6] - 1), self.sample)
        self.assertEqual(self.bridge.command_at, admitted)
        self.assertEqual(self.adapter.last_source, cmd[6])

    def test_delayed_original_is_rejected_with_fresh_transport_proof(self):
        cmd = self.packet(source=self.clock.wall() - 214_000_000)
        with self.assertRaisesRegex(AuthorityError, 'stale remote'):
            self.ingress.accept(cmd, self.sample)
        self.assertIsNone(self.bridge.command_at)

    def test_invalid_clock_bounds_nonce_and_expiry_never_enable_motion(self):
        self.ingress.accept(self.packet(offset=35_000_000), self.sample)
        self.assertIsNone(self.ingress.proof_at)
        cmd = self.packet()
        cmd[1] = 'unissued'
        self.ingress.accept(cmd, self.sample)
        self.assertIsNone(self.bridge.command_at)
        ready = ['ready', *self.packet()[1:6]]
        self.ingress.accept(ready, self.sample)
        self.assertIsNone(self.bridge.command_at)
        self.clock.seconds += 1.
        with self.assertRaisesRegex(AuthorityError, 'clock proof expired'):
            self.ingress.poll(self.sample)

    def test_clock_step_and_writer_replacement_fault(self):
        self.ingress.accept(['ready', *self.packet()[1:6]], self.sample)
        cmd = self.packet()
        cmd[3] = 'bb' * 24
        with self.assertRaisesRegex(AuthorityError, 'owner changed'):
            self.ingress.accept(cmd, self.sample)
        with patch.object(self.adapter.clock, 'wall_ns', return_value=self.clock.wall() + 2 * CLOCK_OFFSET_NS):
            with self.assertRaisesRegex(AuthorityError, 'system clock changed'):
                self.ingress.accept(self.packet(), self.sample)
        self.assertIsNone(self.bridge.command_at)

    def test_remote_silence_stops_and_recovered_packets_cannot_rearm(self):
        self.ingress.accept(self.packet(), self.sample)
        self.driver.tick()
        self.driver.tick()
        self.assertEqual(self.bridge.state, State.RUNNING)
        self.driver.tick(.115)
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertTrue(self.bridge.stop_verified)
        self.ingress.accept(self.packet(), self.sample)
        self.driver.tick()
        self.assertEqual(self.driver.serial.log.count('ARM_CONT'), 1)
        self.assertFalse(self.driver.serial.armed)

    def test_feedback_expiry_can_cut_motion_shorter_than_command_deadline(self):
        self.ingress.accept(self.packet(health=115_000_000), self.sample)
        self.driver.tick(.055)
        self.assertTrue(self.driver.serial.armed)
        self.driver.tick(.045)
        self.assertTrue(self.bridge.stop_verified)
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        with self.assertRaisesRegex(AuthorityError, 'stale scan'):
            self.ingress.accept(self.packet(health=4_000_000), self.sample)

    def test_peer_rejects_datagram_from_another_session(self):
        def make_peer(session):
            with patch('socket.socket') as factory:
                peer = PeerSocket(session, '127.0.0.1', pi=True)
                self.assertEqual(factory.return_value.connect.call_args.args, (('127.0.0.1', 46010),))
                return peer
        peer = make_peer(self.bridge.session)
        wrong = make_peer('ff' * 16)
        peer.send(['challenge'])
        wrong.send(['other'])
        peer.sock.recv.side_effect = [wrong.sock.send.call_args.args[0],
                                      peer.sock.send.call_args.args[0], BlockingIOError()]
        self.assertEqual(list(peer.receive()), [['challenge']])

    def test_private_domain_separates_authority_from_public_telemetry(self):
        with patch.dict('os.environ', ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='43', AMR_NAV2_PUBLIC_DOMAIN='42'):
            self.assertEqual(public_domain(), 42)
            with patch.dict('os.environ', ROS_LOCALHOST_ONLY='0'), self.assertRaises(AuthorityError):
                public_domain()
            with patch.dict('os.environ', ROS_DOMAIN_ID='42'), self.assertRaises(AuthorityError):
                public_domain()


class FeedbackWindowTest(unittest.TestCase):
    def test_each_sensor_bounds_motion_using_measurement_time(self):
        from tools.amr_nav2_source import FeedbackWindow
        wall = 100 * NS
        def header(ns):
            sec, nanosec = divmod(ns, NS)
            return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))
        for oldest in ('scan', 'odom', 'localization'):
            with self.subTest(oldest=oldest), patch('time.time_ns', return_value=wall):
                window = FeedbackWindow(.5)
                for key in window.stamps:
                    offset = 500_000_000 if key == 'localization' else 0
                    stamp = wall - (310_000_000 if key == oldest else 0)
                    window.record(key, header(stamp + offset), offset)
                self.assertLess(window.valid_until(), wall)
                offset = 500_000_000 if oldest == 'localization' else 0
                window.record(oldest, header(wall + offset), offset)
                self.assertGreater(window.valid_until(), wall)
                window.record(oldest, header(wall + offset + 50_000_000), offset)
                self.assertLess(window.valid_until(), wall)
