"""Clock and telemetry regression tests with no motion or device access."""
import os
import time
import unittest
from unittest.mock import patch

from amr_ground_protocol import DEADMAN_NS, metadata_executor, status_qos
from amr_link_metrics import LinkMetrics, summarize
from amr_ros2_bridge_v3 import DEADMAN_S, State
import test_amr_ground_control as ground
from tools.amr_cmd_vel_adapter import CommandAdapter
from tools.amr_link_diagnostics import DiagnosticBridge, attach_diagnostic

SESSION = 'a' * 32
ACTIVATION = 'b' * 32
PUBLISHER = ((4,),)


class StatusTest(unittest.TestCase):
    def setUp(self):
        self.clock = ground.Clock()
        self.metrics = LinkMetrics('adapter')
        self.out = []
        self.adapter = CommandAdapter(SESSION, 'commissioning', self.no_motion, self.out.append,
                                      now=self.clock.now, wall_ns=self.clock.wall,
                                      activation=ACTIVATION, metrics=self.metrics)
        self.sequence = 0

    @staticmethod
    def no_motion(*args):
        raise AssertionError('stationary test unexpectedly forwarded motion')

    def status(self, delay=0, owned=True, probe=True, publishers=PUBLISHER, **fields):
        wall = self.clock.wall()
        rx = wall - round(delay * 1e9)
        self.sequence += 1
        payload = dict(session_id=SESSION, command_source='commissioning', version='3.2',
                       bridge_mode='raised', state='READY', status_sequence=self.sequence,
                       authority_state='OWNED' if owned else 'WAITING_FOR_OWNER',
                       clock_status='VERIFIED', adapter_activation=ACTIVATION if owned else None,
                       clock_probe_ns=rx - 1_000_000 if probe else None) | fields
        self.adapter.status(payload, ground.dds(rx - 1_000_000, rx), publishers)

    def test_periodic_status_gap_below_bound_and_without_probe(self):
        self.status()
        for dt in (.06, .19, .04, .20, .08, .22):
            self.clock.seconds += dt
            self.status(probe=False)
            self.adapter.tick()
        self.assertIsNone(self.adapter.fault_reason)
        self.assertEqual(self.metrics.counts['status_accepted'], 7)
        self.assertEqual(self.metrics.counts['clock_probe_absent'], 6)

    def test_valid_pending_message_capable_of_preventing_false_timeout(self):
        self.status()
        self.clock.seconds += .24
        self.assertTrue(self.adapter.status_overdue())
        self.status(delay=.035)
        self.adapter.tick()
        self.assertIsNone(self.adapter.fault_reason)
        self.assertAlmostEqual(self.adapter.last_status_at, self.clock.now() - .035)

    def test_v33_adapter_requires_uart_readiness_to_forward_commands(self):
        self.status(version='3.3', uart_link_ready=False)
        self.assertIsNone(self.adapter.fault_reason)
        self.assertFalse(self.adapter.bridge_allows_commands)
        self.clock.seconds += .06
        self.status(version='3.3', uart_link_ready=True)
        self.assertIsNone(self.adapter.fault_reason)
        self.assertTrue(self.adapter.bridge_allows_commands)

    def test_real_arrival_gap_cannot_be_hidden_by_callback_schedule(self):
        self.status()
        self.clock.seconds += .24
        self.status()
        self.assertEqual(self.adapter.fault_reason, 'bridge status stream lost')
        self.assertAlmostEqual(self.metrics.latest['timeout_age_s'], .24)
        self.clock.seconds += .06
        self.status()
        self.assertEqual(self.metrics.counts['status_accepted'], 1)
        self.assertFalse(self.adapter.bridge_allows_commands)

    def test_delayed_callback_behind_freshness_bound_rejected(self):
        self.status()
        self.clock.seconds += .30
        self.status(delay=.24)
        self.assertIn('stale DDS status metadata', self.adapter.fault_reason)
        self.assertEqual(self.metrics.counts['status_rejected'], 1)

    def test_invalid_session_and_missing_discovery_do_not_renew(self):
        self.status(owned=False, publishers=())
        self.assertIsNone(self.adapter.last_status_at)
        self.clock.seconds += .30
        self.status()
        before = self.adapter.last_status_at
        self.clock.seconds += .12
        self.status(session_id='c' * 32)
        self.assertEqual(self.adapter.last_status_at, before)
        self.clock.seconds += .12
        self.adapter.tick()
        self.assertEqual(self.adapter.fault_reason, 'bridge status stream lost')
        self.assertEqual(self.metrics.counts['status_reason:session_mismatch'], 1)
        self.assertEqual(self.metrics.counts['status_reason:status_publisher_unresolved'], 1)

    def test_receipt_time_does_not_replace_invalid_dds_source(self):
        self.status()
        sent = self.clock.wall() - DEADMAN_NS
        self.clock.seconds += .06
        incoming = dict(session_id=SESSION, command_source='commissioning', version='3.2',
                        bridge_mode='raised', state='READY', authority_state='OWNED',
                        clock_status='VERIFIED', adapter_activation=ACTIVATION)
        self.adapter.status(incoming, ground.dds(sent, self.clock.wall()), PUBLISHER)
        self.assertIn('stale DDS source timestamp', self.adapter.fault_reason)


class ClockTest(unittest.TestCase):
    def setUp(self):
        self.clock = ground.Clock()
        self.metrics = LinkMetrics('bridge')
        self.bridge = DiagnosticBridge(SESSION, self.metrics, monotonic=self.clock.now,
                                       wall_ns=self.clock.wall)
        self.bridge.observe_publishers(((1,),), ((2,),))
        self.clock.seconds += .26

    def reply(self, t1, delay_before_processing=0):
        t2 = t3 = t1 + 1_000_000
        self.clock.seconds += .002
        rx = self.clock.wall()
        self.clock.seconds += delay_before_processing
        message = dict(kind='clock', session=SESSION, source='commissioning',
                       activation=ACTIVATION, t1=t1, t2=t2, t3=t3)
        self.bridge.control(message, ground.dds(t3, rx))

    def test_clock_proof_before_deadline_processed_afterward(self):
        self.reply(self.bridge.issue_challenge())
        self.clock.seconds += .98
        t1 = self.bridge.issue_challenge()
        self.reply(t1, .04)
        self.bridge.tick()
        self.assertEqual(self.bridge.state, State.READY)
        self.assertAlmostEqual(self.bridge.clock_proof_age(), .04)
        self.assertEqual(self.metrics.counts['clock_success'], 2)

    def test_clock_proof_after_deadline_cannot_renew_latched_authority(self):
        self.reply(self.bridge.issue_challenge())
        self.clock.seconds += 1.0
        self.reply(self.bridge.issue_challenge())
        self.assertEqual(self.bridge.state, State.FAULT_LATCHED)
        self.assertAlmostEqual(self.metrics.latest['timeout_age_s'], 1.002)
        self.assertIsNone(self.bridge.issue_challenge())
        self.assertEqual(self.metrics.counts['clock_success'], 1)

    def test_healthy_stationary_authority_for_120_seconds(self):
        clock, bridge = self.clock, self.bridge
        start = clock.now()
        amplitude = 5_000_000

        def send_control(message):
            sent = clock.wall() + amplitude
            clock.seconds += .002
            bridge.control(message, ground.dds(sent, clock.wall()))

        adapter = CommandAdapter(SESSION, 'commissioning', StatusTest.no_motion, send_control,
                                 now=clock.now, wall_ns=lambda: clock.wall() + amplitude,
                                 activation=ACTIVATION)
        sequence = 0
        while clock.now() < start + 120:
            t1 = bridge.issue_challenge()
            sequence += 1
            status = dict(bridge.status(), clock_probe_ns=t1, status_sequence=sequence)
            sent = clock.wall()
            clock.seconds += .002
            adapter.status(status, ground.dds(sent, clock.wall() + amplitude), PUBLISHER)
            adapter.tick()
            bridge.tick()
            clock.seconds += .058
        self.assertIsNone(bridge.fault_reason)
        self.assertIsNone(adapter.fault_reason)
        self.assertTrue(adapter.bridge_allows_commands)
        self.assertGreater(self.metrics.counts['clock_success'], 500)
        self.assertEqual(DEADMAN_S, .230)

    def test_only_authorized_diagnostic_completion_can_end_healthy_test(self):
        self.reply(self.bridge.issue_challenge())
        payload = dict(kind='diagnostic_done', session=SESSION, source='commissioning',
                       activation='c' * 32)
        self.bridge.control(payload, ground.dds(self.clock.wall()))
        self.assertFalse(self.bridge.peer_complete)
        payload['activation'] = ACTIVATION
        self.bridge.control(payload, ground.dds(self.clock.wall()))
        self.assertTrue(self.bridge.peer_complete)
        self.assertIsNone(self.bridge.fault_reason)

    def test_tool_bridge_has_no_uart_or_motion_path(self):
        self.assertIsNone(self.bridge.uart)
        self.reply(self.bridge.issue_challenge())
        with self.assertRaises(AssertionError):
            self.bridge.command(None, None, None)
        with self.assertRaises(AssertionError):
            self.bridge.send_velocity(True)
        self.bridge.tick()
        self.assertEqual(self.bridge.state, State.READY)


class MetricsTest(unittest.TestCase):
    def test_missing_metadata_is_null_and_samples_are_bounded(self):
        metrics = LinkMetrics('adapter')
        metrics.received('status', {}, callback_ns=100)
        self.assertEqual(metrics.latest['status_timestamps'],
                         dict(dds_source_ns=None, dds_received_ns=None, callback_processing_ns=100))
        for i in range(5000):
            metrics.sample('example_ms', 10_000 if i == 0 else i)
        summary = summarize(metrics.snapshot())
        self.assertEqual(summary['intervals_ms']['example_ms']['samples'], 4096)
        self.assertEqual(summary['intervals_ms']['example_ms']['max'], 10_000)


class HumbleSchedulingTest(unittest.TestCase):
    def test_humble_ready_timer_precedes_status_and_bounded_take_processes_pending(self):
        try:
            import rclpy
            from std_msgs.msg import String
        except ImportError as exc:
            self.skipTest(str(exc))
        with patch.dict(os.environ, ROS_LOCALHOST_ONLY='1'):
            rclpy.init(args=[], domain_id=183)
            node = rclpy.create_node('amr_timer_order_regression')
            executor = metadata_executor()
            order = []
            topic = '/amr_isolated_timer_order_probe'
            sub = executor.subscribe(node, String, topic,
                                     lambda msg, info: order.append(msg.data), status_qos())
            pub = node.create_publisher(String, topic, status_qos(publisher=True))

            def timeout():
                order.append('timer')
                executor.take_pending(sub)
                timer.cancel()

            try:
                for _ in range(100):
                    if pub.get_subscription_count():
                        break
                    time.sleep(.01)
                self.assertEqual(pub.get_subscription_count(), 1)
                pub.publish(String(data='pending status'))
                timer = node.create_timer(.02, timeout)
                time.sleep(.05)
                executor.add_node(node)
                for _ in range(5):
                    executor.spin_once(timeout_sec=.01)
                self.assertEqual(order, ['timer', 'pending status'])
            finally:
                executor.shutdown()
                node.destroy_node()
                rclpy.try_shutdown()

    def test_diagnostic_adapter_has_no_live_topics_or_velocity_publication(self):
        try:
            import rclpy
        except ImportError as exc:
            self.skipTest(str(exc))
        with patch.dict(os.environ, ROS_LOCALHOST_ONLY='1'):
            rclpy.init(args=[], domain_id=183)
            node = rclpy.create_node('amr_no_motion_regression')
            executor = metadata_executor()
            try:
                model, _ = attach_diagnostic(node, executor, 'adapter', SESSION, LinkMetrics('adapter'),
                                          reporter=None, pause_at=None, pause_for=.35)
                topics = {entity.topic_name for entity in (*node.publishers, *node.subscriptions)}
                self.assertFalse(any(topic.startswith(('/amr_bridge/', '/amr/')) for topic in topics))
                with self.assertRaises(AssertionError):
                    model.send_velocity(None, None)
            finally:
                executor.shutdown()
                node.destroy_node()
                rclpy.try_shutdown()


if __name__ == '__main__':
    unittest.main()
