"""Regression tests for the single-slot handoff and process isolation."""

import ctypes
from functools import partial
import multiprocessing
import time
import unittest
from unittest.mock import patch

from amr_joint_telemetry import JointSampleSlot, JointTelemetry
from amr_ros2_bridge_v3 import State
import test_amr_ground_control as ground


def block_publication(slot, diagnostics, *, duration_us, ready, publishing, finished):
    ready.set()
    while slot.fresh_after(float('-inf')) is None:
        time.sleep(.005)
    publishing.set()
    # PyDLL keeps the GIL held across the native call, as in rclpy publish.
    ctypes.PyDLL(None).usleep(duration_us)
    finished.set()


class JointTelemetryTest(unittest.TestCase):
    def test_slot_keeps_only_newest_original_stamp_and_expires_old_feedback(self):
        slot = JointSampleSlot(multiprocessing.get_context('spawn'))
        now = time.monotonic()
        self.assertTrue(slot.submit(11, now - .030, (4, 5), (6., 7.)))
        self.assertTrue(slot.submit(23, now, (10, 11), (8., 9.)))
        self.assertEqual(slot.fresh_after(now - .1), (23, now, 10, 11, 8., 9.))
        self.assertIsNone(slot.fresh_after(now))
        self.assertTrue(slot.submit(23, now - .30, (10, 11), (8., 9.)))
        self.assertIsNone(slot.fresh_after(float('-inf')))

        # Contention across processes must skip the handoff immediately.
        with slot.data.get_lock():
            self.assertFalse(slot.submit(30, now, (1, 2), (3., 4.)))

    def test_stalled_publication_does_not_block_admission_watchdogs_or_stop(self):
        context = multiprocessing.get_context('spawn')
        for duration_us, lose_clock in ((450_000, False), (1_200_000, True)):
            with self.subTest(duration_us=duration_us):
                ready, publishing, finished = context.Event(), context.Event(), context.Event()
                target = partial(block_publication, duration_us=duration_us, ready=ready,
                                 publishing=publishing, finished=finished)
                with patch('amr_joint_telemetry.run_publisher', target):
                    telemetry = JointTelemetry(False)
                try:
                    self.assertTrue(ready.wait(5))
                    self.assertTrue(telemetry.slot.submit(time.time_ns(), time.monotonic(),
                                                          (0, 0), (0., 0.)))
                    self.assertTrue(publishing.wait(5))
                    driver = ground.GroundTest()
                    driver.setup(source='nav2')
                    driver.authorize()
                    if lose_clock:
                        # Link loss while disarmed still runs the independent proof watchdog.
                        driver.tick(dt=1.01)
                        self.assertIn('clock proof', driver.bridge.fault_reason)
                    else:
                        driver.motion()
                        driver.tick(dt=.24)
                        self.assertIn('deadman expired', driver.bridge.fault_reason)
                    self.assertEqual(driver.bridge.state, State.FAULT_LATCHED)
                    self.assertTrue(driver.bridge.stop_verified)
                    self.assertFalse(driver.serial.armed)
                    self.assertFalse(driver.serial.running)
                    arm_count = driver.serial.log.count('ARM_CONT')
                    driver.command()
                    driver.tick()
                    self.assertEqual(driver.serial.log.count('ARM_CONT'), arm_count)
                    self.assertFalse(finished.is_set(), 'control must finish while the publisher is blocked')
                    self.assertTrue(finished.wait(3))
                finally:
                    telemetry.close()
