"""Publish Nav2 wheel feedback in a process separate from motor control."""

import json
from math import pi
import multiprocessing
from struct import Struct
import sys
import time

from amr_ground_protocol import NS, telemetry_qos
from amr_link_metrics import LinkMetrics, PeriodicLinkReport, summarize
from amr_ros2_bridge_v3 import CPR, DEADMAN_S


# ROS stamp (ns), monotonic measurement time (s), ticks and measured RPM.
SAMPLE = Struct('=qdqqdd')
POLL_S = .01


class JointSampleSlot:
    def __init__(self, context):
        self.data = context.Array('B', SAMPLE.size, lock=context.Lock())

    def submit(self, stamp_ns, measured_at, ticks, rpm):
        lock = self.data.get_lock()
        if not lock.acquire(False):
            return False
        try:
            SAMPLE.pack_into(self.data.get_obj(), 0, stamp_ns, measured_at, *ticks, *rpm)
        finally:
            lock.release()
        return True

    def fresh_after(self, previous_at):
        lock = self.data.get_lock()
        if not lock.acquire(False):
            return None
        try:
            sample = SAMPLE.unpack_from(self.data.get_obj())
        finally:
            lock.release()
        stamp_ns, measured_at = sample[:2]
        if stamp_ns > 0 and measured_at > previous_at and 0 <= time.monotonic() - measured_at < DEADMAN_S:
            return sample
        return None


def run_publisher(slot, diagnostics):
    import rclpy
    from sensor_msgs.msg import JointState

    metrics = LinkMetrics('joint_telemetry') if diagnostics else None
    reporter = PeriodicLinkReport(metrics) if metrics else None
    parent = multiprocessing.parent_process()
    node = None
    rclpy.init(args=[])
    try:
        node = rclpy.create_node('amr_joint_telemetry', start_parameter_services=False,
                                 enable_rosout=False)
        pub = node.create_publisher(JointState, '/joint_states', telemetry_qos())
        previous_at = float('-inf')
        while rclpy.ok() and parent.is_alive():
            sample = slot.fresh_after(previous_at)
            if sample is not None:
                stamp_ns, previous_at, left, right, left_rpm, right_rpm = sample
                msg = JointState()
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, NS)
                msg.name = ['left_wheel_joint', 'right_wheel_joint']
                msg.position = [tick * 2 * pi / CPR for tick in (left, right)]
                msg.velocity = [rpm * 2 * pi / 60 for rpm in (left_rpm, right_rpm)]
                started = time.monotonic() if metrics else None
                pub.publish(msg)
                if metrics:
                    metrics.sample('joint_publish_ms', (time.monotonic() - started) * 1000)
                    metrics.event('joint_published')
                    reporter.poll()
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        if metrics:
            print(json.dumps(summarize(metrics.snapshot())), file=sys.stderr, flush=True)


class JointTelemetry:
    def __init__(self, diagnostics):
        context = multiprocessing.get_context('spawn')
        self.slot = JointSampleSlot(context)
        self.process = context.Process(target=run_publisher, args=(self.slot, diagnostics),
                                       name='amr-joint-telemetry', daemon=True)
        self.process.start()

    def close(self):
        if self.process.is_alive():
            self.process.terminate()
        self.process.join(.5)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(.5)
