#!/usr/bin/env python3
"""Laptop source for the Pi-local Nav2 command adapter."""
import argparse
import ctypes as ct
import math
from pathlib import Path
import secrets
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from amr_ground_protocol import (AuthorityError, CLOCK_OFFSET_NS, DDSStamp, DEADMAN_NS, NS,
                                 SOURCE_TOPICS, WallClockGuard, command_qos, token_valid)
from amr_nav2_peer import PeerSocket
from amr_ros2_bridge_v3 import wheel_targets


def valid_pose(position, rotation):
    return (all(math.isfinite(v) for v in (position.x, position.y, position.z))
            and abs(sum(v * v for v in (rotation.x, rotation.y, rotation.z, rotation.w)) - 1.) < .01)


class FeedbackWindow:
    def __init__(self, amcl_tolerance):
        self.stamps = dict(scan=0, odom=0, localization=0)
        self.amcl_offset = round(amcl_tolerance * NS)

    def record(self, key, header, offset=0):
        ns = header.stamp.sec * NS + header.stamp.nanosec - offset
        # Stamps on Pi-originated data include the possible interhost skew.
        if not 0 < ns <= time.time_ns() + CLOCK_OFFSET_NS:
            self.stamps[key] = 0
        else:
            self.stamps[key] = ns

    def scan(self, msg):
        if msg.header.frame_id == 'laser_frame' and msg.ranges and any(
                math.isfinite(r) and msg.range_min <= r <= msg.range_max for r in msg.ranges):
            self.record('scan', msg.header)
        else:
            self.stamps['scan'] = 0

    def odom(self, msg):
        pose = msg.pose.pose
        if (msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link'
                and valid_pose(pose.position, pose.orientation)):
            self.record('odom', msg.header)
        else:
            self.stamps['odom'] = 0

    def tf(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                if valid_pose(transform.transform.translation, transform.transform.rotation):
                    self.record('localization', transform.header, self.amcl_offset)
                else:
                    self.stamps['localization'] = 0

    def valid_until(self):
        # RPP transform_tolerance is 0.3 s. Account for Pi-originated stamps.
        return min(self.stamps.values()) + 300_000_000 - CLOCK_OFFSET_NS


def run(session, pi, library, tolerance):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.utilities import get_rmw_implementation_identifier

    if get_rmw_implementation_identifier() != 'rmw_fastrtps_cpp':
        raise AuthorityError('Nav2 writer host verification requires rmw_fastrtps_cpp')

    take = ct.CDLL(str(library.resolve())).amr_take_twist
    take.argtypes = [ct.c_void_p, ct.POINTER(ct.c_double), ct.POINTER(ct.c_int64), ct.POINTER(ct.c_uint8)]
    take.restype = ct.c_int
    link = PeerSocket(session, pi, pi=False)
    clock = WallClockGuard()
    rclpy.init(args=[])
    node = rclpy.create_node('amr_nav2_source')
    # A separate unspun node lets rcl_take consume the Twist AND its rmw_message_info.
    reader = rclpy.create_node('amr_nav2_writer_reader')
    topic = SOURCE_TOPICS['nav2']
    sub = reader.create_subscription(Twist, topic, lambda _: None, command_qos())
    values, stamps, writer = (ct.c_double * 6)(), (ct.c_int64 * 2)(), (ct.c_uint8 * 24)()
    health = FeedbackWindow(tolerance)
    node.create_subscription(LaserScan, '/scan', health.scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', health.odom, qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf', health.tf, qos_profile_sensor_data)
    instance = secrets.token_hex(16)
    local_prefix = None
    owner = challenge = fault = None
    last_source = next_graph = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.002)
            try:
                clock.check()
            except AuthorityError as exc:
                fault = str(exc)
            for (nonce,) in link.receive():
                challenge = nonce, time.time_ns(), time.monotonic()
                if fault:
                    link.send(['fault', nonce, instance, owner, 0, 0, fault])
                elif owner:
                    link.send(['ready', nonce, instance, owner, challenge[1], time.time_ns()])
            try:
                with sub.handle:
                    rc = take(sub.handle.pointer, values, stamps, writer)
                if rc < 0:
                    raise AuthorityError('failed to take DDS Twist with metadata')
                now = time.monotonic()
                if rc or now >= next_graph:
                    next_graph = now + .04
                    if local_prefix is None:
                        local = [ep for ep in reader.get_subscriptions_info_by_topic(topic)
                                 if ep.node_name == reader.get_name() and ep.node_namespace == reader.get_namespace()]
                        if len(local) > 1:
                            raise AuthorityError('multiple Nav2 source readers')
                        if not local:
                            continue
                        # Fast DDS GUID_t::is_on_same_host_as compares the first four bytes.
                        local_prefix = bytes(local[0].endpoint_gid)[:4]
                    endpoints = reader.get_publishers_info_by_topic(topic)
                    if len(endpoints) > 1 or (owner and not endpoints):
                        raise AuthorityError('Nav2 publisher conflict or loss')
                    if endpoints:
                        ep = endpoints[0]
                        gid = bytes(ep.endpoint_gid).hex()
                        if (ep.node_namespace, ep.node_name) == ('/', 'controller_server'):
                            if bytes(ep.endpoint_gid)[:4] != local_prefix:
                                raise AuthorityError('Nav2 writer is outside the verified laptop clock domain')
                            if owner and gid != owner:
                                raise AuthorityError('controller_server writer GID changed')
                            owner = gid
                        elif owner:
                            raise AuthorityError(f'unexpected Nav2 publisher {ep.node_namespace}/{ep.node_name}')
                if not rc or fault or not owner or not challenge:
                    continue
                if bytes(writer).hex() != owner:
                    raise AuthorityError('Nav2 DDS sample writer mismatch')
                wall = clock.check()
                stamp = DDSStamp.read(dict(source_timestamp=stamps[0], received_timestamp=stamps[1]), wall)
                if stamp.source <= last_source:
                    continue
                last_source = stamp.source
                wheel_targets(values)
                nonce, t2, received_at = challenge
                if time.monotonic() - received_at >= DEADMAN_NS / NS:
                    continue
                until = health.valid_until()
                if any(values) and wall >= until:
                    raise AuthorityError('stale scan, odometry or localization')
                link.send(['cmd', nonce, instance, owner, t2, clock.check(),
                           stamp.source, stamp.received, list(values), until])
            except (AuthorityError, ValueError) as exc:
                if fault is None:
                    fault = str(exc)
                    print(f'Nav2 source inhibited: {fault}', file=sys.stderr, flush=True)
    except ExternalShutdownException:
        pass
    finally:
        link.sock.close()
        reader.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', required=True)
    p.add_argument('--pi', required=True)
    p.add_argument('--library', type=Path, default=Path(__file__).resolve().parents[1] / 'build/nav2_take.so')
    p.add_argument('--amcl-tolerance', type=float, default=.5)
    args = p.parse_args()
    if not token_valid(args.session) or not math.isfinite(args.amcl_tolerance) or args.amcl_tolerance < 0:
        p.error('invalid session or AMCL transform tolerance')
    try:
        run(args.session, args.pi, args.library, args.amcl_tolerance)
    except KeyboardInterrupt:
        pass
