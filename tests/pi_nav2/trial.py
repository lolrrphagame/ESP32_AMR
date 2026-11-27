import json
import math
import time
import os
import signal
import subprocess
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node('pi_nav2_trial')
scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
status, odom = {}, None
status_received_at = 0.
moving_seen = False
buffer = Buffer()
listener = TransformListener(buffer, node)
action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
t0 = time.monotonic()
init_sent = False

def receive(msg):
    global status, moving_seen, status_received_at
    status_received_at = time.monotonic()
    status = json.loads(msg.data)
    moving_seen |= status.get('firmware_running') is True

def read_odom(msg):
    global odom
    odom = msg

scans_enabled = True
scan_count = 0
next_scan = 0.

def publish():
    global init_sent, scan_count, next_scan
    if odom is None or not scans_enabled:
        return
    pose = odom.pose.pose
    yaw = 2 * math.atan2(pose.orientation.z, pose.orientation.w)
    x, y = pose.position.x, pose.position.y
    scan = LaserScan()
    scan.header.frame_id = 'laser_frame'
    now = time.monotonic()
    if now < next_scan:
        return
    # 10-interval cycle averaging ~156 ms, including a 263 ms delivery gap.
    gap = (.263, .129, .146, .145, .146, .145, .146, .145, .146, .145)[scan_count % 10]
    next_scan += gap
    if next_scan <= now:
        next_scan = now + gap
    scan_count += 1
    duration = .256 if scan_count % 10 == 2 else .129
    scan.scan_time = duration
    scan.time_increment = duration / 360
    scan.header.stamp = rclpy.time.Time(nanoseconds=time.time_ns()-round(duration*1e9)).to_msg()
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = math.pi / 180
    scan.range_min, scan.range_max = 0.15, 8.
    for i in range(361):
        angle = scan.angle_min + scan.angle_increment * i + yaw
        dx, dy = math.cos(angle), math.sin(angle)
        rx = ((1.95 if dx > 0 else -1.95) - x) / dx if abs(dx) > 1e-7 else 8.
        ry = ((1.95 if dy > 0 else -1.95) - y) / dy if abs(dy) > 1e-7 else 8.
        scan.ranges.append(min(rx, ry, 8.))
    scan_pub.publish(scan)
    if not init_sent and time.monotonic() - t0 > 4. and pose_pub.get_subscription_count():
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.header.stamp = odom.header.stamp
        initial.pose.pose.orientation.w = 1.
        initial.pose.covariance[0] = initial.pose.covariance[7] = .01
        initial.pose.covariance[35] = .005
        pose_pub.publish(initial)
        init_sent = True

def spin_until(pred, seconds, *, expect_fault=False):
    due = time.monotonic() + seconds
    while time.monotonic() < due:
        rclpy.spin_once(node, timeout_sec=.05)
        ex.spin_once(timeout_sec=0)
        if not expect_fault:
            assert not status.get('fault_latched'), status
        if pred():
            return
    raise RuntimeError(status)

os.environ['ROS_LOCALHOST_ONLY'] = '1'
ctx = Context()
ctx.init(args=[], domain_id=192)
private = rclpy.create_node('test_private_monitor', context=ctx)
ex = SingleThreadedExecutor(context=ctx)
ex.add_node(private)
private.create_subscription(String, '/amr_bridge/status', receive, 10)
node.create_subscription(Odometry, '/odom', read_odom, 10)
node.create_timer(.005, publish)
spin_until(lambda: init_sent and action.server_is_ready() and status.get('stationary_ready')
           and status.get('authority_state') == 'OWNED'
           and buffer.can_transform('map', 'base_link', rclpy.time.Time()), 20)
get_state = node.create_client(GetState, '/bt_navigator/get_state')
spin_until(get_state.service_is_ready, 10)
lifecycle = None
def active():
    global lifecycle
    if lifecycle is None:
        lifecycle = get_state.call_async(GetState.Request())
    if lifecycle.done():
        if lifecycle.result().current_state.label == 'active':
            return True
        lifecycle = None
    return False

spin_until(active, 20)
mode = sys.argv[1]
if mode == 'goal':
    # Public domain, same preflight CLI and callbacks as the physical workflow.
    with subprocess.Popen([sys.executable, 'tools/amr_feedback_preflight.py', '--seconds', '3',
                           '--amcl-tolerance', '.5'], env=dict(os.environ, ROS_LOCALHOST_ONLY='0'),
                          stdout=subprocess.DEVNULL) as diag:
        spin_until(lambda: diag.poll() is not None, 16)
        assert diag.returncode == 0
if mode == 'idle':
    # Suspend the whole ROS ingress process (including native callbacks) longer
    # than proof lifetime. The separate UDP responder must stay timely.
    source_pid = int(os.environ['AMR_TEST_SOURCE_PID'])
    os.kill(source_pid, signal.SIGSTOP)
    try:
        paused_at = time.monotonic()
        spin_until(lambda: time.monotonic() - paused_at >= 1.35, 2)
    finally:
        os.kill(source_pid, signal.SIGCONT)
    first = time.monotonic()
    def stationary():
        assert status['stationary_ready'] and status['stop_verified'] and status['authority_state'] == 'OWNED'
        assert time.monotonic() - status_received_at < 1.
        assert not status['firmware_armed'] and not moving_seen
        return time.monotonic() - first >= 120.
    spin_until(stationary, 125)
    print('Owned stationary seconds', time.monotonic() - first, flush=True)
    sys.exit(0)
transform = buffer.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
goal = NavigateToPose.Goal()
goal.pose.header.frame_id = 'map'
goal.pose.header.stamp = node.get_clock().now().to_msg()
yaw = 2 * math.atan2(transform.rotation.z, transform.rotation.w)
goal.pose.pose.position.x = transform.translation.x + .28 * math.cos(yaw)
goal.pose.pose.position.y = transform.translation.y + .28 * math.sin(yaw)
goal.pose.pose.orientation = transform.rotation
sent = action.send_goal_async(goal)
spin_until(sent.done, 5)
handle = sent.result()
assert handle.accepted
result = handle.get_result_async()
if mode == 'scan-loss':
    spin_until(lambda: moving_seen, 10)
    scans_enabled = False
    spin_until(lambda: status.get('fault_latched') and status.get('stop_verified')
               and not status['firmware_armed'] and status['pwm_duty_percent'] == [0., 0.], 2, expect_fault=True)
    print('Verified stationary STOP after scan loss:', status['fault_reason'], flush=True)
    sys.exit(0)
if mode == 'loss':
    spin_until(lambda: moving_seen, 10)
    source_pid = int(os.environ['AMR_TEST_SOURCE_PID'])
    os.kill(source_pid, signal.SIGSTOP)
    def stopped():
        return (status.get('fault_latched') and status.get('stop_verified')
                and not status['firmware_armed'] and status['pwm_duty_percent'] == [0., 0.])
    try:
        spin_until(stopped, 2, expect_fault=True)
        assert 'deadman' in status['fault_reason'], status
    finally:
        os.kill(source_pid, signal.SIGCONT)
    first = time.monotonic()
    def stays_stopped():
        assert stopped(), status
        return time.monotonic() - first >= 1.0
    spin_until(stays_stopped, 2, expect_fault=True)
    print('Verified STOP after remote loss and stayed disarmed after source resumed', flush=True)
    sys.exit(0)
spin_until(result.done, 24)
print('NavigateToPose result', result.result().status, 'motion observed', moving_seen, flush=True)
assert result.result().status == GoalStatus.STATUS_SUCCEEDED and moving_seen
spin_until(lambda: status.get('stationary_ready') and status['stop_verified'], 3)
until = time.monotonic() + .5
spin_until(lambda: time.monotonic() >= until, 2)
assert not status['firmware_armed'] and status['pwm_duty_percent'] == [0., 0.]
print('Verified settled STOP after goal', status['state'], status['firmware_armed'],
      status['pwm_duty_percent'], flush=True)
