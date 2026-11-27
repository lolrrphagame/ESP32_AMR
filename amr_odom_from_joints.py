
#!/usr/bin/env python3
"""Phase 4K: odometry from ROS 2 /joint_states.
READ-ONLY. Does not open UART or control motors.
"""

import math
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

RADIUS = 0.0435
SEPARATION = 0.300


class WheelOdom(Node):
    def __init__(self):
        super().__init__('amr_phase4k_wheel_odom')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev = None

        self.pub = self.create_publisher(
            Odometry, '/odom', 10
        )

        self.tf_pub = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10
        )

        self.get_logger().info(
            'Phase 4K ready: /joint_states -> /odom + TF'
        )

    def callback(self, msg):
        try:
            li = msg.name.index('left_wheel_joint')
            ri = msg.name.index('right_wheel_joint')
            left = float(msg.position[li])
            right = float(msg.position[ri])
        except (ValueError, IndexError, TypeError):
            self.get_logger().warning(
                'Invalid wheel joint data'
            )
            return

        if not (math.isfinite(left) and math.isfinite(right)):
            return

        # Carry the encoder measurement time through odom and TF, including
        # when DDS delivers an in-flight joint sample late.
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9

        if self.prev is None:
            self.prev = (left, right, t)
            self.publish(stamp, 0.0, 0.0)
            return

        pl, pr, pt = self.prev
        dt = t - pt

        dl_rad = left - pl
        dr_rad = right - pr

        if dt <= 0.0:
            return
        self.prev = (left, right, t)

        # Reject stale samples or unexpected encoder jumps.
        if (dt > 1.0
                or abs(dl_rad) > 2.0
                or abs(dr_rad) > 2.0):
            self.get_logger().warning(
                'Sample gap/reset: re-baselining encoders'
            )
            self.publish(stamp, 0.0, 0.0)
            return

        dl = dl_rad * RADIUS
        dr = dr_rad * RADIUS

        ds = (dl + dr) / 2.0
        dtheta = (dr - dl) / SEPARATION

        mid_heading = self.yaw + dtheta / 2.0

        self.x += ds * math.cos(mid_heading)
        self.y += ds * math.sin(mid_heading)

        self.yaw += dtheta
        self.yaw = math.atan2(
            math.sin(self.yaw),
            math.cos(self.yaw)
        )

        self.publish(stamp, ds / dt, dtheta / dt)

    def publish(self, stamp, linear, angular):
        z = math.sin(self.yaw / 2.0)
        w = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w

        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular

        # Provisional covariance, not calibrated.
        for i, v in enumerate(
            [0.05, 0.05, 1e6, 1e6, 1e6, 0.2]
        ):
            odom.pose.covariance[i * 6 + i] = v

        for i, v in enumerate(
            [0.1, 0.1, 1e6, 1e6, 1e6, 0.4]
        ):
            odom.twist.covariance[i * 6 + i] = v

        self.pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = z
        tf.transform.rotation.w = w

        self.tf_pub.sendTransform(tf)


def main():
    rclpy.init()
    node = WheelOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
