#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_odom')

        # ---- CONFIG ----
        self.wheel_radius = 0.1575  # meters (from your URDF)
        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint',
        ]

        # State
        self.prev_positions = None
        self.prev_time = None
        self.x = 0.0  # integrated forward position (meters)

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.on_joint_states,
            10
        )

        self.pub = self.create_publisher(Odometry, '/wheel/odom', 10)

    def on_joint_states(self, msg: JointState):
        now = self.get_clock().now()

        # Build joint name → position map
        joint_pos = dict(zip(msg.name, msg.position))

        try:
            positions = [joint_pos[n] for n in self.joint_names]
        except KeyError:
            # joint_states not ready yet
            return

        if self.prev_positions is None:
            self.prev_positions = positions
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # Δθ for each wheel (radians)
        dtheta = [
            p - pp for p, pp in zip(positions, self.prev_positions)
        ]

        # Average wheel angular velocity (rad/s)
        avg_omega = sum(dtheta) / len(dtheta) / dt

        # Linear forward velocity (m/s)
        v = avg_omega * self.wheel_radius

        # Integrate x only
        self.x += v * dt

        # ---- Publish odometry ----
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position: x only
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0

        # Orientation: identity (yaw ignored)
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Velocity: forward only
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # Covariance: trust vx, distrust everything else
        odom.twist.covariance = [
            0.01, 0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,  999.0, 0.0,   0.0,   0.0,   0.0,
            0.0,  0.0,   999.0, 0.0,   0.0,   0.0,
            0.0,  0.0,   0.0,   999.0, 0.0,   0.0,
            0.0,  0.0,   0.0,   0.0,   999.0, 0.0,
            0.0,  0.0,   0.0,   0.0,   0.0,   999.0,
        ]

        self.pub.publish(odom)

        self.prev_positions = positions
        self.prev_time = now


def main():
    rclpy.init()
    node = WheelOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
