#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState

class WheelJointStatePublisher(Node):
    def __init__(self):
        super().__init__('wheel_joint_state_publisher')

        # ---- CONFIG ----
        self.ticks_per_rev = 2048  # CHANGE THIS
        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint',
        ]
        self.rad_per_tick = 2.0 * math.pi / float(self.ticks_per_rev)

        # state
        self.prev_ticks = None
        self.angles = [0.0, 0.0, 0.0, 0.0]
        self.prev_time = None

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(Int32MultiArray, '/wheel_ticks', self.on_ticks, 10)

    def on_ticks(self, msg: Int32MultiArray):
        now = self.get_clock().now()

        ticks = list(msg.data)
        if len(ticks) != 4:
            self.get_logger().warn(f"Expected 4 ticks, got {len(ticks)}")
            return

        if self.prev_ticks is None:
            self.prev_ticks = ticks
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-6

        d_ticks_raw = [t - p for t, p in zip(ticks, self.prev_ticks)]

        # Right wheels are joints 1 and 3 in your list -> flip sign
        sign = [1, -1, 1, -1]
        d_ticks = [s * d for s, d in zip(sign, d_ticks_raw)]

        d_theta = [d * self.rad_per_tick for d in d_ticks]

        # integrate angle
        self.angles = [a + d for a, d in zip(self.angles, d_theta)]

        # velocity (rad/s)
        vels = [d / dt for d in d_theta]

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self.joint_names
        js.position = self.angles
        js.velocity = vels
        self.pub.publish(js)

        self.prev_ticks = ticks
        self.prev_time = now

def main():
    rclpy.init()
    node = WheelJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
