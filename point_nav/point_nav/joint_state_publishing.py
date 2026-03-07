#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState

class WheelJointStatePublisher(Node):
    def __init__(self):
        super().__init__('wheel_joint_state_publisher')

        self.declare_parameter('ticks_per_rev', 589824 * 4)
        self.declare_parameter(
            'joint_names',
            [
                'front_left_wheel_joint',
                'front_right_wheel_joint',
                'back_left_wheel_joint',
                'back_right_wheel_joint',
            ],
        )
        self.declare_parameter('wheel_ticks_topic', '/wheel_ticks')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('right_wheel_indices', [1, 3])

        self.joint_names = list(self.get_parameter('joint_names').value)
        ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.rad_per_tick = 2.0 * math.pi / max(1.0, ticks_per_rev)
        self.right_wheel_indices = set(int(i) for i in self.get_parameter('right_wheel_indices').value)

        # state
        self.prev_ticks = None
        self.angles = [0.0] * len(self.joint_names)
        self.prev_time = None

        ticks_topic = str(self.get_parameter('wheel_ticks_topic').value)
        joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        self.pub = self.create_publisher(JointState, joint_states_topic, 10)
        self.sub = self.create_subscription(Int32MultiArray, ticks_topic, self.on_ticks, 10)
        self.get_logger().info(
            f'Publishing {joint_states_topic} from {ticks_topic}; ticks_per_rev={ticks_per_rev:.1f}'
        )

    def on_ticks(self, msg: Int32MultiArray):
        now = self.get_clock().now()

        ticks = list(msg.data)
        if len(ticks) != len(self.joint_names):
            self.get_logger().warn(f"Expected {len(self.joint_names)} ticks, got {len(ticks)}")
            return

        if self.prev_ticks is None:
            self.prev_ticks = ticks
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-6

        d_ticks_raw = [t - p for t, p in zip(ticks, self.prev_ticks)]

        # Right side wheel encoders are often inverted relative to joint convention.
        sign = [(-1 if i in self.right_wheel_indices else 1) for i in range(len(self.joint_names))]
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
