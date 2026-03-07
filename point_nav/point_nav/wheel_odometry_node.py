import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class WheelOdomNode(Node):
    def __init__(self) -> None:
        super().__init__('wheel_odom')

        self.declare_parameter('wheel_radius', 0.155)
        self.declare_parameter(
            'joint_names',
            [
                'front_left_wheel_joint',
                'front_right_wheel_joint',
                'back_left_wheel_joint',
                'back_right_wheel_joint',
            ],
        )
        self.declare_parameter('spin_sum_threshold', 0.05)
        self.declare_parameter('spin_diff_threshold', 0.10)
        self.declare_parameter('output_topic', '/wheel/odom')

        self._prev_positions = None
        self._prev_time = None
        self._x = 0.0

        output_topic = str(self.get_parameter('output_topic').value)

        self._sub = self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self._pub = self.create_publisher(Odometry, output_topic, 10)

        self.get_logger().info(f'Wheel odometry publishing on {output_topic}')

    def _on_joint_states(self, msg: JointState) -> None:
        now = self.get_clock().now()

        joint_pos = dict(zip(msg.name, msg.position))
        joint_names = list(self.get_parameter('joint_names').value)

        try:
            positions = [float(joint_pos[name]) for name in joint_names]
        except KeyError:
            return

        if self._prev_positions is None:
            self._prev_positions = positions
            self._prev_time = now
            return

        dt = (now - self._prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        dtheta = [p - pp for p, pp in zip(positions, self._prev_positions)]
        d_fl, d_fr, d_bl, d_br = dtheta

        left = (d_fl + d_bl) / 2.0
        right = (d_fr + d_br) / 2.0

        wheel_radius = float(self.get_parameter('wheel_radius').value)
        v_left = (left / dt) * wheel_radius
        v_right = (right / dt) * wheel_radius
        v = (v_left + v_right) / 2.0

        spin_sum_threshold = float(self.get_parameter('spin_sum_threshold').value)
        spin_diff_threshold = float(self.get_parameter('spin_diff_threshold').value)

        is_spin = (abs(v_left + v_right) < spin_sum_threshold) and (
            abs(v_left - v_right) > spin_diff_threshold
        )
        if is_spin:
            v = 0.0
        else:
            self._x += v * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # Trust only linear.x velocity from wheel odom.
        odom.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 999.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 999.0,
        ]

        # Keep pose covariance large so downstream fusion can prioritize camera pose.
        huge = 0.09
        odom.pose.covariance = [
            huge, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, huge, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, huge, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, huge, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, huge, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, huge,
        ]

        self._pub.publish(odom)
        self._prev_positions = positions
        self._prev_time = now


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
