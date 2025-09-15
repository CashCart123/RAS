import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Joy


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    # Standard yaw extraction (Z axis) from quaternion.
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle: float) -> float:
    a = (angle + math.pi) % (2.0 * math.pi)
    if a < 0:
        a += 2.0 * math.pi
    return a - math.pi


class PointNavNode(Node):
    def __init__(self) -> None:
        super().__init__('point_nav_node')

        # Parameters
        self.declare_parameter('goal_tolerance', 0.2)  # meters
        self.declare_parameter('max_linear_speed', 1.0)  # normalized to joy [-1..1] -> map to triggers
        self.declare_parameter('max_angular_speed', 0.5)  # rad/s mapped to steering [-1..1]
        self.declare_parameter('k_v', 1.2)
        self.declare_parameter('k_w', 0.75)
        self.declare_parameter('angle_slowdown_threshold', 0.3)  # rad; reduce v when > threshold

        # Internal state
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._has_odom = False  # kept name for backward clarity

        self._goal: Optional[Point] = None
        self._active = False
        self._last_stop_sent = False

        # QoS suitable for ZED pose (usually reliable)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pose_sub = self.create_subscription(
            PoseStamped, '/zed/zed_node/pose', self._pose_cb, qos
        )
        self._goal_sub = self.create_subscription(
            Point, '/goal_point', self._goal_cb, qos
        )
        self._joy_pub = self.create_publisher(Joy, '/joy', 10)

        self._timer = self.create_timer(0.05, self._on_timer)  # 20 Hz

        self.get_logger().info('PointNavNode ready. Publish geometry_msgs/Point to /goal_point')

    # Callbacks
    def _pose_cb(self, msg: PoseStamped) -> None:
        self._x = msg.pose.position.x
        self._y = msg.pose.position.y
        q = msg.pose.orientation
        self._yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._has_odom = True

    def _goal_cb(self, msg: Point) -> None:
        self._goal = msg
        self._active = True
        self._last_stop_sent = False
        self.get_logger().info(f'Received goal: x={msg.x:.3f}, y={msg.y:.3f}')

    # Control loop
    def _on_timer(self) -> None:
        if not self._has_odom:
            # No odom yet; send neutral once to be safe then wait
            if not self._last_stop_sent:
                self._publish_stop()
                self._last_stop_sent = True
            return

        if not self._active or self._goal is None:
            # Idle; keep neutral periodically
            if not self._last_stop_sent:
                self._publish_stop()
                self._last_stop_sent = True
            return

        # Parameters (fetch lazily to allow dynamic changes)
        goal_tol = float(self.get_parameter('goal_tolerance').value)
        max_v = float(self.get_parameter('max_linear_speed').value)
        max_w = float(self.get_parameter('max_angular_speed').value)
        k_v = float(self.get_parameter('k_v').value)
        k_w = float(self.get_parameter('k_w').value)
        angle_slow = float(self.get_parameter('angle_slowdown_threshold').value)

        # Compute errors in odom frame (which is aligned with rover/camera axes as given)
        dx = self._goal.x - self._x
        dy = self._goal.y - self._y
        dist = math.hypot(dx, dy)

        if dist <= goal_tol:
            self._publish_stop()
            if not self._last_stop_sent:
                self.get_logger().info('Goal reached. Stopping.')
            self._active = False
            self._last_stop_sent = True
            return

        desired_heading = math.atan2(dy, dx)
        heading_error = wrap_to_pi(desired_heading - self._yaw)

        # Proportional controller
        v_cmd = k_v * dist
        # Slow or stop linear speed when heading error is large
        if abs(heading_error) > angle_slow:
            # reduce smoothly using cosine shaping; never negative here
            scale = max(0.0, math.cos(min(abs(heading_error), math.pi/2)))
            v_cmd *= scale

        # Clamp speeds
        v_cmd = max(0.0, min(max_v, v_cmd))
        w_cmd = max(-max_w, min(max_w, k_w * heading_error))

        # Continuous status printout about heading and turn command
        heading_off_deg = abs(math.degrees(heading_error))
        turn_dir = 'left' if w_cmd > 0.0 else ('right' if w_cmd < 0.0 else 'none')
        turn_mag_deg = abs(math.degrees(w_cmd))
        self.get_logger().info(
            f"i am {heading_off_deg:.1f} deg heading off and {dist:.2f} m away; "
            f"i will now turn {turn_dir} {turn_mag_deg:.1f} deg/s to fix this"
        )

        # Map to Joy
        joy_msg = self._joy_from_cmd(v_cmd, w_cmd, max_v, max_w)
        self._joy_pub.publish(joy_msg)
        self._last_stop_sent = False

    def _publish_stop(self) -> None:
        # Neutral: steering 0.0; triggers released -> 1.0
        joy = Joy()
        axes = [0.0] * 6  # ensure we have indices 0..5
        axes[0] = 0.0  # steering center
        axes[2] = 1.0  # backward trigger released
        axes[5] = 1.0  # forward trigger released
        joy.axes = axes
        joy.buttons = []
        self._joy_pub.publish(joy)

    @staticmethod
    def _joy_from_cmd(v_cmd: float, w_cmd: float, max_v: float, max_w: float) -> Joy:
        # Steering: map w_cmd in [-max_w, max_w] to [-1, 1]
        steer = 0.0 if max_w <= 0.0 else max(-1.0, min(1.0, w_cmd / max_w))

        # Triggers: neutral 1.0; increasing command moves toward -1.0
        forward_axis = 1.0
        backward_axis = 1.0

        if v_cmd >= 0.0:
            forward_axis = 1.0 if max_v <= 0.0 else 1.0 - 2.0 * max(0.0, min(1.0, v_cmd / max_v))
        else:
            # If reverse is ever used
            backward_axis = 1.0 if max_v <= 0.0 else 1.0 - 2.0 * max(0.0, min(1.0, (-v_cmd) / max_v))

        joy = Joy()
        axes = [0.0] * 6
        axes[0] = steer  # left +1.0, right -1.0
        axes[2] = backward_axis  # back trigger
        axes[5] = forward_axis   # fwd trigger
        joy.axes = axes
        joy.buttons = []
        return joy


def main(args=None):
    rclpy.init(args=args)
    node = PointNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
