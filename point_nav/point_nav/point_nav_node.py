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
        # Defaults below are conservative and match the installed YAML; YAML will override at runtime
        self.declare_parameter('max_linear_speed', 0.4)  # normalized to joy [-1..1]
        self.declare_parameter('max_angular_speed', 0.5)  # rad/s equivalent, mapped to steering [-1..1]
        self.declare_parameter('k_v', 0.5)
        self.declare_parameter('k_w', 0.8)
        self.declare_parameter('angle_slowdown_threshold', 0.3)  # rad; reduce v when > threshold
        # Limit how far the forward trigger is pressed: 1.0 (neutral) -> -1.0 (full).
        # Setting forward_axis_scale=0.5 caps to half-press (axis min 0.0), preventing full speed.
        self.declare_parameter('forward_axis_scale', 0.5)
        # Limit reverse trigger (axes[2]) when commanding reverse.
        self.declare_parameter('backward_axis_scale', 0.5)
        # Limit steering output on axes[0] to avoid full-lock turns. 1.0 = no cap, 0.5 = half lock.
        self.declare_parameter('steer_axis_scale', 0.7)
        # Throttle status update period (seconds). Set to 1.0 for 1 Hz.
        self.declare_parameter('status_log_period', 1.0)
        # Allow reversing when goal is largely behind rover
        self.declare_parameter('allow_reverse', True)

        # Internal state
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        # We operate in the ground (XY) plane only; Z is ignored.
        # Track whether we have received at least one pose.
        self._has_pose = False

        self._goal: Optional[Point] = None
        self._active = False
        self._last_stop_sent = False
        self._last_status_log_ns = 0  # throttle logs

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
        # Use only X,Y from pose; Z (elevation) is deliberately ignored so hills do not affect control.
        self._x = msg.pose.position.x
        self._y = msg.pose.position.y
        q = msg.pose.orientation
        self._yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._has_pose = True

    def _goal_cb(self, msg: Point) -> None:
        self._goal = msg
        self._active = True
        self._last_stop_sent = False
        self.get_logger().info(f'Received goal: x={msg.x:.3f}, y={msg.y:.3f}')

    # Control loop
    def _on_timer(self) -> None:
        if not self._has_pose:
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
        forward_scale = float(self.get_parameter('forward_axis_scale').value)
        backward_scale = float(self.get_parameter('backward_axis_scale').value)
        allow_reverse = bool(self.get_parameter('allow_reverse').value)

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

        # Decide whether to reverse based on heading alignment
        use_reverse = False
        heading_eff = heading_error
        if allow_reverse and abs(heading_error) > (math.pi / 2.0):
            use_reverse = True
            heading_eff = wrap_to_pi((desired_heading + math.pi) - self._yaw)

        # Proportional controller
        v_cmd = (-k_v * dist) if use_reverse else (k_v * dist)
        if abs(heading_eff) > angle_slow:
            scale = max(0.0, math.cos(min(abs(heading_eff), math.pi/2)))
            v_cmd *= scale

        # Clamp speeds (allow reverse)
        v_cmd = max(-max_v, min(max_v, v_cmd))
        w_cmd = max(-max_w, min(max_w, k_w * heading_eff))

        # Throttled status about heading and turn command (configurable)
        now_ns = self.get_clock().now().nanoseconds
        status_period = float(self.get_parameter('status_log_period').value) if self.has_parameter('status_log_period') else 1.0
        period_ns = max(0.1, status_period) * 1_000_000_000
        if now_ns - self._last_status_log_ns >= period_ns:
            heading_off_deg = abs(math.degrees(heading_error))
            turn_dir = 'left' if w_cmd > 0.0 else ('right' if w_cmd < 0.0 else 'none')
            turn_mag_deg = abs(math.degrees(w_cmd))
            motion = 'forward' if v_cmd >= 0.0 else 'backward'
            # Print at INFO so it shows in normal runs; still throttled to avoid spam.
            self.get_logger().info(
                f"moving {motion}; heading error {heading_off_deg:.1f} deg, dist {dist:.2f} m; "
                f"turning {turn_dir} {turn_mag_deg:.1f} deg/s"
            )
            self._last_status_log_ns = now_ns

        # Map to Joy
        steer_scale = float(self.get_parameter('steer_axis_scale').value) if self.has_parameter('steer_axis_scale') else 1.0
        joy_msg = self._joy_from_cmd(v_cmd, w_cmd, max_v, max_w, forward_scale, backward_scale, steer_scale)
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
    def _joy_from_cmd(v_cmd: float, w_cmd: float, max_v: float, max_w: float, forward_scale: float = 1.0, backward_scale: float = 1.0, steer_scale: float = 1.0) -> Joy:
        # Steering: map w_cmd in [-max_w, max_w] to [-1, 1], then scale to limit full lock
        steer = 0.0 if max_w <= 0.0 else max(-1.0, min(1.0, w_cmd / max_w))
        steer_scale = max(0.0, min(1.0, steer_scale))
        steer *= steer_scale

        # Triggers: neutral 1.0; increasing command moves toward -1.0
        forward_axis = 1.0
        backward_axis = 1.0

        if v_cmd >= 0.0:
            # Apply output scaling so the trigger is never fully pressed.
            scale = max(0.0, min(1.0, forward_scale))
            forward_axis = 1.0 if max_v <= 0.0 else 1.0 - 2.0 * scale * max(0.0, min(1.0, v_cmd / max_v))
        else:
            # Reverse command: apply reverse trigger scaling
            bscale = max(0.0, min(1.0, backward_scale))
            backward_axis = 1.0 if max_v <= 0.0 else 1.0 - 2.0 * bscale * max(0.0, min(1.0, (-v_cmd) / max_v))

        joy = Joy()
        axes = [0.0] * 6
        axes[0] = steer  # left +scale, right -scale
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
