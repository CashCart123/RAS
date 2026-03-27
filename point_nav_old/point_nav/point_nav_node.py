import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Joy

try:
    import yaml
except ImportError:  # pragma: no cover - ROS environments normally include pyyaml
    yaml = None


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle: float) -> float:
    a = (angle + math.pi) % (2.0 * math.pi)
    if a < 0:
        a += 2.0 * math.pi
    return a - math.pi


@dataclass
class GoalItem:
    x: float
    y: float
    throttle_scale: float


class PointNavNode(Node):
    def __init__(self) -> None:
        super().__init__('point_nav_node')

        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('k_v', 0.5)
        self.declare_parameter('k_w', 0.8)
        self.declare_parameter('angle_slowdown_threshold', 0.3)
        self.declare_parameter('forward_axis_scale', 0.5)
        self.declare_parameter('backward_axis_scale', 0.5)
        self.declare_parameter('steer_axis_scale', 0.7)
        self.declare_parameter('status_log_period', 1.0)
        self.declare_parameter('allow_reverse', True)

        self.declare_parameter('camera_pose_topic', '/zed/zed_node/pose')
        self.declare_parameter('goal_override_topic', '/goal_point')
        self.declare_parameter('goal_add_topic', '/goal_point/add')
        self.declare_parameter('global_throttle_scale', 1.0)
        self.declare_parameter('goal_file_path', '')
        self.declare_parameter('load_goals_on_startup', False)
        self.declare_parameter('goal_file_mode', 'queue')

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._has_pose = False

        self._goal_queue: List[GoalItem] = []
        self._goal: Optional[GoalItem] = None
        self._active = False

        self._last_stop_sent = False
        self._last_status_log_ns = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        pose_topic = str(self.get_parameter('camera_pose_topic').value)
        override_topic = str(self.get_parameter('goal_override_topic').value)
        add_topic = str(self.get_parameter('goal_add_topic').value)

        self._pose_sub = self.create_subscription(PoseStamped, pose_topic, self._pose_cb, qos)
        self._goal_override_sub = self.create_subscription(Point, override_topic, self._goal_override_cb, qos)
        self._goal_add_sub = self.create_subscription(Point, add_topic, self._goal_add_cb, qos)
        self._joy_pub = self.create_publisher(Joy, '/joy', 10)
        self._timer = self.create_timer(0.05, self._on_timer)

        if bool(self.get_parameter('load_goals_on_startup').value):
            self._load_goals_from_file()

        self.get_logger().info(
            f'PointNav ready. pose={pose_topic}, override={override_topic}, add={add_topic}'
        )

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._x = msg.pose.position.x
        self._y = msg.pose.position.y
        q = msg.pose.orientation
        self._yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._has_pose = True

    def _goal_override_cb(self, msg: Point) -> None:
        self._goal_queue.clear()
        self._goal = self._goal_from_point(msg)
        self._active = True
        self._last_stop_sent = False
        self.get_logger().info(
            f'Override goal: x={self._goal.x:.3f}, y={self._goal.y:.3f}, throttle={self._goal.throttle_scale:.2f}'
        )

    def _goal_add_cb(self, msg: Point) -> None:
        goal = self._goal_from_point(msg)
        self._goal_queue.append(goal)

        if not self._active and self._goal is None:
            self._activate_next_goal()
        else:
            self.get_logger().info(
                f'Queued goal #{len(self._goal_queue)}: x={goal.x:.3f}, y={goal.y:.3f}, throttle={goal.throttle_scale:.2f}'
            )

    def _goal_from_point(self, msg: Point) -> GoalItem:
        throttle = float(msg.z) if msg.z > 0.0 else 1.0
        throttle = max(0.05, min(1.0, throttle))
        return GoalItem(x=float(msg.x), y=float(msg.y), throttle_scale=throttle)

    def _activate_next_goal(self) -> bool:
        if not self._goal_queue:
            self._goal = None
            self._active = False
            return False

        self._goal = self._goal_queue.pop(0)
        self._active = True
        self._last_stop_sent = False
        self.get_logger().info(
            f'Activated queued goal: x={self._goal.x:.3f}, y={self._goal.y:.3f}, throttle={self._goal.throttle_scale:.2f}; '
            f'{len(self._goal_queue)} remaining'
        )
        return True

    def _on_timer(self) -> None:
        if not self._has_pose:
            if not self._last_stop_sent:
                self._publish_stop()
                self._last_stop_sent = True
            return

        if not self._active or self._goal is None:
            if self._activate_next_goal():
                return
            if not self._last_stop_sent:
                self._publish_stop()
                self._last_stop_sent = True
            return

        goal_tol = float(self.get_parameter('goal_tolerance').value)
        max_v = float(self.get_parameter('max_linear_speed').value)
        max_w = float(self.get_parameter('max_angular_speed').value)
        k_v = float(self.get_parameter('k_v').value)
        k_w = float(self.get_parameter('k_w').value)
        angle_slow = float(self.get_parameter('angle_slowdown_threshold').value)
        forward_scale = float(self.get_parameter('forward_axis_scale').value)
        backward_scale = float(self.get_parameter('backward_axis_scale').value)
        allow_reverse = bool(self.get_parameter('allow_reverse').value)
        steer_scale = float(self.get_parameter('steer_axis_scale').value)
        global_throttle = float(self.get_parameter('global_throttle_scale').value)

        dx = self._goal.x - self._x
        dy = self._goal.y - self._y
        dist = math.hypot(dx, dy)

        if dist <= goal_tol:
            self.get_logger().info(
                f'Goal reached. x={self._goal.x:.3f}, y={self._goal.y:.3f}, remaining={len(self._goal_queue)}'
            )
            self._goal = None
            self._active = False
            self._publish_stop()
            self._last_stop_sent = True
            return

        desired_heading = math.atan2(dy, dx)
        heading_error = wrap_to_pi(desired_heading - self._yaw)

        use_reverse = False
        heading_eff = heading_error
        if allow_reverse and abs(heading_error) > (math.pi / 2.0):
            use_reverse = True
            heading_eff = wrap_to_pi((desired_heading + math.pi) - self._yaw)

        v_cmd = (-k_v * dist) if use_reverse else (k_v * dist)
        if abs(heading_eff) > angle_slow:
            slowdown = max(0.0, math.cos(min(abs(heading_eff), math.pi / 2.0)))
            v_cmd *= slowdown

        v_cmd = max(-max_v, min(max_v, v_cmd))
        w_cmd = max(-max_w, min(max_w, k_w * heading_eff))

        throttle_scale = max(0.05, min(1.0, self._goal.throttle_scale * max(0.05, min(1.0, global_throttle))))
        v_cmd *= throttle_scale

        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(max(0.1, float(self.get_parameter('status_log_period').value)) * 1e9)
        if now_ns - self._last_status_log_ns >= period_ns:
            motion = 'forward' if v_cmd >= 0.0 else 'reverse'
            self.get_logger().info(
                f'motion={motion} dist={dist:.2f}m head_err={math.degrees(heading_error):.1f}deg '
                f'v={v_cmd:.2f} w={w_cmd:.2f} throttle={throttle_scale:.2f}'
            )
            self._last_status_log_ns = now_ns

        joy_msg = self._joy_from_cmd(v_cmd, w_cmd, max_v, max_w, forward_scale, backward_scale, steer_scale)
        self._joy_pub.publish(joy_msg)
        self._last_stop_sent = False

    def _load_goals_from_file(self) -> None:
        file_path = str(self.get_parameter('goal_file_path').value).strip()
        if not file_path:
            self.get_logger().warn('load_goals_on_startup=true but goal_file_path is empty.')
            return
        if yaml is None:
            self.get_logger().error('PyYAML is unavailable. Cannot load goal file.')
            return

        path = Path(file_path)
        if not path.exists():
            self.get_logger().warn(f'Goal file not found: {path}')
            return

        try:
            data = yaml.safe_load(path.read_text()) or {}
        except Exception as exc:  # pragma: no cover - safety logging
            self.get_logger().error(f'Failed to parse goal file {path}: {exc}')
            return

        goals = self._extract_goals_from_yaml(data)
        if not goals:
            self.get_logger().warn(f'No goals found in {path}. Expected key: goal_inputs')
            return

        mode = str(self.get_parameter('goal_file_mode').value).strip().lower()
        if mode == 'override':
            self._goal_queue = goals[1:]
            self._goal = goals[0]
            self._active = True
            self._last_stop_sent = False
            self.get_logger().info(f'Loaded {len(goals)} goal(s) from {path} in override mode.')
            return

        self._goal_queue.extend(goals)
        if not self._active and self._goal is None:
            self._activate_next_goal()
        self.get_logger().info(f'Loaded {len(goals)} goal(s) from {path} in queue mode.')

    @staticmethod
    def _extract_goals_from_yaml(data: object) -> List[GoalItem]:
        goal_items = None
        if isinstance(data, dict):
            if isinstance(data.get('goal_inputs'), list):
                goal_items = data.get('goal_inputs')
            elif isinstance(data.get('point_nav'), dict):
                pn = data.get('point_nav')
                if isinstance(pn.get('goal_inputs'), list):
                    goal_items = pn.get('goal_inputs')
                elif isinstance(pn.get('ros__parameters'), dict):
                    if isinstance(pn['ros__parameters'].get('goal_inputs'), list):
                        goal_items = pn['ros__parameters']['goal_inputs']

        if not isinstance(goal_items, list):
            return []

        goals: List[GoalItem] = []
        for item in goal_items:
            if not isinstance(item, dict):
                continue
            if 'x' not in item or 'y' not in item:
                continue
            throttle = max(0.05, min(1.0, float(item.get('throttle_scale', 1.0))))
            goals.append(GoalItem(x=float(item['x']), y=float(item['y']), throttle_scale=throttle))
        return goals

    def _publish_stop(self) -> None:
        joy = Joy()
        axes = [0.0] * 6
        axes[0] = 0.0
        axes[2] = 1.0
        axes[5] = 1.0
        joy.axes = axes
        joy.buttons = []
        self._joy_pub.publish(joy)

    @staticmethod
    def _joy_from_cmd(
        v_cmd: float,
        w_cmd: float,
        max_v: float,
        max_w: float,
        forward_scale: float = 1.0,
        backward_scale: float = 1.0,
        steer_scale: float = 1.0,
    ) -> Joy:
        steer = 0.0 if max_w <= 0.0 else max(-1.0, min(1.0, w_cmd / max_w))
        steer *= max(0.0, min(1.0, steer_scale))

        forward_axis = 1.0
        backward_axis = 1.0

        if v_cmd >= 0.0:
            scale = max(0.0, min(1.0, forward_scale))
            forward_axis = 1.0 if max_v <= 0.0 else 1.0 - 2.0 * scale * max(0.0, min(1.0, v_cmd / max_v))
        else:
            scale = max(0.0, min(1.0, backward_scale))
            backward_axis = 1.0 if max_v <= 0.0 else 1.0 - 2.0 * scale * max(0.0, min(1.0, (-v_cmd) / max_v))

        joy = Joy()
        axes = [0.0] * 6
        axes[0] = steer
        axes[2] = backward_axis
        axes[5] = forward_axis
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
