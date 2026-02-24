#!/usr/bin/env python3

import math
from enum import Enum
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
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


class Mode(Enum):
    ROTATE = 1
    DRIVE = 2
    IDLE = 3


class WaypointNavNode(Node):
    """
    Waypoint follower that ONLY:
      - spins on the spot to align heading
      - drives straight forward with zero steering (no differential steering while moving)

    Pose source: /odometry_filtered (nav_msgs/Odometry)
    Output: Joy to /joy (axes[0]=steer, axes[5]=forward trigger, axes[2]=reverse trigger)
    """

    def __init__(self) -> None:
        super().__init__("waypoint_nav_node")

        # ---- Parameters ----
        self.declare_parameter("goal_tolerance", 0.25)          # meters
        self.declare_parameter("heading_tolerance", 0.15)       # rad (~8.6 deg). Must be within this to start driving.
        self.declare_parameter("reorient_threshold", 0.35)      # rad (~20 deg). If exceeded while driving, stop+rotate.

        # Speeds / gains
        self.declare_parameter("max_linear_speed", 0.4)         # normalized mapping to trigger
        self.declare_parameter("max_angular_speed", 0.6)        # rad/s equivalent mapping to steer
        self.declare_parameter("k_v", 0.6)
        self.declare_parameter("k_w", 1.2)

        # Joy scaling / safety caps
        self.declare_parameter("forward_axis_scale", 0.6)       # cap forward trigger press (0..1)
        self.declare_parameter("steer_axis_scale", 0.7)         # cap steer magnitude (0..1)

        # Waypoints as a flat list: [x1, y1, x2, y2, ...]
        self.declare_parameter("waypoints", [0.0, 0.0])                 # list[float]

        # Logging throttle
        self.declare_parameter("status_log_period", 1.0)

        # ---- State ----
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._has_pose = False

        self._wps: List[Tuple[float, float]] = self._load_waypoints_param()
        self._wp_idx = 0
        self._mode = Mode.IDLE if not self._wps else Mode.ROTATE
        self._last_stop_sent = False
        self._last_status_log_ns = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self._odom_cb, qos
        )

        self._joy_pub = self.create_publisher(Joy, "/joy", 10)
        self._timer = self.create_timer(0.05, self._on_timer)  # 20 Hz

        if self._wps:
            self.get_logger().info(f"Loaded {len(self._wps)} waypoint(s). Starting.")
        else:
            self.get_logger().warn(
                "No waypoints provided. Set parameter 'waypoints: [x1, y1, x2, y2, ...]'"
            )

    def _load_waypoints_param(self) -> List[Tuple[float, float]]:
        raw = list(self.get_parameter("waypoints").value)
        if len(raw) == 0:
            return []
        if len(raw) % 2 != 0:
            self.get_logger().error(
                "Parameter 'waypoints' must contain an even number of floats: [x1,y1,x2,y2,...]"
            )
            return []
        wps = []
        for i in range(0, len(raw), 2):
            wps.append((float(raw[i]), float(raw[i + 1])))
        return wps

    def _odom_cb(self, msg: Odometry) -> None:
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._has_pose = True

    def _current_wp(self) -> Tuple[float, float]:
        return self._wps[self._wp_idx]

    def _advance_wp(self) -> None:
        self._wp_idx += 1
        if self._wp_idx >= len(self._wps):
            self._mode = Mode.IDLE
            self.get_logger().info("Final waypoint reached. Going IDLE.")
        else:
            self._mode = Mode.ROTATE
            wx, wy = self._current_wp()
            self.get_logger().info(f"Next waypoint {self._wp_idx+1}/{len(self._wps)}: ({wx:.2f}, {wy:.2f})")

    def _on_timer(self) -> None:
        if not self._has_pose:
            if not self._last_stop_sent:
                self._publish_stop()
                self._last_stop_sent = True
            return

        if self._mode == Mode.IDLE:
            if not self._last_stop_sent:
                self._publish_stop()
                self._last_stop_sent = True
            return

        goal_tol = float(self.get_parameter("goal_tolerance").value)
        head_tol = float(self.get_parameter("heading_tolerance").value)
        reorient_thr = float(self.get_parameter("reorient_threshold").value)

        max_v = float(self.get_parameter("max_linear_speed").value)
        max_w = float(self.get_parameter("max_angular_speed").value)
        k_v = float(self.get_parameter("k_v").value)
        k_w = float(self.get_parameter("k_w").value)

        forward_scale = float(self.get_parameter("forward_axis_scale").value)
        steer_scale = float(self.get_parameter("steer_axis_scale").value)

        wx, wy = self._current_wp()
        dx = wx - self._x
        dy = wy - self._y
        dist = math.hypot(dx, dy)

        desired_heading = math.atan2(dy, dx)
        heading_error = wrap_to_pi(desired_heading - self._yaw)

        # Reached waypoint?
        if dist <= goal_tol:
            self._publish_stop()
            self._last_stop_sent = True
            self.get_logger().info(
                f"Waypoint {self._wp_idx+1}/{len(self._wps)} reached (dist {dist:.2f} m)."
            )
            self._advance_wp()
            return

        # ---- State machine enforcing: spin OR straight only ----
        v_cmd = 0.0
        w_cmd = 0.0

        if self._mode == Mode.ROTATE:
            # Spin until facing waypoint within heading tolerance
            if abs(heading_error) <= head_tol:
                self._mode = Mode.DRIVE
                # Stop once at the transition to avoid mixing commands on the boundary
                self._publish_stop()
                self._last_stop_sent = True
                return

            # Spin on spot (no forward motion)
            w_cmd = max(-max_w, min(max_w, k_w * heading_error))
            v_cmd = 0.0

        elif self._mode == Mode.DRIVE:
            # Drive straight ONLY (steer forced to 0 at publish time)
            # If we drift too far off heading, stop and rotate again.
            if abs(heading_error) > reorient_thr:
                self._mode = Mode.ROTATE
                self._publish_stop()
                self._last_stop_sent = True
                return

            v_cmd = max(0.0, min(max_v, k_v * dist))  # forward only
            w_cmd = 0.0  # IMPORTANT: no steering while moving forward

        # ---- Throttled status logs ----
        now_ns = self.get_clock().now().nanoseconds
        status_period = float(self.get_parameter("status_log_period").value)
        period_ns = max(0.1, status_period) * 1_000_000_000
        if now_ns - self._last_status_log_ns >= period_ns:
            self.get_logger().info(
                f"mode={self._mode.name} wp={self._wp_idx+1}/{len(self._wps)} "
                f"dist={dist:.2f}m head_err={math.degrees(heading_error):.1f}deg "
                f"v={v_cmd:.2f} w={w_cmd:.2f}"
            )
            self._last_status_log_ns = now_ns

        # ---- Publish Joy enforcing your constraints ----
        joy_msg = self._joy_from_cmd(
            v_cmd=v_cmd,
            w_cmd=w_cmd,
            max_v=max_v,
            max_w=max_w,
            forward_scale=forward_scale,
            steer_scale=steer_scale,
            force_zero_steer_when_moving=True,   # <--- your key requirement
        )
        self._joy_pub.publish(joy_msg)
        self._last_stop_sent = False

    def _publish_stop(self) -> None:
        joy = Joy()
        axes = [0.0] * 6
        axes[0] = 0.0
        axes[2] = 1.0  # reverse released
        axes[5] = 1.0  # forward released
        joy.axes = axes
        joy.buttons = []
        self._joy_pub.publish(joy)

    @staticmethod
    def _joy_from_cmd(
        v_cmd: float,
        w_cmd: float,
        max_v: float,
        max_w: float,
        forward_scale: float,
        steer_scale: float,
        force_zero_steer_when_moving: bool,
    ) -> Joy:
        # Map angular -> steering axis
        steer = 0.0 if max_w <= 0.0 else max(-1.0, min(1.0, w_cmd / max_w))
        steer_scale = max(0.0, min(1.0, steer_scale))
        steer *= steer_scale

        # If we are moving forward, hard-force steering to zero (no differential steering while moving)
        if force_zero_steer_when_moving and v_cmd > 1e-6:
            steer = 0.0

        # Forward trigger: neutral 1.0, pressed toward -1.0
        forward_axis = 1.0
        if v_cmd > 0.0 and max_v > 0.0:
            scale = max(0.0, min(1.0, forward_scale))
            forward_axis = 1.0 - 2.0 * scale * max(0.0, min(1.0, v_cmd / max_v))

        # We never command reverse in this policy
        backward_axis = 1.0

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
    node = WaypointNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
