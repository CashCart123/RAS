# two_link_plotter.py
# Run on your ROS 2 machine (not on the Teensy):
#   python3 two_link_plotter.py

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# Send pacing
MIN_SEND_GAP = 0.02        # ~50 Hz
IDLE_ZERO_HEARTBEAT = 0.02 # keep publishing 0 dps at ~50 Hz when idle

# Telemetry plotting
PLOT_HZ = 10
WINDOW_SEC = 60.0

# ----------------------------
# Geometry and limits
# ----------------------------
L1, L2 = 0.5, 0.5
R_MAX = L1 + L2

JOY_H_AXIS = 3   # +1 left,  -1 right (ROS joy_node)
JOY_V_AXIS = 4   # +1 up,    -1 down
JOY_DEADZONE = 0.10

MAX_EE_SPEED = 0.2         # m/s at full stick
MAX_JOINT_DPS = 90.0       # per joint cap
MAX_JOINT_RAD = math.radians(MAX_JOINT_DPS)

# ----------------------------
# Initialisation mode settings
# ----------------------------
INIT_TARGET_BASE_DEG = 30
INIT_TARGET_ELBOW_DEG = 160
INIT_KP = 2.0                 # proportional speed gain (dps per deg error)
INIT_MAX_DPS = 5.0           # cap during init move
INIT_TOL_DEG = 2.0            # finished when both joints within this tolerance

def clampf(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def clamp_outer(x, y):
    r = math.hypot(x, y)
    if r > R_MAX:
        s = R_MAX / (r + 1e-12)
        return x * s, y * s
    return x, y

def project_velocity_at_boundary(px, py, vx, vy, tol=1e-4):
    r = math.hypot(px, py)
    if r < R_MAX - tol or r < 1e-9:
        return vx, vy
    rx, ry = px / r, py / r
    vout = vx * rx + vy * ry
    if vout > 0.0:
        vx -= vout * rx
        vy -= vout * ry
    return vx, vy

class ArmPlot(Node):
    def __init__(self):
        super().__init__('arm_plot_ik_controller')

        # Raw state from Teensy (degrees)
        self.raw_th1_deg = 0.0
        self.raw_th2_deg = 0.0

        # Python-zeroed state (degrees)
        self.th1_deg = 0.0
        self.th2_deg = 180.0

        # Python-only zero offsets
        self.base_zero_deg = None
        self.elbow_zero_deg = None

        self.ee = Point()

        # IK target in task-space (kept continuous for boundary logic)
        self.target_x = 0.4
        self.target_y = 0.0

        # Control flags/timers
        self.has_sent_motion = False
        self.is_holding = False
        self.last_cmd_pub = 0.0
        self.last_joy_time = None

        # Initialisation mode state
        self.initialising = False

        # ROS I/O
        self.create_subscription(Float32, '/base_joint_deg',  self.cb_th1, 10)
        self.create_subscription(Float32, '/elbow_joint_deg', self.cb_th2, 10)
        self.create_subscription(Point,   '/ee_xy',           self.cb_ee,  10)
        self.create_subscription(Joy,     '/joy',             self.cb_joy, 10)

        # Telemetry subscriptions
        self.cur_base = 0.0
        self.cur_elbow = 0.0
        self.tmp_base = 0.0
        self.tmp_elbow = 0.0
        self.create_subscription(Float32, '/base_current_a', self.cb_base_i, 10)
        self.create_subscription(Float32, '/elbow_current_a', self.cb_elbow_i, 10)
        self.create_subscription(Float32, '/base_temp_c',    self.cb_base_t, 10)
        self.create_subscription(Float32, '/elbow_temp_c',   self.cb_elbow_t, 10)

        self.pub_v1 = self.create_publisher(Float32, '/cmd_joint1_speed_dps', 10)
        self.pub_v2 = self.create_publisher(Float32, '/cmd_joint2_speed_dps', 10)
        self.pub_hold = self.create_publisher(Point, '/hold_target_deg', 10)

        # Plot (arm)
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        plt.subplots_adjust(bottom=0.18)  # make room for the button

        self.ax.set_aspect('equal')
        r = L1 + L2
        self.ax.set_xlim(-r*1.1, r*1.1)
        self.ax.set_ylim(-r*1.1, r*1.1)
        self.ax.grid(True, alpha=0.3)
        (self.line,) = self.ax.plot([], [], '-o', lw=3)
        (self.eept,) = self.ax.plot([], [], 'x', ms=10)
        self.text = self.ax.text(
            0.02, 0.98, "", transform=self.ax.transAxes,
            ha='left', va='top'
        )

        # Initialisation button
        self.init_btn_ax = self.fig.add_axes([0.32, 0.04, 0.36, 0.08])
        self.init_button = Button(self.init_btn_ax, 'Initialisation Mode')
        self.init_button.on_clicked(self.on_init_button_clicked)

        plt.ion()
        plt.show(block=False)

        # Telemetry figure (currents & temps)
        self.fig2, (self.ax_i, self.ax_t) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        (self.li_base_i,)  = self.ax_i.plot([], [], label='Base I (A)')
        (self.li_elbow_i,) = self.ax_i.plot([], [], label='Elbow I (A)')
        self.ax_i.set_ylabel('Current (A)')
        self.ax_i.grid(True, alpha=0.3)
        self.ax_i.legend(loc='upper right')

        (self.li_base_t,)  = self.ax_t.plot([], [], label='Base Temp (°C)')
        (self.li_elbow_t,) = self.ax_t.plot([], [], label='Elbow Temp (°C)')
        self.ax_t.set_ylabel('Temp (°C)')
        self.ax_t.set_xlabel('Time (s)')
        self.ax_t.grid(True, alpha=0.3)
        self.ax_t.legend(loc='upper right')

        # Rolling buffers
        maxlen = int(WINDOW_SEC * PLOT_HZ) + 5
        self.tbuf = deque(maxlen=maxlen)
        self.base_i = deque(maxlen=maxlen)
        self.elbow_i = deque(maxlen=maxlen)
        self.base_t = deque(maxlen=maxlen)
        self.elbow_t = deque(maxlen=maxlen)
        self.t0 = time.time()

        # Timers
        self.timer = self.create_timer(1/30.0, self.redraw)
        self.timer2 = self.create_timer(1.0 / PLOT_HZ, self.redraw_telemetry)
        self.init_timer = self.create_timer(1.0 / 50.0, self.run_initialisation_mode)

    # ---------- helpers ----------
    def _pub_joint_speeds(self, v1_dps: float, v2_dps: float, now: float):
        """Rate-limited publisher for joint speeds (deg/s)."""
        if (now - self.last_cmd_pub) >= MIN_SEND_GAP:
            m1 = Float32(); m1.data = v1_dps
            m2 = Float32(); m2.data = v2_dps
            self.pub_v1.publish(m1)
            self.pub_v2.publish(m2)
            self.last_cmd_pub = now

    def _publish_hold_current_pose(self):
        hold = Point()
        hold.x = self.th1_deg
        hold.y = self.th2_deg
        hold.z = 0.0
        self.pub_hold.publish(hold)
        self.is_holding = True

    def _sync_target_to_current_pose(self):
        t1 = math.radians(self.th1_deg)
        t2 = math.radians(self.th2_deg)
        x1 = L1 * math.cos(t1)
        y1 = L1 * math.sin(t1)
        self.target_x = x1 + L2 * math.cos(t1 + t2)
        self.target_y = y1 + L2 * math.sin(t1 + t2)

    # ---------------- Callbacks ----------------
    def cb_th1(self, msg: Float32):
        # Raw from Teensy
        self.raw_th1_deg = msg.data

        # Python-only zeroing:
        # first received base angle becomes 0 deg
        if self.base_zero_deg is None:
            self.base_zero_deg = msg.data
            self.get_logger().info(f'Base zero captured at {self.base_zero_deg:.2f} deg')

        self.th1_deg = self.raw_th1_deg - self.base_zero_deg

    def cb_th2(self, msg: Float32):
        # Raw from Teensy
        self.raw_th2_deg = msg.data

        # Python-only zeroing:
        # first received elbow angle becomes 180 deg in the local IK frame
        if self.elbow_zero_deg is None:
            self.elbow_zero_deg = msg.data
            self.get_logger().info(
                f'Elbow zero captured at {self.elbow_zero_deg:.2f} deg -> mapped to 180 deg'
            )

        self.th2_deg = (self.raw_th2_deg - self.elbow_zero_deg) + 180.0

    def cb_ee(self, msg: Point):
        self.ee = msg

    # Telemetry callbacks
    def cb_base_i(self, msg: Float32):  self.cur_base  = msg.data
    def cb_elbow_i(self, msg: Float32): self.cur_elbow = msg.data
    def cb_base_t(self, msg: Float32):  self.tmp_base  = msg.data
    def cb_elbow_t(self, msg: Float32): self.tmp_elbow = msg.data

    def on_init_button_clicked(self, event):
        if self.base_zero_deg is None or self.elbow_zero_deg is None:
            self.get_logger().warn('Cannot start initialisation mode yet: waiting for joint zero capture.')
            return

        self.initialising = True
        self.is_holding = False
        self.has_sent_motion = True
        self.get_logger().info(
            f'Initialisation mode started: moving to base={INIT_TARGET_BASE_DEG:.1f}°, '
            f'elbow={INIT_TARGET_ELBOW_DEG:.1f}°'
        )

    def run_initialisation_mode(self):
        if not self.initialising:
            return

        now = time.time()

        # Error in local Python-zeroed joint frame
        err1 = INIT_TARGET_BASE_DEG - self.th1_deg
        err2 = INIT_TARGET_ELBOW_DEG - self.th2_deg

        # Done?
        if abs(err1) <= INIT_TOL_DEG and abs(err2) <= INIT_TOL_DEG:
            self._pub_joint_speeds(0.0, 0.0, now)
            self._publish_hold_current_pose()
            self._sync_target_to_current_pose()
            self.initialising = False
            self.get_logger().info('Initialisation mode complete.')
            return

        # Simple proportional speed controller
        v1 = clampf(INIT_KP * err1, -INIT_MAX_DPS, INIT_MAX_DPS)
        v2 = clampf(INIT_KP * err2, -INIT_MAX_DPS, INIT_MAX_DPS)

        # Prevent tiny jitter near target
        if abs(err1) <= INIT_TOL_DEG:
            v1 = 0.0
        if abs(err2) <= INIT_TOL_DEG:
            v2 = 0.0

        self._pub_joint_speeds(v1, v2, now)

    def cb_joy(self, joy: Joy):
        # Ignore joystick IK motion while initialising
        if self.initialising:
            return

        # Guard against early empty arrays from joy_node
        if len(joy.axes) <= max(JOY_H_AXIS, JOY_V_AXIS):
            return

        # Wait until Python zero has been captured
        if self.base_zero_deg is None or self.elbow_zero_deg is None:
            return

        now = time.time()
        if self.last_joy_time is None:
            self.last_joy_time = now
        dt = now - self.last_joy_time
        self.last_joy_time = now
        dt = min(dt, 1.0/15.0)

        # Right stick
        h_raw = -1.0 * joy.axes[JOY_H_AXIS]  # +1 left -> -x
        v_raw =  joy.axes[JOY_V_AXIS]        # +1 up   -> +y

        moving = (abs(h_raw) >= JOY_DEADZONE) or (abs(v_raw) >= JOY_DEADZONE)

        # Current Python-zeroed joint angles (radians)
        t1 = math.radians(self.th1_deg)
        t2 = math.radians(self.th2_deg)

        if moving:
            h = 0.0 if abs(h_raw) < JOY_DEADZONE else h_raw
            v = 0.0 if abs(v_raw) < JOY_DEADZONE else v_raw

            # Map stick -> EE velocity
            vx = -h * MAX_EE_SPEED
            vy =  v * MAX_EE_SPEED

            vx, vy = project_velocity_at_boundary(self.target_x, self.target_y, vx, vy)

            # Integrate EE target
            self.target_x += vx * dt
            self.target_y += vy * dt
            self.target_x, self.target_y = clamp_outer(self.target_x, self.target_y)

            # Jacobian inverse -> joint rates (IK sign; CCW+)
            s1, c1 = math.sin(t1), math.cos(t1)
            s12, c12 = math.sin(t1 + t2), math.cos(t1 + t2)
            a = -(L1*s1 + L2*s12); b = -L2*s12
            c =  (L1*c1 + L2*c12); d =  L2*c12
            det = a*d - b*c

            vth1 = 0.0
            vth2 = 0.0
            if abs(det) > 1e-4:
                inv = 1.0 / det
                vth1 =  ( d*vx - b*vy) * inv
                vth2 = (-c*vx + a*vy) * inv

            # Rate limits
            vth1 = clampf(vth1, -MAX_JOINT_RAD, MAX_JOINT_RAD)
            vth2 = clampf(vth2, -MAX_JOINT_RAD, MAX_JOINT_RAD)

            # Publish at ~50 Hz max
            self._pub_joint_speeds(math.degrees(vth1), math.degrees(vth2), now)
            self.has_sent_motion = True
            self.is_holding = False

        else:
            # Deadzone: send one-shot HOLD after we’ve moved at least once
            if self.has_sent_motion and not self.is_holding:
                self._publish_hold_current_pose()

            # Keep target_x/y coherent with current pose when idle
            self._sync_target_to_current_pose()

            # Zero-speed heartbeat
            self._pub_joint_speeds(0.0, 0.0, now)

    # ---------------- UI ----------------
    def redraw(self):
        t1 = math.radians(self.th1_deg)
        t2 = math.radians(self.th2_deg)
        p0 = (0.0, 0.0)
        p1 = (L1*math.cos(t1), L1*math.sin(t1))
        p2 = (p1[0] + L2*math.cos(t1+t2), p1[1] + L2*math.sin(t1+t2))

        self.line.set_data([p0[0], p1[0], p2[0]], [p0[1], p1[1], p2[1]])
        self.eept.set_data([self.ee.x], [self.ee.y])

        init_state = "ACTIVE" if self.initialising else "OFF"

        self.text.set_text(
            f"θ1={self.th1_deg:.1f}°, θ2={self.th2_deg:.1f}°\n"
            f"Raw θ1={self.raw_th1_deg:.1f}°, Raw θ2={self.raw_th2_deg:.1f}°\n"
            f"EE: ({self.ee.x:.3f}, {self.ee.y:.3f}) m\n"
            f"Init mode: {init_state}"
        )

        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def redraw_telemetry(self):
        t = time.time() - self.t0
        self.tbuf.append(t)
        self.base_i.append(self.cur_base)
        self.elbow_i.append(self.cur_elbow)
        self.base_t.append(self.tmp_base)
        self.elbow_t.append(self.tmp_elbow)

        x = list(self.tbuf)
        self.li_base_i.set_data(x, list(self.base_i))
        self.li_elbow_i.set_data(x, list(self.elbow_i))
        self.li_base_t.set_data(x, list(self.base_t))
        self.li_elbow_t.set_data(x, list(self.elbow_t))

        if x:
            xmin = max(0.0, x[-1] - WINDOW_SEC)
            self.ax_i.set_xlim(xmin, x[-1] + 0.1)

            def autoscale(ax, s1, s2):
                if s1 and s2:
                    ymin = min(min(s1), min(s2))
                    ymax = max(max(s1), max(s2))
                else:
                    ymin, ymax = 0.0, 1.0
                if ymin == ymax:
                    ymin -= 0.5
                    ymax += 0.5
                pad = 0.1 * (ymax - ymin)
                ax.set_ylim(ymin - pad, ymax + pad)

            autoscale(self.ax_i, self.base_i, self.elbow_i)
            autoscale(self.ax_t, self.base_t, self.elbow_t)

        self.fig2.canvas.draw_idle()
        plt.pause(0.001)

def main():
    rclpy.init()
    node = ArmPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()