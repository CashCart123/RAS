# point_nav

Point-to-point navigation for a rover using visual odometry, publishing joystick-like commands to `/joy`.

- ROS 2: Humble (Ubuntu 22.04)
- Input pose: `/zed/zed_node/pose` (`geometry_msgs/PoseStamped`)
- Output command: `/joy` (`sensor_msgs/Joy`)
- Goal input: `/goal_point` (`geometry_msgs/Point`) in rover/camera frame (X forward, Y left)

## Control behavior

- Closed-loop P controller:
  - Linear: `v = k_v * distance`, limited by `max_linear_speed`.
  - Angular: `w = k_w * heading_error`, limited by `max_angular_speed`.
  - Reduces linear speed when heading error is large.
- Stops when within `goal_tolerance` meters of the goal and publishes neutral Joy.

## Joy mapping

- `axes[5]` (forward trigger): neutral 1.0, full forward -1.0
- `axes[2]` (back trigger): neutral 1.0, full reverse -1.0
- `axes[0]` (steering): left +1.0, right -1.0

## Build and run

Assuming a ROS 2 workspace at `~/ros2_ws` (or use your own):

```
# Create overlay workspace if needed
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Copy or symlink this package into src, or clone your repo here
# e.g., if this repo is at /home/cashcart/Documents/RAS
ln -s /home/cashcart/Documents/RAS/point_nav .

cd ..
colcon build --packages-select point_nav
. install/setup.bash

# Run the node
ros2 launch point_nav point_nav.launch.py

# Send a goal in rover frame (meters)
# CLI helper
ros2 run point_nav set_goal 2.0 1.0

## CLI helper usage

Synopsis:
`ros2 run point_nav set_goal <x> <y> [--count N] [--delay S] [--topic NAME]`

- x: forward distance (meters) in the rover frame. Positive is forward; negative is backward.
- y: lateral distance (meters) in the rover frame. Positive is left; negative is right.
- --count N: publish the same goal N times (default 3). The command repeats the goal message N times, spaced by `--delay`, to improve delivery if you send a goal before the node subscribes (startup race) or if QoS timing drops a single message.
- --delay S: seconds between repeated publishes (default 0.1). Typical range 0.05–0.2.
- --topic NAME: goal topic (default `/goal_point`). Include namespace if used (e.g., `/ns/goal_point`).

Examples:
- `ros2 run point_nav set_goal 1.5 0.0`         # straight 1.5 m ahead
- `ros2 run point_nav set_goal 2.0 -0.5`        # 2.0 m ahead, 0.5 m to the right
- `ros2 run point_nav set_goal 0.0 1.0 --count 5 --delay 0.2`  # send multiple times

## Parameters (launch defaults via YAML)

- `goal_tolerance` (float, m): default 0.2
- `max_linear_speed` (float, normalized): default 0.4
- `max_angular_speed` (float, rad/s normalized): default 0.5
- `k_v` (float): default 0.5
- `k_w` (float): default 0.8
- `angle_slowdown_threshold` (float, rad): default 0.3

- `forward_axis_scale` (0.0..1.0): scales forward trigger output; 0.5 caps to half‑press (keeps top speed at ~50%).
- `backward_axis_scale` (0.0..1.0): scales reverse trigger output; e.g., 0.5 caps reverse to half‑press.
- `steer_axis_scale` (0.0..1.0): scales steering output; e.g., 0.7 limits to 70% lock.
- `status_log_period` (seconds): throttle period for the status line (default 1.0).
- `allow_reverse` (bool): when true, the rover will back up if the goal is largely behind it (> ~90° off current heading).

These defaults live in `config/point_nav.defaults.yaml` and are loaded by the launch file. Override via another YAML:

`ros2 launch point_nav point_nav.launch.py params_file:=/path/to/custom.yaml`

Or tweak live with params:

`ros2 param set /point_nav max_linear_speed 0.35`

### Parameter details
- goal_tolerance: stop within this distance of the goal (m).
- max_linear_speed: hard cap on forward command; lower to reduce top speed.
- k_v: linear gain; lower for gentler acceleration toward the cap.
- max_angular_speed: cap on steering magnitude; lower for gentler turns.
- k_w: angular gain; lower for slower heading corrections.
- angle_slowdown_threshold: radians of heading error above which linear speed is reduced; smaller means slow down sooner when misaligned.

### Custom params file
- Create a YAML with only the params you want to override (others use defaults):

```
# my_params.yaml
point_nav:
  ros__parameters:
    max_linear_speed: 0.35
    k_v: 0.45
    max_angular_speed: 0.5
    k_w: 0.8
    goal_tolerance: 0.25
    angle_slowdown_threshold: 0.3
```

- Launch using your custom parameters:

```
ros2 launch point_nav point_nav.launch.py params_file:=/full/path/to/my_params.yaml
```

## Assumptions

- ZED pose frame axes are aligned with rover: X forward, Y left, Z up.
- The vehicle consuming `/joy` interprets the provided axes as described above.
- If your setup differs (e.g., different axes indices or neutral values), adjust the mapping in `point_nav/point_nav/point_nav_node.py`.

Note on hills / elevation (Z):
- The controller is intentionally planar (2D). It reads pose X,Y and ignores Z (elevation).
- Heading (yaw) is computed from the quaternion about the global Z axis, so roll/pitch from slopes do not affect steering.

## Docs

- Quick start build/run and simulation: [QUICKSTART.md](./QUICKSTART.md)
- Move to another PC via Git (VS Code or CLI): [MOVING.md](./MOVING.md)

## Status Logging

- The controller prints a status line at info level every `status_log_period` seconds (default 1.0), including movement direction ("moving forward/backward") and turn direction.
- Change the rate at runtime:

```
ros2 param set /point_nav status_log_period 0.5
```

## Topic Remapping (optional)

- If your pose topic differs from `/zed/zed_node/pose`, remap at launch:

```
ros2 launch point_nav point_nav.launch.py --ros-args -r /zed/zed_node/pose:=/your/pose/topic
```
