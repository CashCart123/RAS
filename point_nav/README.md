# point_nav

Point-to-point navigation for a rover using visual odometry, publishing joystick-like commands to `/joy`.

- ROS 2: Humble (Ubuntu 22.04)
- Input pose: `/zed/zed_node/pose` (`geometry_msgs/PoseStamped`)
- Output command: `/joy` (`sensor_msgs/Joy`)
- Goal input: `/goal_point` (`geometry_msgs/Point`) in rover/camera frame (X forward, Y left, Z up)

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
ros2 topic pub /goal_point geometry_msgs/Point "{x: 2.0, y: 1.0, z: 0.0}" -1
```

## Parameters

- `goal_tolerance` (float, m): default 0.2
- `max_linear_speed` (float, normalized): default 0.6
- `max_angular_speed` (float, rad/s normalized): default 1.0
- `k_v` (float): default 0.6
- `k_w` (float): default 1.5
- `angle_slowdown_threshold` (float, rad): default 0.6

Override via the launch file or `ros2 param set` while running.

## Assumptions

- ZED pose frame axes are aligned with rover: X forward, Y left, Z up.
- The vehicle consuming `/joy` interprets the provided axes as described above.
- If your setup differs (e.g., different axes indices or neutral values), adjust the mapping in `point_nav/point_nav/point_nav_node.py`.

## Docs

- Quick start build/run and simulation: [QUICKSTART.md](./QUICKSTART.md)
- Move to another PC via Git (VS Code or CLI): [MOVING.md](./MOVING.md)
