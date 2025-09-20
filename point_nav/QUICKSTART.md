# Point-Nav Quickstart (ROS 2 Humble, Ubuntu 22.04)

## Prerequisites
- ROS 2 Humble installed: `source /opt/ros/humble/setup.bash`
- Colcon installed: `sudo apt install -y python3-colcon-common-extensions`

## Build (run once per change)
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select point_nav
. install/setup.bash  # tell this shell about the new build
```

## Every new terminal
Always do this before using ROS 2 in that terminal:
```bash
source /opt/ros/humble/setup.bash
. ~/ros2_ws/install/setup.bash
```

## Run the controller (Terminal A)
```bash
source /opt/ros/humble/setup.bash && . ~/ros2_ws/install/setup.bash
ros2 launch point_nav point_nav.launch.py
```

### Adjust status log rate (optional)
```bash
ros2 param set /point_nav status_log_period 0.5
```

## Send a goal (Terminal B)
```bash
source /opt/ros/humble/setup.bash && . ~/ros2_ws/install/setup.bash
# CLI helper
ros2 run point_nav set_goal 2.0 0.5

### CLI helper usage
Synopsis:
`ros2 run point_nav set_goal <x> <y> [--count N] [--delay S] [--topic NAME]`

- x, y: meters in rover frame (X forward, Y left). Use negative x to request reverse (requires `allow_reverse`=true).
- --count: publish N times (default 3)
- --delay: seconds between publishes (default 0.1)
- --topic: goal topic (default `/goal_point`)
```

### Reverse test (optional)
```bash
# Enable reverse behavior if not already true via YAML
ros2 param set /point_nav allow_reverse true

# Send a goal behind the current heading (negative X)
ros2 run point_nav set_goal -1.0 0.0
```

## If the rover/camera is not connected (simulate pose)
Terminal C:
```bash
source /opt/ros/humble/setup.bash && . ~/ros2_ws/install/setup.bash
# Publish a single pose at (0,0, yaw=0)
ros2 topic pub --once /zed/zed_node/pose geometry_msgs/PoseStamped '{header: {frame_id: "odom"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

# To simulate arrival at the goal position
ros2 topic pub --once /zed/zed_node/pose geometry_msgs/PoseStamped '{header: {frame_id: "odom"}, pose: {position: {x: 2.0, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

```
With controller running (Terminal A), send a goal (Terminal B) as above.

Watch Joy output (any terminal):
```bash
ros2 topic echo /joy   # Ctrl+C to stop
```

## Remap pose topic (if your ZED uses a different name)
```bash
ros2 launch point_nav point_nav.launch.py --ros-args -r /zed/zed_node/pose:=/your/pose/topic
```

## Tune parameters at runtime (optional)
```bash
# Slow linear speed further
ros2 param set /point_nav max_linear_speed 0.35
ros2 param set /point_nav k_v 0.5

# Gentler turns
ros2 param set /point_nav max_angular_speed 0.5
ros2 param set /point_nav k_w 0.8

# Stop distance
ros2 param set /point_nav goal_tolerance 0.3
```

## Common checks
```bash
ros2 node list                    # expect /point_nav
ros2 topic list | grep -E "goal_point|joy|pose"
ros2 topic info /joy -v
ros2 node info /point_nav         # publishers/subscribers and params
```

## Notes
- Axes mapping: Joy `axes[5]` = forward (1.0 neutral → -1.0 full), `axes[2]` = reverse (1.0 → -1.0), `axes[0]` = steer (left +1.0, right -1.0).
- If another program publishes `/joy` (e.g., joystick driver), stop it to avoid conflicts.
