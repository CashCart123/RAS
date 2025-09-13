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

## Send a goal (Terminal B)
```bash
source /opt/ros/humble/setup.bash && . ~/ros2_ws/install/setup.bash
ros2 topic pub --once /goal_point geometry_msgs/Point "{x: 2.0, y: 0.5, z: 0.0}"
```

## If the rover/camera is not connected (simulate odometry)
Terminal C:
```bash
source /opt/ros/humble/setup.bash && . ~/ros2_ws/install/setup.bash
# Start fake odom at 20 Hz at (0,0, yaw=0)
ros2 topic pub -r 20 /zed/zed_node/odom nav_msgs/Odometry '{header: {frame_id: "odom"}, child_frame_id: "base_link", pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'
```
With controller running (Terminal A), send a goal (Terminal B) as above.

Watch Joy output (any terminal):
```bash
ros2 topic echo /joy   # Ctrl+C to stop
```

## Remap odometry topic (if your ZED uses a different name)
```bash
ros2 launch point_nav point_nav.launch.py --ros-args -r /zed/zed_node/odom:=/your/odom/topic
```

## Tune parameters at runtime (optional)
```bash
ros2 param set /point_nav k_v 0.6
ros2 param set /point_nav k_w 1.5
ros2 param set /point_nav goal_tolerance 0.3
```

## Common checks
```bash
ros2 node list                    # expect /point_nav
ros2 topic list | grep -E "goal_point|joy|odom"
ros2 topic info /joy -v
```

## Notes
- Axes mapping: Joy `axes[5]` = forward (1.0 neutral → -1.0 full), `axes[2]` = reverse (1.0 → -1.0), `axes[0]` = steer (left +1.0, right -1.0).
- If another program publishes `/joy` (e.g., joystick driver), stop it to avoid conflicts.
