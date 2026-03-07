# Point-Nav Quickstart (ROS 2 Humble)

## 1) Build
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select point_nav
. install/setup.bash
```

## 2) Launch stack
```bash
source /opt/ros/humble/setup.bash
. ~/ros2_ws/install/setup.bash
ros2 launch point_nav point_nav.launch.py
```

This starts:
- `robot_state_publisher` (xacro by default)
- `wheel_odom_node`
- `ekf_filter_node`
- `point_nav_node`

## 3) Send goals
```bash
# Override current goal
ros2 run point_nav set_goal 2.0 0.5 --override

# Queue additional goal
ros2 run point_nav set_goal 1.0 -0.3 --add-to-queue
```

## 4) File-based goals
Edit `config/point_nav.defaults.yaml` under `goal_inputs`, then:
```bash
ros2 run point_nav set_goal --from-defaults --file-mode queue
```

## 5) E-stop
```bash
ros2 run point_nav set_goal --estop
ros2 run point_nav set_goal --clear-estop
```

## 6) Useful toggles
```bash
# If zed_wrapper xacro macros are not available
ros2 launch point_nav point_nav.launch.py use_xacro_description:=false

# If you have wheel ticks topic and need joint_states
ros2 launch point_nav point_nav.launch.py enable_joint_state_from_ticks:=true

# Disable EKF (fallback to camera+wheel dead-reckoning mode in point_nav)
ros2 launch point_nav point_nav.launch.py enable_ekf:=false
```

## 6.1) When adding new cameras later
- Uncomment and tune camera mount placeholders in:
  - `urdf/robot_description.urdf.xacro` (primary path)
  - `urdf/point_nav_rover.urdf` (fallback path)
- Uncomment `odom2` / `odom3` placeholder blocks in `config/ekf_fusion.yaml`.
- Relaunch and confirm new odom topics + TF are valid before trusting fusion.

## 7) Sanity checks
```bash
ros2 topic list | grep -E "goal_point|odometry/filtered|wheel/odom|joy|joint_states|point_nav/estop"
ros2 node list
ros2 topic echo /odometry/filtered
```

## 8) Runtime tuning examples
```bash
ros2 param set /point_nav global_throttle_scale 0.7
ros2 param set /point_nav max_linear_speed 0.35
ros2 param set /point_nav status_log_period 0.5
```
