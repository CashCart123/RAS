# Point-Nav Quickstart (ROS 2 Humble)

## Workspace Layout (your setup)
```text
/home/elder3/RAS-main/point_nav           # source package you edit
/home/elder3/point_nav_ws/src/point_nav   # symlink to source package
```

Create/refresh that link:
```bash
bash ~/RAS-main/point_nav/tools/setup_ws.sh
```

## 1) Build
```bash
cd ~/point_nav_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select point_nav
. install/setup.bash
```

## 2) Launch stack
```bash
cd ~/point_nav_ws
source /opt/ros/humble/setup.bash
. install/setup.bash
ros2 launch point_nav point_nav.launch.py
```

This starts:
- `robot_state_publisher` (xacro by default)
- `wheel_odom_node`
- `ekf_filter_node`
- `point_nav_node`

Default EKF config expects 3 camera odom topics:
- `/zed/zed_node/odom`
- `/zed_left/zed_node/odom`
- `/zed_right/zed_node/odom`

## 3) Send goals
```bash
# Override current goal (default mode)
ros2 run point_nav set_goal 2.0 0.5

# Queue additional goal
ros2 run point_nav set_goal 1.0 -0.3 --queue
```

## 4) File-based goals
Edit `config/goal_inputs.defaults.yaml` under `goal_inputs`, then:
```bash
# Default file input (override first, queue rest)
ros2 run point_nav set_goal

# Queue all file goals
ros2 run point_nav set_goal --queue
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

## 6.1) If only one camera is installed right now
- Edit `config/ekf_fusion.yaml` and disable/remove `odom2` and `odom3` blocks.
- Keep only the front camera odom input active.
- Relaunch after saving the config.

## 6.2) ZED 2i + ZED Mini mix
- Camera mounts for front/left/right are already defined in URDF/xacro.
- In ZED node configs, disable camera TF publishing when EKF is used (`pos_tracking.publish_tf: false`).
- Ensure namespaces/topics match EKF config (`/zed`, `/zed_left`, `/zed_right`).

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

## 9) Dry test (no rover connected)
Use this to validate queue/override/e-stop and `/joy` output without hardware.

Terminal A:
```bash
source /opt/ros/humble/setup.bash
. ~/ros2_ws/install/setup.bash
ros2 launch point_nav point_nav.launch.py \
  enable_ekf:=false \
  enable_wheel_odom:=false \
  enable_robot_state_publisher:=false
```

Terminal B (fake odom feed for point_nav):
```bash
source /opt/ros/humble/setup.bash
. ~/ros2_ws/install/setup.bash
ros2 topic pub --once /odometry/filtered nav_msgs/Odometry \
'{header: {frame_id: "odom"},
  child_frame_id: "base_link",
  pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0},
                orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0},
                  angular: {x: 0.0, y: 0.0, z: 0.0}}}}'
```

Terminal C (drive tests):
```bash
source /opt/ros/humble/setup.bash
. ~/ros2_ws/install/setup.bash

# Observe output command
ros2 topic echo /joy

# In another tab/terminal:
ros2 run point_nav set_goal 2.0 0.5
ros2 run point_nav set_goal 1.0 0.0 --queue
ros2 run point_nav set_goal --estop
ros2 run point_nav set_goal --clear-estop
```

Optional goal-reached test (publish pose at goal once):
```bash
ros2 topic pub --once /odometry/filtered nav_msgs/Odometry \
'{header: {frame_id: "odom"},
  child_frame_id: "base_link",
  pose: {pose: {position: {x: 2.0, y: 0.5, z: 0.0},
                orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
  twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0},
                  angular: {x: 0.0, y: 0.0, z: 0.0}}}}'
```

### 9.1) File-input dry test (no rover)
After Terminals A/B/C above are running:

Terminal D:
```bash
source /opt/ros/humble/setup.bash
. ~/point_nav_ws/install/setup.bash

# Queue goals from config/goal_inputs.defaults.yaml
ros2 run point_nav set_goal --queue
```

Expected behavior:
- Terminal A logs queued + activated goals from file input.
- Terminal C (`ros2 topic echo /joy`) shows command changes as each file goal is executed.

Optional (file input in override mode):
```bash
ros2 run point_nav set_goal
```
