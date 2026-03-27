# Point-Nav Old Quickstart

## 1) Build
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select point_nav
. install/setup.bash
```

## 2) Launch
```bash
source /opt/ros/humble/setup.bash
. ~/ros2_ws/install/setup.bash
ros2 launch point_nav point_nav.launch.py
```

This starts only the camera-pose controller node.

Useful launch overrides:
```bash
# Use a different controller params file
ros2 launch point_nav point_nav.launch.py params_file:=/path/to/point_nav.defaults.yaml

# Use a different file-goal list
ros2 launch point_nav point_nav.launch.py goal_file_path:=/path/to/goal_inputs.yaml
```

## 3) Send goals
```bash
# Override active goal
ros2 run point_nav set_goal 2.0 0.5

# Queue another goal
ros2 run point_nav set_goal 1.0 -0.2 --queue

# Override from file (first goal overrides, rest queue by default)
ros2 run point_nav set_goal

# Queue every goal from file
ros2 run point_nav set_goal --queue

# Direct goal with per-goal throttle
ros2 run point_nav set_goal 2.0 0.0 --throttle-scale 0.6
```

Useful CLI overrides:
```bash
# Ignore the default file and send only the direct goal
ros2 run point_nav set_goal 1.5 0.0 --no-file

# Use explicit config files
ros2 run point_nav set_goal --params-file /path/to/point_nav.defaults.yaml --file /path/to/goals.yaml

# Force override mode even if cli_defaults.mode is queue
ros2 run point_nav set_goal --override 2.0 0.5
```

## 4) Startup file loading
Edit `config/point_nav.defaults.yaml`:
```yaml
point_nav:
  ros__parameters:
    load_goals_on_startup: true
    goal_file_mode: queue
```

Then launch normally. The node will load `goal_file_path` at startup.

## 5) Simulate pose if the rover/camera is not connected
```bash
source /opt/ros/humble/setup.bash
. ~/ros2_ws/install/setup.bash

ros2 topic pub --once /zed/zed_node/pose geometry_msgs/PoseStamped \
'{header: {frame_id: "odom"},
  pose: {position: {x: 0.0, y: 0.0, z: 0.0},
         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

To simulate arrival at a goal:
```bash
ros2 topic pub --once /zed/zed_node/pose geometry_msgs/PoseStamped \
'{header: {frame_id: "odom"},
  pose: {position: {x: 2.0, y: 0.5, z: 0.0},
         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

## 6) Watch output
```bash
ros2 topic echo /joy
ros2 topic list | grep -E "goal_point|joy|pose"
ros2 node info /point_nav
```

## 7) Runtime tuning
```bash
ros2 param set /point_nav max_linear_speed 0.35
ros2 param set /point_nav global_throttle_scale 0.8
ros2 param set /point_nav status_log_period 0.5
ros2 param set /point_nav load_goals_on_startup true
```

## 8) Default config files
- Controller params: `/home/elder3/RAS-main/point_nav_old/config/point_nav.defaults.yaml`
- Goal file: `/home/elder3/RAS-main/point_nav_old/config/goal_inputs.defaults.yaml`

## Notes
- No URDF, wheel odom, EKF, or joint-state nodes are part of this package path.
- No staged move-then-turn behavior is enabled here; steering remains proportional throughout the run.
