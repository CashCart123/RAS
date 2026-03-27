# point_nav_old

Camera-pose-only point navigation that publishes joystick-style commands on `/joy`.

This variant intentionally keeps the simple controller path and does not include:
- URDF / `robot_state_publisher`
- wheel odometry / joint-state publishing
- EKF fusion
- staged "move first, then turn" behavior

## What It Does Now
- Follows goals from `/zed/zed_node/pose` by default.
- Accepts goal override on `/goal_point`.
- Accepts queued goals on `/goal_point/add`.
- Supports file-based goals from `config/goal_inputs.defaults.yaml`.
- Supports per-goal throttle from `Point.z` or `goal_inputs[].throttle_scale`.
- Supports optional startup goal loading via config.

## Main Topics
- Pose input: `/zed/zed_node/pose` (`geometry_msgs/PoseStamped`)
- Goal override: `/goal_point` (`geometry_msgs/Point`)
- Goal queue add: `/goal_point/add` (`geometry_msgs/Point`)
- Output command: `/joy` (`sensor_msgs/Joy`)

## Launch
```bash
ros2 launch point_nav point_nav.launch.py
```

Launch arguments:
- `params_file:=/path/to/point_nav.defaults.yaml`
- `goal_file_path:=/path/to/goal_inputs.defaults.yaml`

`goal_file_path` is injected into the node separately so the main params file stays a normal ROS parameter file while the goal list stays editable as plain YAML.

## CLI (`set_goal`)
```bash
# Override the active goal
ros2 run point_nav set_goal 2.0 0.5

# Queue a goal
ros2 run point_nav set_goal 1.0 -0.3 --queue

# Override with one goal and queue the rest from the default goal file
ros2 run point_nav set_goal

# Queue all goals from the default goal file
ros2 run point_nav set_goal --queue

# Direct goal with a slower per-goal throttle
ros2 run point_nav set_goal 2.0 0.0 --throttle-scale 0.6

# Use a different params file and goal file
ros2 run point_nav set_goal --params-file /path/to/point_nav.defaults.yaml --file /path/to/goals.yaml
```

CLI defaults are read from `config/point_nav.defaults.yaml`:
- `goal_override_topic`
- `goal_add_topic`
- `cli_defaults.publish_count`
- `cli_defaults.publish_delay`
- `cli_defaults.mode`

If you run `set_goal` with no `x y`, it loads the packaged `config/goal_inputs.defaults.yaml` unless `--no-file` is given.

## Parameters
Main defaults live in `config/point_nav.defaults.yaml`.

Navigation:
- `goal_tolerance`
- `max_linear_speed`
- `max_angular_speed`
- `k_v`
- `k_w`
- `angle_slowdown_threshold`
- `allow_reverse`

Joy shaping:
- `forward_axis_scale`
- `backward_axis_scale`
- `steer_axis_scale`
- `status_log_period`

Goal and file input:
- `camera_pose_topic`
- `goal_override_topic`
- `goal_add_topic`
- `global_throttle_scale`
- `goal_file_path`
- `load_goals_on_startup`
- `goal_file_mode`

## Goal File Format
`config/goal_inputs.defaults.yaml`:
```yaml
goal_inputs:
  - x: 1.5
    y: 0.0
    throttle_scale: 0.7
  - x: 2.0
    y: 0.8
    throttle_scale: 0.6
```

Behavior:
- `goal_file_mode: override` makes the first file goal replace the active goal and queues the rest.
- `goal_file_mode: queue` appends all file goals to the queue.
- `load_goals_on_startup: true` loads the file when the node starts.

## Throttle Behavior
- `global_throttle_scale` limits every run.
- Per-goal throttle comes from `Point.z` or `goal_inputs[].throttle_scale`.
- Effective linear command is `global_throttle_scale * per_goal_throttle`, clamped to `0.05..1.0`.

## Notes
- The controller is planar. It uses X/Y position and yaw only.
- Goal coordinates are in the same frame as the incoming pose.
- If another node publishes `/joy`, stop it or remap to avoid conflicts.

## Docs
- Quick start: [QUICKSTART.md](/home/elder3/RAS-main/point_nav_old/QUICKSTART.md)
- Move to another PC: [MOVING.md](/home/elder3/RAS-main/point_nav_old/MOVING.md)
- Task list: [TODO.md](/home/elder3/RAS-main/point_nav_old/TODO.md)
