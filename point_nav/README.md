# point_nav

Point-to-point rover navigation that publishes joystick-style commands on `/joy`.

## What It Does Now
- Follows goals using a P controller (`/goal_point` override + `/goal_point/add` queue).
- Uses fused odometry by default from `robot_localization` EKF (`/odometry/filtered`).
- Runs wheel odometry (`/joint_states -> /wheel/odom`) and fuses with camera odometry.
- Supports e-stop (`/point_nav/estop`) to immediately stop and clear queue.
- Supports file-based goals from YAML (`goal_inputs`).
- Supports per-goal throttle (`Point.z`) and global throttle scaling.

## Main Topics
- Goal override: `/goal_point` (`geometry_msgs/Point`)
- Goal queue add: `/goal_point/add` (`geometry_msgs/Point`)
- E-stop: `/point_nav/estop` (`std_msgs/Bool`)
- Output command: `/joy` (`sensor_msgs/Joy`)
- EKF output odom: `/odometry/filtered` (`nav_msgs/Odometry`)
- Wheel odom: `/wheel/odom` (`nav_msgs/Odometry`)

## Launch
```bash
ros2 launch point_nav point_nav.launch.py
```

### Useful launch args
- `enable_ekf:=true|false`
- `enable_wheel_odom:=true|false`
- `enable_robot_state_publisher:=true|false`
- `use_xacro_description:=true|false` (default true; uses Simple_Autonomous xacro)
- `enable_joint_state_from_ticks:=true|false` (publish `/joint_states` from `/wheel_ticks`)
- `params_file:=/path/to/point_nav.defaults.yaml`
- `ekf_params_file:=/path/to/ekf_fusion.yaml`

## CLI (`set_goal`)
```bash
# Override active goal (default mode)
ros2 run point_nav set_goal 2.0 0.5 --override

# Add one goal to queue
ros2 run point_nav set_goal 1.0 -0.2 --add-to-queue

# Load goals from defaults file goal_inputs and queue them
ros2 run point_nav set_goal --from-defaults --file-mode queue

# One-off slower run (per-goal throttle)
ros2 run point_nav set_goal 2.0 0.0 --throttle-scale 0.6

# E-stop engage / release
ros2 run point_nav set_goal --estop
ros2 run point_nav set_goal --clear-estop
```

## Throttle Controls
- `global_throttle_scale`: global multiplier for all runs.
- `forward_axis_scale` / `backward_axis_scale`: trigger output shaping per direction.
- Per-goal `throttle_scale`: from `goal_inputs[].throttle_scale` or `set_goal --throttle-scale`.

Effective speed is influenced by all of the above.

## Config Files
- Main params + optional `goal_inputs`: `config/point_nav.defaults.yaml`
- EKF fusion config: `config/ekf_fusion.yaml`
- Rover description (xacro + fallback URDF): `urdf/`

## Future Camera Placeholders
- Placeholder entries for extra camera odometry are already added in `config/ekf_fusion.yaml` (commented as `odom2` and `odom3`).
- Placeholder mount blocks are already added in:
  - `urdf/robot_description.urdf.xacro`
  - `urdf/point_nav_rover.urdf`
- They are intentionally commented out, so current runtime behavior is unchanged until you uncomment and tune mounts/topics.

## Notes
- Default xacro model references `zed_wrapper` macros. If unavailable, launch with `use_xacro_description:=false` to use fallback URDF.
- Wheel odometry requires valid `/joint_states` with wheel joint names.
