# point_nav

Point-to-point rover navigation that publishes joystick-style commands on `/joy`.

## Workspace Setup
- Edit source here: `/home/elder3/RAS-main/point_nav`
- Build/run workspace here: `/home/elder3/point_nav_ws`
- Symlink used: `/home/elder3/point_nav_ws/src/point_nav -> /home/elder3/RAS-main/point_nav`
- Helper script: `bash ~/RAS-main/point_nav/tools/setup_ws.sh`

Typical workflow:
```bash
bash ~/RAS-main/point_nav/tools/setup_ws.sh
cd ~/point_nav_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select point_nav --symlink-install
. install/setup.bash
ros2 launch point_nav point_nav.launch.py
```

## What It Does Now
- Follows goals using a P controller (`/goal_point` override + `/goal_point/add` queue).
- Uses fused odometry by default from `robot_localization` EKF (`/odometry/filtered`).
- Runs wheel odometry (`/joint_states -> /wheel/odom`) and fuses with camera odometry.
- EKF is configured for three camera odom inputs: front, left, right (`odom1/odom2/odom3`).
- Supports e-stop (`/point_nav/estop`) to immediately stop and clear queue.
- Supports file-based goals from YAML (`goal_inputs` in `config/goal_inputs.defaults.yaml`).
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

For a no-hardware validation path, see the **Dry test (no rover connected)** section in `QUICKSTART.md`.

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
ros2 run point_nav set_goal 2.0 0.5

# Add one goal to queue
ros2 run point_nav set_goal 1.0 -0.2 --queue

# Load goals from defaults goal-input file (override first, then queue rest)
ros2 run point_nav set_goal

# Load defaults goal-input file in queue mode
ros2 run point_nav set_goal --queue

# One-off slower run (per-goal throttle)
ros2 run point_nav set_goal 2.0 0.0 --throttle-scale 0.6

# E-stop engage / release
ros2 run point_nav set_goal --estop
ros2 run point_nav set_goal --clear-estop
```

## Throttle Controls
- `global_throttle_scale`: global multiplier for all runs.
- `forward_axis_scale` / `backward_axis_scale`: trigger output shaping per direction.
- Per-goal `throttle_scale`: from `goal_inputs[].throttle_scale` in `goal_inputs.defaults.yaml` or `set_goal --throttle-scale`.

Effective speed is influenced by all of the above.

## Config Files
- Main ROS params: `config/point_nav.defaults.yaml`
- File-based goal list: `config/goal_inputs.defaults.yaml`
- EKF fusion config: `config/ekf_fusion.yaml`
- Rover description (xacro + fallback URDF): `urdf/`

## Multi-Camera Notes
- `config/ekf_fusion.yaml` currently expects:
  - `/zed/zed_node/odom`
  - `/zed_left/zed_node/odom`
  - `/zed_right/zed_node/odom`
- If left/right cameras are not connected yet, update `ekf_fusion.yaml` before launch (remove/disable `odom2` and `odom3` blocks).
- Side camera mounts are active in both:
  - `urdf/robot_description.urdf.xacro`
  - `urdf/point_nav_rover.urdf`
- Xacro supports per-camera model args (`camera_model_front`, `camera_model_left`, `camera_model_right`) for `zed2i`/`zedm` style swaps.

## Notes
- Default xacro model references `zed_wrapper` macros. If unavailable, launch with `use_xacro_description:=false` to use fallback URDF.
- Wheel odometry requires valid `/joint_states` with wheel joint names.
- For EKF + ZED integration, disable per-camera TF publishing in ZED nodes to avoid TF conflicts.
