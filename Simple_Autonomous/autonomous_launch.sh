#!/usr/bin/env bash

# Always source bashrc
SOURCE_CMD="source ~/.bashrc && cd ~/Simple_Autonomous"

# --------------------------
# Joint state publisher (wheel ticks -> RViz robot motion)
# --------------------------
gnome-terminal --title="Wheel Joint States" -- bash -c "
$SOURCE_CMD
echo '[Wheel Joint State Publisher]'
python3 joint_state_publishing.py
exec bash
"

# --------------------------
# RViz (robot model + TF)
# --------------------------
gnome-terminal --title="RViz" -- bash -c "
$SOURCE_CMD
echo '[RViz]'
ros2 launch view_robot.launch.py
exec bash
"

# --------------------------
# Wheel odometry node (forward/back distance)
# --------------------------
gnome-terminal --title="Wheel Odometry" -- bash -c "
$SOURCE_CMD
echo '[Wheel Odometry]'
python3 wheel_odometry_node.py
exec bash
"

# --------------------------
# EKF (wheel-only)
# --------------------------
gnome-terminal --title="EKF (Wheel Only)" -- bash -c "
$SOURCE_CMD
echo '[EKF - Wheel Only]'
ros2 run robot_localization ekf_node --ros-args --params-file ekf_wheel_only.yaml
exec bash
"

# --------------------------
# ZED Camera
# --------------------------
gnome-terminal --title="ZED Camera" -- bash -c "
$SOURCE_CMD
echo '[ZED Camera]'
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  camera_name:=zed_front \
  base_frame:=base_link \
  publish_urdf:=false \
  publish_tf:=false
exec bash
"
