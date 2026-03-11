#!/usr/bin/env bash

TARGET_IP="192.168.1.5"
TARGET_USER="ras_unswcbr"
TARGET_PASS="Rov3rChallenge@2025"
CHILD_PIDS=()

cleanup() {
    echo
    echo "Script interrupted. Killing all spawned Terminator windows..."
    for pid in "${CHILD_PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    exit 1
}
trap cleanup SIGINT SIGTERM

echo "→ Opening ZED camera launch terminals on ${TARGET_USER}@${TARGET_IP}"

# 1) ZED 2i
terminator -e "bash -c '\"ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i camera_name:=zed serial_number:=35346563 pos_tracking.pos_tracking_enabled:=true pos_tracking.publish_tf:=true pos_tracking.publish_map_tf:=false pos_tracking.publish_odom_pose:=true; exec bash\"; exec bash'" &
CHILD_PIDS+=($!)

sleep 2

# 2) ZED Mini left
terminator -e "bash -c '\"ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm camera_name:=zed_left serial_number:=15316378 pos_tracking.pos_tracking_enabled:=true pos_tracking.publish_tf:=false pos_tracking.publish_map_tf:=false pos_tracking.publish_odom_pose:=true; exec bash\"; exec bash'" &
CHILD_PIDS+=($!)

sleep 2

# 3) ZED Mini right
terminator -e "bash -c '\"ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm camera_name:=zed_right serial_number:=15172184 pos_tracking.pos_tracking_enabled:=true pos_tracking.publish_tf:=false pos_tracking.publish_map_tf:=false pos_tracking.publish_odom_pose:=true; exec bash\"; exec bash'" &
CHILD_PIDS+=($!)

echo
echo "All camera terminals opened."
echo "Press Ctrl+C in this launcher window to close all spawned Terminator windows."
echo

wait