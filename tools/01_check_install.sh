#!/usr/bin/env bash
# Step 01 sanity check: ROS pub/sub works and Gazebo can open a window.
#   bash tools/01_check_install.sh
source "$(dirname "$0")/ros_env.sh"
set -uo pipefail
log_dir="$(dirname "$0")/../logs"; mkdir -p "$log_dir"

echo "== ROS_DISTRO=$ROS_DISTRO"

echo "== talker -> listener (8 s)"
ros2 run demo_nodes_cpp talker >/dev/null 2>&1 &
talker=$!
heard=$(timeout 8 ros2 run demo_nodes_py listener 2>&1 | grep -c "I heard")
kill "$talker" 2>/dev/null; wait "$talker" 2>/dev/null
if (( heard > 0 )); then echo "PASS: listener heard $heard messages"; else echo "FAIL: listener heard nothing"; fi

echo "== gz sim shapes.sdf (window opens for 20 s, then closes)"
timeout 20 gz sim shapes.sdf >"$log_dir/gz_check.log" 2>&1
rc=$?
# 124 = killed by timeout, i.e. Gazebo was still running fine.
if (( rc == 124 )); then echo "PASS: Gazebo ran for 20 s"; else echo "FAIL: Gazebo exited with $rc, see $log_dir/gz_check.log"; fi
