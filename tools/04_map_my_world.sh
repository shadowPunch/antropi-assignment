#!/usr/bin/env bash
# Step 04: map my_world with SLAM, save the map into the package, then relaunch on the saved
# map and check AMCL localizes and Nav2 can cross the whole course.
#   bash tools/04_map_my_world.sh
source "$(dirname "$0")/ros_env.sh"
source "$(dirname "$0")/sim_lib.sh"
set -uo pipefail

log_dir="$(cd "$(dirname "$0")/.." && pwd)/logs/04_map"
map_prefix="$TB3_WS/src/tb3_nav_demo/maps/my_world"
mkdir -p "$log_dir"

echo "== phase 1: SLAM + coverage drive"
start_sim "$log_dir/slam_launch.log" tb3_nav_demo nav_demo.launch.py slam:=True
ros2 run tb3_nav_demo explore_and_map.py --save "$map_prefix" 2>&1 | tee "$log_dir/explore.log"
explore_rc=${PIPESTATUS[0]}
stop_sim
[[ -f $map_prefix.yaml ]] || { echo "FAIL: map was not saved"; exit 1; }

echo "== rebuild so the new map is installed"
(cd "$TB3_WS" && colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  >"$log_dir/build.log" 2>&1) || { echo "FAIL: build, see $log_dir/build.log"; exit 1; }

echo "== phase 2: AMCL on the saved map, drive the S-course into the east clutter"
start_sim "$log_dir/amcl_launch.log" tb3_nav_demo nav_demo.launch.py
ros2 run tb3_nav_demo send_goal.py 3.3 -0.6 2>&1 | tee "$log_dir/amcl_goal.log"
goal_rc=${PIPESTATUS[0]}

echo "== summary: explore exit $explore_rc, AMCL goal exit $goal_rc"
exit $(( explore_rc || goal_rc ))
