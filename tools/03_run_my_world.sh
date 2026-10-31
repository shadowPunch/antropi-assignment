#!/usr/bin/env bash
# Step 03: smoke-test the tb3_nav_demo package: my world loads, and Nav2 drives from the
# start area to the far side of start_box (inside the area the lidar sees from spawn).
# There is no saved map yet, so this runs in SLAM mode (no AMCL, map built on the fly).
#   bash tools/03_run_my_world.sh
source "$(dirname "$0")/ros_env.sh"
source "$(dirname "$0")/sim_lib.sh"
set -uo pipefail

log_dir="$(cd "$(dirname "$0")/.." && pwd)/logs/03_my_world"
mkdir -p "$log_dir"

start_sim "$log_dir/launch.log" tb3_nav_demo nav_demo.launch.py slam:=True
ros2 run tb3_nav_demo send_goal.py -3.8 2.3 --slam 2>&1 | tee "$log_dir/goal.log"
goal_rc=${PIPESTATUS[0]}

echo "== world entities"
gz model --list 2>/dev/null | tee "$log_dir/models.txt"
dump_graph "$log_dir"
exit "$goal_rc"
