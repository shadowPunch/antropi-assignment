#!/usr/bin/env bash
# Step 02: stock TB3 + Nav2 simulation: set the initial pose, drive behind the pillars,
# and record nodes/topics/TF. Needs the tb3_nav_demo workspace built (for send_goal.py).
#   bash tools/02_run_baseline.sh          # stops the sim when done
#   bash tools/02_run_baseline.sh --keep   # leaves Gazebo + RViz open for manual clicking
source "$(dirname "$0")/ros_env.sh"
source "$(dirname "$0")/sim_lib.sh"
set -uo pipefail

keep=false; [[ ${1:-} == --keep ]] && keep=true
log_dir="$(cd "$(dirname "$0")/.." && pwd)/logs/02_baseline"
mkdir -p "$log_dir"

start_sim "$log_dir/launch.log" nav2_bringup tb3_simulation_launch.py headless:=False
ros2 run tb3_nav_demo send_goal.py 1.8 0.5 2>&1 | tee "$log_dir/goal.log"
goal_rc=${PIPESTATUS[0]}

echo "== under the hood"
dump_graph "$log_dir"

if $keep; then
  echo "== sim left running; press Ctrl+C to stop"
  wait "$SIM_PID"
fi
exit "$goal_rc"
