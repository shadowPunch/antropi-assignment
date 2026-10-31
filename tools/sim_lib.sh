# Shared helpers for the step scripts. Source after ros_env.sh.
#   start_sim <log_file> <ros2 launch args...>   start a launch in its own process group
#   stop_sim                                     Ctrl+C it, then force-kill leftovers
#   dump_graph <dir>                             save nodes, topics and the TF tree PDF

start_sim() {
  local log=$1; shift
  local pid_file
  pid_file=$(mktemp)
  # New session so stop_sim reaches every node the launch spawns (Gazebo included).
  # setsid may fork, so $! is not reliable: the shell records its own PID, then execs the launch.
  setsid bash -c 'echo $$ >"$0"; exec ros2 launch "$@"' "$pid_file" "$@" >"$log" 2>&1 &
  until [[ -s $pid_file ]]; do sleep 0.1; done
  SIM_PID=$(<"$pid_file"); rm -f "$pid_file"
  trap stop_sim EXIT
  echo "== launched: ros2 launch $* (pid $SIM_PID, log $log)"
}

stop_sim() {
  [[ -n ${SIM_PID:-} ]] || return 0
  kill -INT -- "-$SIM_PID" 2>/dev/null
  for _ in {1..30}; do kill -0 -- "-$SIM_PID" 2>/dev/null || break; sleep 0.5; done
  kill -KILL -- "-$SIM_PID" 2>/dev/null
  SIM_PID=
}

dump_graph() {
  local dir=$1
  # Skip the CLI daemon: its discovery cache can return a partial graph right after launch.
  ros2 node list  --no-daemon --spin-time 5    | sort >"$dir/nodes.txt"
  ros2 topic list --no-daemon --spin-time 5 -t | sort >"$dir/topics.txt"
  echo "nodes: $(wc -l <"$dir/nodes.txt"), topics: $(wc -l <"$dir/topics.txt")"
  (cd "$dir" && ros2 run tf2_tools view_frames >view_frames.log 2>&1) && ls "$dir"/frames_*.pdf
}
