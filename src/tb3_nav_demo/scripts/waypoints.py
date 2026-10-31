#!/usr/bin/env python3
"""Repeatable demo: set the initial pose, then follow 4 waypoints across the course (AMCL mode).

  ros2 run tb3_nav_demo waypoints.py
"""
import sys

import rclpy
from nav2_simple_commander.robot_navigator import TaskResult
from nav_utils import SPAWN, make_navigator, make_pose, set_initial_pose, wait_until_active

# World frame (x, y, yaw): around start_box, down the corridor, into the clutter, north-east corner.
WAYPOINTS = [
    (-3.8, 2.3, 0.0),
    (0.0, -2.3, 0.0),
    (3.3, -0.6, 1.57),
    (2.0, 2.4, 3.14),
]


def main() -> int:
    rclpy.init()
    nav = make_navigator()
    wait_until_active(nav, 'amcl')
    set_initial_pose(nav, *SPAWN)
    # waypoint_follower is activated after bt_navigator; a request sent in between is rejected.
    wait_until_active(nav, 'waypoint_follower')

    nav.followWaypoints([make_pose(nav, *wp) for wp in WAYPOINTS])
    current = -1
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb and fb.current_waypoint != current:
            current = fb.current_waypoint
            print(f'heading to waypoint {current + 1}/{len(WAYPOINTS)}: {WAYPOINTS[current][:2]}', flush=True)

    result = nav.getResult()
    print(f'RESULT: {result.name}', flush=True)
    nav.destroy_node()
    rclpy.shutdown()
    return 0 if result == TaskResult.SUCCEEDED else 1


if __name__ == '__main__':
    sys.exit(main())
