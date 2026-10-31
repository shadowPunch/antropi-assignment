#!/usr/bin/env python3
"""Set the initial pose (AMCL mode) and drive to one goal.

Goal is in Gazebo world coordinates.
  ros2 run tb3_nav_demo send_goal.py 3.3 -0.6          # AMCL mode: goal x y [yaw]
  ros2 run tb3_nav_demo send_goal.py -3.8 2.3 --slam   # SLAM mode: no AMCL, map frame starts at spawn
"""
import argparse
import sys

import rclpy
from rclpy.utilities import remove_ros_args
from nav_utils import SPAWN, go_to, make_navigator, set_initial_pose, wait_until_active


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('yaw', type=float, nargs='?', default=0.0)
    parser.add_argument('--initial', type=float, nargs=3, default=SPAWN, metavar=('X', 'Y', 'YAW'),
                        help='initial pose for AMCL (default: spawn pose)')
    parser.add_argument('--slam', action='store_true', help='SLAM mode: skip AMCL, shift goal into the SLAM map frame')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    nav = make_navigator()
    if not args.slam:
        wait_until_active(nav, 'amcl')
        set_initial_pose(nav, *args.initial)
    wait_until_active(nav, 'bt_navigator')
    ok = go_to(nav, args.x, args.y, args.yaw, slam=args.slam).succeeded
    print(f'RESULT: {"SUCCEEDED" if ok else "FAILED"}', flush=True)

    nav.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
