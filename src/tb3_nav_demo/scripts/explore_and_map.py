#!/usr/bin/env python3
"""Drive a coverage route through my_world while slam_toolbox builds the map, then save it.

Scripted stand-in for teleop driving in step 04: Nav2 plans through unknown space
(allow_unknown: true), so the robot can reach areas the map has not fully seen yet.

  ros2 run tb3_nav_demo explore_and_map.py --save ~/tb3_nav_ws/src/tb3_nav_demo/maps/my_world
"""
import argparse
import subprocess
import sys

import rclpy
import yaml
from rclpy.utilities import remove_ros_args
from nav_utils import SPAWN, go_to, make_navigator, wait_until_active

# World coordinates. Ordered so each goal lies inside the area already seen by the lidar:
# Nav2 rejects goals outside the current map bounds.
COVERAGE_ROUTE = [
    (-3.8, 0.0, 1.57),    # west start area
    (-2.0, 2.3, 0.0),     # north-west, around start_box
    (-0.1, 2.3, -1.57),   # enter corridor through the north gap
    (-0.2, -0.5, -1.57),  # corridor, west of the pillar
    (0.0, -2.4, 0.0),     # corridor south end
    (2.0, -2.4, 0.0),     # through the south gap into the east area
    (3.3, -0.6, 1.57),    # clutter centre
    (4.0, 1.0, 1.57),     # east side
    (2.0, 2.5, 3.14),     # north-east corner
]


def save_map(prefix: str) -> None:
    """Save /map with map_saver_cli, then move its origin into the Gazebo world frame.

    slam_toolbox's map frame starts at the spawn pose; shifting the origin by SPAWN makes the
    saved map's frame equal the world frame, so AMCL poses and goals use world coordinates.
    """
    subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', prefix,
                    '--ros-args', '-p', 'use_sim_time:=true'], check=True)
    yaml_path = f'{prefix}.yaml'
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    meta['origin'][0] += SPAWN[0]
    meta['origin'][1] += SPAWN[1]
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False, default_flow_style=None)
    print(f'saved {yaml_path} (origin shifted to world frame: {meta["origin"]})', flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--save', metavar='PREFIX', help='save the map to PREFIX.yaml / PREFIX.pgm when done')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    nav = make_navigator()
    wait_until_active(nav, 'bt_navigator')

    reached = 0
    for i, (x, y, yaw) in enumerate(COVERAGE_ROUTE, 1):
        print(f'[{i}/{len(COVERAGE_ROUTE)}] heading to ({x}, {y})', flush=True)
        reached += go_to(nav, x, y, yaw, slam=True).succeeded
    print(f'RESULT: reached {reached}/{len(COVERAGE_ROUTE)} coverage points', flush=True)

    nav.destroy_node()
    rclpy.shutdown()

    if args.save:
        save_map(args.save)
    return 0 if reached == len(COVERAGE_ROUTE) else 1


if __name__ == '__main__':
    sys.exit(main())
