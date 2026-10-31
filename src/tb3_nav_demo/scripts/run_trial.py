#!/usr/bin/env python3
"""Drive a fixed route on the saved map (AMCL mode) and record how well it went.

Writes a JSON report: per-goal outcome, time and distance, ground-truth clearance to obstacles,
and (with --surprise) whether Nav2 saw and replanned around a box dropped into its path.

  ros2 run tb3_nav_demo run_trial.py --route course --out trial.json
  ros2 run tb3_nav_demo run_trial.py --route corridor --surprise --out surprise.json
"""
import argparse
import json
import math
import subprocess
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from nav_utils import SPAWN, go_to, make_navigator, set_initial_pose, wait_until_active
from world_geometry import ROBOT_RADIUS, Box, default_world, gap, load_obstacles

# World-frame goals (x, y, yaw).
ROUTES = {
    # North gap -> corridor past the pillar -> south gap -> east clutter -> north-east corner.
    'course': [(0.0, -2.3, -1.57), (3.3, -0.6, 1.57), (2.0, 2.4, 3.14)],
    # Down the corridor only; used for the surprise obstacle.
    'corridor': [(0.0, -2.3, -1.57)],
}

# Surprise box: dropped on the corridor's west lane (the planned path) once the robot has
# turned into the corridor. Leaves 0.4 m on the west (too narrow) and 1.0 m on the east.
SURPRISE = Box('surprise_box', -0.3, -1.0, 0.0, 0.2, 0.2)
SURPRISE_SDF = (
    '<sdf version="1.6"><model name="surprise_box"><static>true</static><link name="link">'
    '<collision name="c"><geometry><box><size>0.4 0.4 0.6</size></box></geometry></collision>'
    '<visual name="v"><geometry><box><size>0.4 0.4 0.6</size></box></geometry>'
    '<material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material></visual>'
    '</link></model></sdf>'
)


class Recorder:
    """Collects ground-truth samples, global plans and local-costmap hits while goals run."""

    def __init__(self, nav, shapes, surprise: bool):
        self.nav, self.shapes, self.surprise = nav, shapes, surprise
        self.samples = []        # (t, x, y, speed, gap)
        self.plans = []          # (t, min distance from plan to the surprise box)
        self.spawn_t = None
        self.costmap_hit_t = None
        nav.create_subscription(Odometry, 'odom', self.on_odom, 10)
        if surprise:
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            nav.create_subscription(Path, 'plan', self.on_plan, 10)
            nav.create_subscription(OccupancyGrid, 'local_costmap/costmap', self.on_local_costmap, latched)

    def now(self) -> float:
        return self.nav.get_clock().now().nanoseconds / 1e9

    def on_odom(self, msg: Odometry):
        # Gazebo odometry is drift-free here; its frame starts at the spawn pose.
        x = msg.pose.pose.position.x + SPAWN[0]
        y = msg.pose.pose.position.y + SPAWN[1]
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        shapes = self.shapes + ([SURPRISE] if self.spawn_t is not None else [])
        self.samples.append((self.now(), x, y, speed, gap(shapes, x, y)))
        robot_in_corridor = x > -0.9 and y < 1.5
        if self.surprise and self.spawn_t is None and robot_in_corridor:
            self.spawn_box()

    def spawn_box(self):
        self.spawn_t = self.now()
        print(f'  >> dropping surprise box at ({SURPRISE.x}, {SURPRISE.y})', flush=True)
        subprocess.Popen(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'default',
                          '-name', SURPRISE.name, '-x', str(SURPRISE.x), '-y', str(SURPRISE.y),
                          '-z', '0.3', '-string', SURPRISE_SDF],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def on_plan(self, msg: Path):
        if msg.poses:
            d = min(SURPRISE.distance(p.pose.position.x, p.pose.position.y) for p in msg.poses)
            self.plans.append((self.now(), d))

    def on_local_costmap(self, msg: OccupancyGrid):
        if self.spawn_t is None or self.costmap_hit_t is not None:
            return
        # The local costmap rolls with the robot in the odom frame, which starts at the spawn pose.
        bx, by = (SURPRISE.x - SPAWN[0], SURPRISE.y - SPAWN[1]) if msg.header.frame_id == 'odom' \
            else (SURPRISE.x, SURPRISE.y)
        info = msg.info
        col = int((bx - info.origin.position.x) / info.resolution)
        row = int((by - info.origin.position.y) / info.resolution)
        if 0 <= col < info.width and 0 <= row < info.height and msg.data[row * info.width + col] >= 99:
            self.costmap_hit_t = self.now()


def path_length(samples) -> float:
    return sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(samples, samples[1:]))


def summarize(rec: Recorder, goals, outcomes) -> dict:
    gaps = sorted(s[4] for s in rec.samples)
    report = {
        'succeeded': all(o.succeeded for o in outcomes),
        'goals_reached': sum(o.succeeded for o in outcomes),
        'goals_total': len(goals),
        'time_s': round(sum(o.duration_s for o in outcomes), 2),
        'distance_m': round(path_length(rec.samples), 3),
        'recoveries': sum(o.recoveries for o in outcomes),
        'min_gap_m': round(gaps[0], 3) if gaps else None,
        'p10_gap_m': round(gaps[len(gaps) // 10], 3) if gaps else None,
        'mean_gap_m': round(sum(gaps) / len(gaps), 3) if gaps else None,
        'mean_speed_mps': round(sum(s[3] for s in rec.samples) / max(len(rec.samples), 1), 3),
        'goals': [{'goal': g, **vars(o)} for g, o in zip(goals, outcomes)],
        'samples': [[round(v, 3) for v in s] for s in rec.samples[::3]],  # ~10 Hz for plotting
    }
    if rec.surprise:
        before = [d for t, d in rec.plans if rec.spawn_t is None or t < rec.spawn_t]
        after = [d for t, d in rec.plans if rec.spawn_t is not None and t >= rec.spawn_t]
        box_gaps = [SURPRISE.distance(s[1], s[2]) - ROBOT_RADIUS for s in rec.samples
                    if rec.spawn_t is not None and s[0] >= rec.spawn_t]
        report['surprise'] = {
            'spawned': rec.spawn_t is not None,
            'plan_to_box_before_m': round(before[-1], 3) if before else None,
            'plan_to_box_final_m': round(after[-1], 3) if after else None,
            # The robot is 0.22 m in radius; a plan farther than that from the box goes around it.
            'replanned_around_box': bool(after) and after[-1] > ROBOT_RADIUS,
            'local_costmap_latency_s': (round(rec.costmap_hit_t - rec.spawn_t, 2)
                                        if rec.costmap_hit_t and rec.spawn_t else None),
            'min_gap_to_box_m': round(min(box_gaps), 3) if box_gaps else None,
        }
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--route', choices=ROUTES, default='course')
    parser.add_argument('--surprise', action='store_true', help='drop a box into the corridor mid-run')
    parser.add_argument('--out', required=True, help='JSON report path')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    nav = make_navigator()
    rec = Recorder(nav, load_obstacles(default_world()), args.surprise)

    wait_until_active(nav, 'amcl')
    set_initial_pose(nav, *SPAWN)
    wait_until_active(nav, 'bt_navigator')

    goals = ROUTES[args.route]
    outcomes = []
    for x, y, yaw in goals:
        outcomes.append(go_to(nav, x, y, yaw))
        if not outcomes[-1].succeeded:
            break  # later goals assume the robot got here

    report = summarize(rec, goals, outcomes)
    with open(args.out, 'w') as f:
        json.dump(report, f, indent=1)
    print('RESULT: ' + json.dumps({k: v for k, v in report.items() if k not in ('goals', 'samples')}),
          flush=True)

    nav.destroy_node()
    rclpy.shutdown()
    return 0 if report['succeeded'] else 1


if __name__ == '__main__':
    sys.exit(main())
