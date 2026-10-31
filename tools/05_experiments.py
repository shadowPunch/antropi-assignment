"""Step 05: run navigation trials under different Nav2 settings and log each one to W&B.

Each trial: write a params variant -> launch the sim headless on the saved map -> run
`run_trial.py` (or `waypoints.py`) -> stop the sim -> log config, time series and metrics.

  bash tools/05_experiments.sh                       # everything, 2 reps each
  bash tools/05_experiments.sh --only tune-tight --reps 1
  TB3_WANDB=0 bash tools/05_experiments.sh           # no W&B (offline)
"""
import argparse
import copy
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / 'logs' / '05_experiments'
PKG_SRC = Path(os.environ.get('TB3_WS', ROOT)) / 'src' / 'tb3_nav_demo'  # the repo is the workspace
STOCK_PARAMS = Path('/opt/ros/lyrical/share/nav2_bringup/params/nav2_params.yaml')
SHIPPED_PARAMS = PKG_SRC / 'config' / 'nav2_params.yaml'
WANDB_PROJECT = 'tb3-nav2'
USE_WANDB = os.environ.get('TB3_WANDB', '1') != '0'


def inflation(radius: float, scaling: float) -> dict:
    """Same inflation settings in both costmaps."""
    layer = {'inflation_layer': {'inflation_radius': radius, 'cost_scaling_factor': scaling}}
    return {'local_costmap': {'local_costmap': {'ros__parameters': layer}},
            'global_costmap': {'global_costmap': {'ros__parameters': layer}}}


# Regulated Pure Pursuit, from the Nav2 README, capped at the same 0.5 m/s as the stock MPPI config.
RPP = {'controller_server': {'ros__parameters': {'FollowPath': {
    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'max_linear_vel': 0.5, 'min_linear_vel': -0.5, 'max_angular_vel': 1.9, 'min_angular_vel': -1.9,
    'max_linear_accel': 2.5, 'max_linear_decel': -2.5, 'max_angular_accel': 3.2, 'max_angular_decel': -3.2,
    'lookahead_dist': 0.6, 'min_lookahead_dist': 0.3, 'max_lookahead_dist': 0.9, 'lookahead_time': 1.5,
    'rotate_to_heading_angular_vel': 1.8, 'use_velocity_scaled_lookahead_dist': False,
    'min_approach_linear_velocity': 0.05, 'approach_velocity_scaling_dist': 1.0,
    'use_collision_detection': True, 'max_allowed_time_to_collision_up_to_carrot': 1.0,
    'use_regulated_linear_velocity_scaling': True, 'use_cost_regulated_linear_velocity_scaling': False,
    'regulated_linear_scaling_min_radius': 0.9, 'regulated_linear_scaling_min_speed': 0.25,
    'use_fixed_curvature_lookahead': False, 'curvature_lookahead_dist': 1.0,
    'use_rotate_to_heading': True, 'rotate_to_heading_min_angle': 0.785,
    'interpolate_curvature_after_goal': False, 'cost_scaling_dist': 0.3, 'cost_scaling_gain': 1.0,
    'inflation_cost_scaling_factor': 3.0, 'use_dynamic_window': False,
    'allow_obstacle_checking_beyond_goal': False,
}}}}


@dataclass
class Experiment:
    id: str
    group: str
    description: str
    base: Path = STOCK_PARAMS
    overrides: dict = field(default_factory=dict)
    route: str = 'course'
    surprise: bool = False
    script: str = 'run_trial.py'


EXPERIMENTS = [
    Experiment('tune-stock', 'tuning', 'stock params: inflation 0.70 m, cost_scaling 3.0'),
    Experiment('tune-tight', 'tuning', 'inflation 0.45 m, cost_scaling 6.0', overrides=inflation(0.45, 6.0)),
    Experiment('tune-wide', 'tuning', 'inflation 1.00 m, cost_scaling 1.5', overrides=inflation(1.0, 1.5)),
    Experiment('ctrl-rpp', 'controller', 'Regulated Pure Pursuit instead of MPPI', overrides=RPP),
    Experiment('surprise-stock', 'surprise', 'stock params, box dropped into the corridor',
               route='corridor', surprise=True),
    Experiment('final-course', 'final', 'shipped package params', base=SHIPPED_PARAMS),
    Experiment('final-surprise', 'final', 'shipped params, box dropped into the corridor',
               base=SHIPPED_PARAMS, route='corridor', surprise=True),
    Experiment('final-waypoints', 'final', 'shipped params, waypoints.py (followWaypoints)',
               base=SHIPPED_PARAMS, script='waypoints.py'),
]


def deep_merge(base: dict, overrides: dict) -> dict:
    merged = copy.deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def flatten(d: dict, prefix: str = '') -> dict:
    out = {}
    for k, v in d.items():
        key = f'{prefix}.{k}' if prefix else k
        out.update(flatten(v, key) if isinstance(v, dict) else {key: v})
    return out


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:12]


def package_version(pkg: str) -> str:
    out = subprocess.run(['dpkg-query', '-W', '-f=${Version}', pkg], capture_output=True, text=True)
    return out.stdout.strip() or 'unknown'


def running_sims() -> list:
    """PIDs of Gazebo servers or Nav2 containers already running (a stale stack would duplicate
    node names and /clock, and corrupt the trial)."""
    pids = []
    for name in ('gz-sim-main', 'component_conta'):  # comm names are truncated to 15 chars
        out = subprocess.run(['pgrep', '-x', name], capture_output=True, text=True).stdout
        pids += [int(p) for p in out.split()]
    return pids


class Sim:
    """Headless simulation on the saved map, in its own session so one signal reaches every node."""

    def __init__(self, params: Path, log: Path):
        self.log = open(log, 'w')
        self.proc = subprocess.Popen(
            ['ros2', 'launch', 'tb3_nav_demo', 'nav_demo.launch.py', 'headless:=True', 'use_rviz:=False',
             f'params_file:={params}'],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def stop(self) -> None:
        for sig, wait_s in ((signal.SIGINT, 20), (signal.SIGKILL, 5)):
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + wait_s
            while time.monotonic() < deadline and process_group_alive(self.proc.pid):
                time.sleep(0.5)
        self.proc.wait()
        self.log.close()


def process_group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def run_one(exp: Experiment, rep: int, extra_tags: list) -> dict:
    run_dir = OUT / f'{exp.id}-r{rep}'
    run_dir.mkdir(parents=True, exist_ok=True)
    params = run_dir / 'params.yaml'
    params.write_text(yaml.safe_dump(deep_merge(yaml.safe_load(exp.base.read_text()), exp.overrides),
                                     sort_keys=False))
    report_path = run_dir / 'trial.json'
    config = {
        'experiment': exp.id, 'description': exp.description, 'rep': rep,
        'route': exp.route, 'surprise': exp.surprise, 'script': exp.script,
        'base_params': str(exp.base), 'params_sha': sha256(params),
        'overrides': flatten(exp.overrides),
        'world_sha': sha256(PKG_SRC / 'worlds' / 'my_world.sdf.xacro'),
        'map_sha': sha256(PKG_SRC / 'maps' / 'my_world.pgm'),
        'ros_distro': 'lyrical', 'nav2_version': package_version('ros-lyrical-navigation2'),
        'headless': True, 'stochastic_note': 'MPPI samples random trajectories; no fixed seed',
    }

    wb = None
    if USE_WANDB:
        import wandb
        wb = wandb.init(project=WANDB_PROJECT, group=exp.group, job_type=exp.script.removesuffix('.py'),
                        name=f'{exp.id}-r{rep}', config=config, tags=[exp.group, exp.id, *extra_tags],
                        reinit='finish_previous')

    print(f'\n=== {exp.id} rep {rep}: {exp.description}', flush=True)
    sim = Sim(params, run_dir / 'launch.log')
    cmd = ['ros2', 'run', 'tb3_nav_demo', exp.script]
    if exp.script == 'run_trial.py':
        cmd += ['--route', exp.route, '--out', str(report_path)] + (['--surprise'] if exp.surprise else [])
    started = time.monotonic()
    trial_log = run_dir / 'trial.log'
    with open(trial_log, 'w') as f:  # written live, so progress can be followed with tail -f
        try:
            rc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=900).returncode
        except subprocess.TimeoutExpired:
            f.write('\nTIMEOUT\n')
            rc = -1
    wall_s = time.monotonic() - started
    sim.stop()
    output = trial_log.read_text()
    print('\n'.join(line for line in output.splitlines() if 'goal world' in line or 'RESULT' in line
                    or '>>' in line or 'waypoint' in line), flush=True)

    report = json.loads(report_path.read_text()) if report_path.exists() else {}
    report.setdefault('succeeded', rc == 0)
    report['exit_code'], report['wall_s'] = rc, round(wall_s, 1)

    if wb:
        log_to_wandb(wb, report)
    return {'id': exp.id, 'rep': rep, **{k: v for k, v in report.items() if k not in ('samples', 'goals')}}


def log_to_wandb(wb, report: dict) -> None:
    import wandb
    samples = report.get('samples', [])
    if samples:
        t0 = samples[0][0]
        for t, x, y, speed, gap in samples:
            wb.log({'trial/t_s': t - t0, 'trial/speed_mps': speed, 'trial/gap_m': gap})
        wb.log({'trajectory': wandb.plot.scatter(
            wandb.Table(data=[[s[1], s[2]] for s in samples], columns=['x', 'y']), 'x', 'y',
            title='Ground-truth trajectory (world frame)')})
    if report.get('goals'):
        cols = ['x', 'y', 'yaw', 'succeeded', 'result', 'duration_s', 'recoveries', 'error']
        wb.log({'goals': wandb.Table(columns=cols, data=[[*g['goal'], g['succeeded'], g['result'],
                                                          g['duration_s'], g['recoveries'], g['error']]
                                                         for g in report['goals']])})
    summary = {k: v for k, v in report.items() if k not in ('samples', 'goals', 'surprise')}
    summary.update({f'surprise/{k}': v for k, v in report.get('surprise', {}).items()})
    wb.summary.update(summary)
    if not report.get('succeeded'):
        wb.tags = (*wb.tags, 'failed')
    wb.finish(exit_code=0 if report.get('succeeded') else 1)


def write_summary(rows: list, extra_tags: list) -> None:
    """Average reps per experiment into a markdown table (and a W&B table)."""
    by_id = {}
    for row in rows:
        by_id.setdefault(row['id'], []).append(row)

    def mean(rs, key):
        vals = [r[key] for r in rs if r.get(key) is not None]
        return round(sum(vals) / len(vals), 2) if vals else None

    cols = ['experiment', 'reps', 'succeeded', 'time_s', 'distance_m', 'min_gap_m', 'mean_gap_m',
            'mean_speed_mps', 'recoveries']
    table = [[eid, len(rs), sum(bool(r['succeeded']) for r in rs), mean(rs, 'time_s'), mean(rs, 'distance_m'),
              min((r['min_gap_m'] for r in rs if r.get('min_gap_m') is not None), default=None),
              mean(rs, 'mean_gap_m'), mean(rs, 'mean_speed_mps'), mean(rs, 'recoveries')]
             for eid, rs in by_id.items()]
    lines = ['| ' + ' | '.join(cols) + ' |', '|' + ' --- |' * len(cols)]
    lines += ['| ' + ' | '.join('' if v is None else str(v) for v in row) + ' |' for row in table]
    surprise = [f"- {r['id']} r{r['rep']}: {json.dumps(r['surprise'])}" for r in rows if r.get('surprise')]
    text = '\n'.join(lines + ([''] + surprise if surprise else []))
    (OUT / 'summary.md').write_text(text + '\n')
    (OUT / 'runs.json').write_text(json.dumps(rows, indent=1))
    print('\n' + text, flush=True)

    if USE_WANDB:
        import wandb
        with wandb.init(project=WANDB_PROJECT, group='summary', job_type='summary', tags=['summary', *extra_tags],
                        name=f'summary-{time.strftime("%Y%m%d-%H%M%S")}', reinit='finish_previous') as wb:
            wb.log({'comparison': wandb.Table(columns=cols, data=table)})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--only', help='comma-separated experiment ids (default: all)')
    parser.add_argument('--reps', type=int, default=2)
    parser.add_argument('--tag', action='append', default=[], help='extra W&B tag (e.g. smoke-test)')
    args = parser.parse_args()

    selected = [e for e in EXPERIMENTS if not args.only or e.id in args.only.split(',')]
    if not selected:
        sys.exit(f'no experiment matches --only {args.only}; ids: {[e.id for e in EXPERIMENTS]}')
    OUT.mkdir(parents=True, exist_ok=True)
    if stale := running_sims():
        sys.exit(f'A simulation is already running (pids {stale}); stop it first.')

    rows = []
    for exp in selected:
        reps = 1 if exp.script == 'waypoints.py' else args.reps
        rows += [run_one(exp, rep, args.tag) for rep in range(1, reps + 1)]
    write_summary(rows, args.tag)
    return 0 if all(r['succeeded'] for r in rows) else 1


if __name__ == '__main__':
    sys.exit(main())
