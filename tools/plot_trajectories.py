"""Draw the README figure: driven paths on the SLAM map, stock vs tuned params, and the surprise box.

  python3 tools/plot_trajectories.py [--logs logs/05_experiments] [--out docs/trajectories.png]
"""
import argparse
import json
from pathlib import Path

import matplotlib
import yaml
from PIL import Image

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402  (backend must be chosen first)
from matplotlib.patches import Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
MAP_YAML = ROOT / 'src' / 'tb3_nav_demo' / 'maps' / 'my_world.yaml'
SURPRISE_BOX = (-0.3, -1.0, 0.4)  # centre x, y and side, as in run_trial.py


def load_xy(logs: Path, run: str):
    samples = json.loads((logs / run / 'trial.json').read_text())['samples']
    return [s[1] for s in samples], [s[2] for s in samples]


def draw_map(ax):
    meta = yaml.safe_load(MAP_YAML.read_text())
    img = Image.open(MAP_YAML.parent / meta['image'])
    x0, y0 = meta['origin'][:2]
    extent = [x0, x0 + img.width * meta['resolution'], y0, y0 + img.height * meta['resolution']]
    ax.imshow(img, cmap='gray', extent=extent, vmin=0, vmax=255)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--logs', type=Path, default=ROOT / 'logs' / '05_experiments')
    parser.add_argument('--out', type=Path, default=ROOT / 'docs' / 'trajectories.png')
    args = parser.parse_args()

    fig, (left, right) = plt.subplots(1, 2, figsize=(14, 5), constrained_layout=True)

    draw_map(left)
    for run, label, color in (('tune-stock-r1', 'stock inflation 0.70 / 3.0', '#d1495b'),
                              ('final-course-r1', 'tuned inflation 0.45 / 6.0', '#2e86ab')):
        left.plot(*load_xy(args.logs, run), color=color, lw=2, label=label)
    left.plot(-2.0, -0.5, 'k^', ms=9, label='spawn')
    left.set_title('Course: north gap, corridor, clutter, north-east corner')
    left.legend(loc='lower right', fontsize=8)

    draw_map(right)
    bx, by, side = SURPRISE_BOX
    right.add_patch(Rectangle((bx - side / 2, by - side / 2), side, side, color='orange', label='surprise box'))
    right.plot(*load_xy(args.logs, 'final-surprise-r1'), color='#2e86ab', lw=2, label='driven path (tuned)')
    right.plot(-2.0, -0.5, 'k^', ms=9, label='spawn')
    right.set_title('Box dropped into the corridor while driving: robot goes round it')
    right.legend(loc='lower right', fontsize=8)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=110)
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
