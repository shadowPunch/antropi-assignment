#!/usr/bin/env bash
# Step 05 entry point: ROS env + the project venv (which adds wandb), then the experiment runner.
#   bash tools/05_experiments.sh [--only id1,id2] [--reps N]
source "$(dirname "$0")/ros_env.sh"
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
if [[ ${TB3_WANDB:-1} != 0 && ! -x $root/.venv/bin/python ]]; then
  echo "W&B logging needs the venv: uv venv --python /usr/bin/python3.14 --system-site-packages .venv && uv pip install --python .venv/bin/python wandb" >&2
  exit 1
fi
python="$root/.venv/bin/python"; [[ -x $python ]] || python=python3
exec "$python" -u "$root/tools/05_experiments.py" "$@"
