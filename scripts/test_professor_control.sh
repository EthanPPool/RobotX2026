#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$repo_dir/coordination_ws/src/robotx_professor_control:$repo_dir/coordination_ws/src/robotx_coordination:$repo_dir/boat_ws/src/boat_vehicle${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
python3 -m unittest discover -s "$repo_dir/tests/professor_control" -v
