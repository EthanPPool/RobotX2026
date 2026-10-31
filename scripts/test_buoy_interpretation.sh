#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$repo_dir/coordination_ws/src/robotx_buoy_interpretation:$repo_dir/coordination_ws/src/robotx_coordination${PYTHONPATH:+:$PYTHONPATH}"
python3 -m unittest discover -s "$repo_dir/tests/buoy_interpretation" -v
