#!/usr/bin/env bash
set -euo pipefail
PKG=/home/ll/gem_ws/src/gem_learned_mpc
CFG=${1:-$PKG/config/simulation_scenarios.yaml}
MODEL=${2:-}
source /opt/ros/noetic/setup.bash
source /home/ll/gem_ws/devel/setup.bash
for S in nominal left_offset right_offset heading_error combined high_speed solver_fault; do
  echo "===== Running $S ====="
  rosrun gem_learned_mpc run_scenario.py --config "$CFG" --scenario "$S" --model-json "$MODEL"
  python3 "$PKG/scripts/generate_acceptance_report.py" "$PKG/results/mpc/$S.csv" --config "$PKG/config/acceptance.yaml" --output-dir "$PKG/results/mpc/${S}_report" || true
  sleep 3
done
python3 "$PKG/scripts/summarize_scenarios.py" --results-root "$PKG/results/mpc"
