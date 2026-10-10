#!/bin/bash
# The "item 2" sitting (2026-10-09): settle the constellation estimator's yaw-offset
# gauge with ~40 CORRECTED throws, then the affine's validity in that frame.
# Prerequisites: the sweep-estimator branch merged and built in the live stack; the
# stack launched; BB calibrated by its sweep (mocap_node logs the method, fitted
# latency and gate verdict); hopper loaded; QTM continuous capture running.
# Usage: [PLAN=<plan.json>] ./run_item2_sitting.sh [N_THROWS]      (defaults: local_validation_plan.json, 40 throws, DX DY 0.3 -0.6)
set -euo pipefail
N=${1:-40}
PLAN=${PLAN:-local_validation_plan.json}   # e.g. PLAN=local_validation_plan_range.json for the range-weighted pin plan
CAND=${CAND:-$HOME/bb_calibration_sessions/20261009T002142_931068Z/analysis/correction_candidate.json}
SOLVER_SHA=${SOLVER_SHA:-cb09095e}
DXDY=${DXDY:-"0.3 -0.6"}
WS=${WS:-$HOME/Desktop/Jugglebot-skills/ros_ws/install/setup.bash}
VENV=$HOME/Desktop/PDJ_venv/venv/bin/activate
cd "$(dirname "$0")"
# shellcheck disable=SC1090
# ROS/colcon setup scripts read unset vars (COLCON_TRACE etc.), so drop -u while sourcing
set +u; source "$VENV"; source "$WS"; set -u
LOG=$(mktemp /tmp/item2_sitting_XXXX.log)
echo "== 1/4 preflight (nothing moves): solver sha, candidate, $N feasible throws"
python run_local_calibration.py run $PLAN --schedule-to-mocap $DXDY --expect-solver-sha "$SOLVER_SHA" \
    --apply-correction "$CAND" --limit "$N" --check-only
echo
echo "Check the mocap_node calibration line (method, fitted latency, gate), the hopper and the QTM capture."
read -r -p "Throw $N corrected throws now? [Enter = go, Ctrl-C = abort] "
echo "== 2/4 throwing"
python run_local_calibration.py run $PLAN --schedule-to-mocap $DXDY --expect-solver-sha "$SOLVER_SHA" \
    --apply-correction "$CAND" --limit "$N" | tee "$LOG"
OUT=$(grep -o 'Output: .*' "$LOG" | tail -1 | sed 's/^Output: //')
[ -d "$OUT" ] && OUT="$OUT/session.json"
echo "== 3/4 extracting landings: $OUT"
python analyze_local_calibration.py "$OUT" --extract-only
echo "== 4/4 settling the gauge (pre-registered rule: |mean bearing| <= 0.15 deg confirms the frame)"
/usr/bin/python3 settle_yaw_gauge.py "$OUT"
