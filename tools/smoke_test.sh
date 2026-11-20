#!/usr/bin/env bash
# Smoke test of the whole system, without windows: the simulation, the camera and its detector, the MTC task and
# project 1's success check, on the first trials of M4's draw (ADR 0004, seed 1). CI runs it in the Docker image; it
# runs the same from a sourced workspace. Exit code 0 only when every trial succeeds. About 2 minutes.
#
#   tools/smoke_test.sh [trials]   # 2 by default; logs and rows in a temporary folder, printed at the end
TRIALS="${1:-2}"
OUT="$(mktemp -d)"
export DATA_DIR="$OUT"  # run_trials writes its rows here, never into ~/data
set -u

# each in its own process group, so that one signal stops it whole
setsid ros2 launch so101_mtc_bringup sim.launch.py headless:=true rviz:=false > "$OUT/sim.log" 2>&1 < /dev/null &
SIM=$!
stop() { kill -INT -- -"$SIM" ${TASK:+-"$TASK"} 2>/dev/null; sleep 5; kill -KILL -- -"$SIM" ${TASK:+-"$TASK"} 2>/dev/null; }
trap stop EXIT

for _ in $(seq 180); do grep -q "You can start planning now" "$OUT/sim.log" && break; sleep 1; done
if ! grep -q "You can start planning now" "$OUT/sim.log"; then
  echo "FAIL: MoveIt not ready within 180 s"; tail -40 "$OUT/sim.log"; exit 1
fi
grep -h -E "EGL|GLFW|camera" "$OUT/sim.log" | head -5  # how the cameras render here

setsid ros2 launch so101_mtc_task pick_place.launch.py run_at_start:=false camera:=true > "$OUT/task.log" 2>&1 \
  < /dev/null &
TASK=$!

ros2 run so101_mtc_eval_ros run_trials --trials "$TRIALS" --seed 1 --out smoke 2>&1 | tee "$OUT/trials.log"
echo "logs: $OUT"
if grep -q "\"successes\": $TRIALS," "$OUT/trials.log"; then
  echo "PASS: $TRIALS of $TRIALS trials succeeded"
else
  echo "FAIL: not every trial succeeded"; tail -20 "$OUT/task.log"; exit 1
fi
