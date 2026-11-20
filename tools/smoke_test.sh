#!/usr/bin/env bash
# Smoke test of the whole system, without windows: the simulation, the camera and its detector, the MTC task and
# project 1's success check, on the first trials of M4's draw (ADR 0004, seed 1). CI runs it in the Docker image; it
# runs the same from a sourced workspace, on its own ROS domain, so a simulation in another terminal stays apart.
# Exit code 0 only when every trial succeeds. About a minute.
#
#   tools/smoke_test.sh [trials]   # 2 by default; logs and rows in a temporary folder, printed at the end
TRIALS="${1:-2}"
[[ $TRIALS =~ ^[1-9][0-9]*$ ]] || { echo "usage: smoke_test.sh [trials], at least one trial"; exit 2; }
OUT="$(mktemp -d)"
export DATA_DIR="$OUT"   # run_trials writes its rows here, never into ~/data
export ROS_DOMAIN_ID=87  # apart from a simulation running elsewhere on this computer
set -u

# each in its own process group, so that one signal stops it whole
setsid ros2 launch so101_mtc_bringup sim.launch.py headless:=true rviz:=false > "$OUT/sim.log" 2>&1 < /dev/null &
SIM=$!
TASK=
stop() { kill -INT -- -"$SIM" ${TASK:+-"$TASK"} 2>/dev/null; sleep 5; kill -KILL -- -"$SIM" ${TASK:+-"$TASK"} 2>/dev/null; }
trap stop EXIT

for _ in $(seq 180); do
  grep -q "You can start planning now" "$OUT/sim.log" && break
  kill -0 "$SIM" 2>/dev/null || break  # the launch has stopped
  sleep 1
done
if ! grep -q "You can start planning now" "$OUT/sim.log"; then
  echo "FAIL: MoveIt is not ready"; tail -40 "$OUT/sim.log"; exit 1
fi
grep -h -E "EGL|GLFW" "$OUT/sim.log"  # how the cameras render: GLFW with a display, EGL without one

setsid ros2 launch so101_mtc_task pick_place.launch.py run_at_start:=false camera:=true > "$OUT/task.log" 2>&1 \
  < /dev/null &
TASK=$!

ros2 run so101_mtc_eval_ros run_trials --trials "$TRIALS" --seed 1 --out smoke 2>&1 | tee "$OUT/trials.log"
echo "logs: $OUT"
if grep -q "\"successes\": $TRIALS," "$OUT/trials.log"; then
  echo "PASS: $TRIALS of $TRIALS trials succeeded"
else
  echo "FAIL: not every trial succeeded"
  grep -h -E "EGL|GLFW|ERROR" "$OUT/sim.log" | tail -20
  tail -20 "$OUT/task.log"
  exit 1
fi
