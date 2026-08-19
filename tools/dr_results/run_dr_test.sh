#!/usr/bin/env bash
# One dead-reckoning run.
# usage: run_dr_test.sh <label> <true|false vslam> <outdir> [eval_sim_seconds] [laps] [domain_id]
set +u
LABEL="$1"; VSLAM="$2"; OUT="$3"; EVAL_S="${4:-70}"; LAPS="${5:-1}"; DOMAIN="${6:-77}"
# Resolve paths relative to this script so the harness works from any checkout.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
mkdir -p "$OUT"
LOG="$OUT/$LABEL.log"
: > "$LOG"

source /opt/ros/humble/setup.bash
source $REPO/qpl_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$REPO/qpl_ws/src/qpl_rover/worlds:${GAZEBO_MODEL_PATH}
# A distinct DDS domain per run, so a straggler from the previous run can never
# be mistaken for this run's publisher during the readiness waits.
export ROS_DOMAIN_ID=$DOMAIN

PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT -"$p" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -9 -"$p" 2>/dev/null; done
  for pat in gzserver gzclient rgbd_odometry ekf_node twist_mux \
             robot_state_publisher topic_tools dr_eval.py dr_drive.py \
             'ros2 launch qpl_rover' spawner drum_command_interface; do
    pkill -9 -f "$pat" 2>/dev/null
  done
  sleep 3
}
trap cleanup EXIT

launch() { setsid ros2 launch "$@" >>"$LOG" 2>&1 & PIDS+=($!); }

# wait_topic <topic> <timeout_s> — deadline measured against the WALL CLOCK.
# The previous version counted loop iterations assuming each cost 3 s, but
# `ros2 topic echo --once` fails in ~0.2 s when the topic does not exist yet, so
# a 90 s budget was really ~6 s and any slow-starting node aborted the run.
wait_topic() {
  local topic="$1"
  local timeout="$2"
  # Separate statements on purpose: in a single `local a=.. b=$((a))`, bash
  # evaluates every right-hand side before the earlier names are assigned, so
  # the arithmetic sees an empty `timeout` and the deadline comes out as 0.
  local deadline=$((SECONDS + timeout))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if timeout 5 ros2 topic echo --once "$topic" >/dev/null 2>&1; then
      echo "  ready: $topic ($((SECONDS)) s)" | tee -a "$LOG"
      return 0
    fi
    sleep 2
  done
  echo "TIMEOUT waiting for $topic after ${timeout}s" | tee -a "$LOG"
  return 1
}

echo "=== RUN $LABEL vslam=$VSLAM laps=$LAPS eval=${EVAL_S}s domain=$DOMAIN ===" | tee -a "$LOG"

# run_components:=false, so map_localisation — the AprilTag detector and the
# global EKF that consumes /apriltag/pose — is never started. The tag pipeline
# is absent, not merely ignored.
launch qpl_rover sim.launch.py headless:=true run_components:=false
wait_topic /gazebo/model_states 120 || exit 1
wait_topic /clock 60 || exit 1

launch qpl_rover controllers.launch.py use_sim_time:=true
wait_topic /diff_cont/odom 120 || exit 1

launch qpl_rover odom_localisation.launch.py use_sim_time:=true
wait_topic /odometry/filtered 120 || exit 1

if [ "$VSLAM" = "true" ]; then
  launch qpl_rover vslam_launch.py use_sim_time:=true
  wait_topic /vo/odom 150 || exit 1
fi

echo "--- all up, starting eval ---" | tee -a "$LOG"
WALL0=$SECONDS
setsid python3 $REPO/tools/dr_eval.py --duration "$EVAL_S" \
   --csv "$OUT/$LABEL.csv" --json "$OUT/$LABEL.json" --label "$LABEL" \
   >>"$LOG" 2>&1 & EVAL_PID=$!; PIDS+=($EVAL_PID)
sleep 2
setsid python3 $REPO/tools/dr_drive.py --laps "$LAPS" >>"$LOG" 2>&1 & DRIVE_PID=$!; PIDS+=($DRIVE_PID)

wait $EVAL_PID
WALL=$((SECONDS - WALL0))
echo "--- eval done in ${WALL}s wall for ${EVAL_S}s sim (RTF ~$(python3 -c "print(f'{$EVAL_S/max($WALL,1):.2f}')")) ---" | tee -a "$LOG"
kill -INT -$DRIVE_PID 2>/dev/null

# A run only counts if the eval actually wrote its result.
[ -s "$OUT/$LABEL.json" ] || { echo "NO JSON PRODUCED" | tee -a "$LOG"; exit 1; }
exit 0
