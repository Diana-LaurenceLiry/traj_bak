#!/usr/bin/env bash
set -euo pipefail

# Batch runner:
# 1) launch simulation (auto patrol on)
# 2) record a bag
# 3) stop when route completes (or timeout)
# 4) parse bag metrics
# 5) write a single summary report for all runs

RUNS="${RUNS:-10}"
TIMEOUT_SEC="${TIMEOUT_SEC:-240}"
TAIL_SEC="${TAIL_SEC:-4}"
STARTUP_SEC="${STARTUP_SEC:-8}"
COOLDOWN_SEC="${COOLDOWN_SEC:-3}"
CLEANUP_WAIT_SEC="${CLEANUP_WAIT_SEC:-20}"
METHOD="${METHOD:-global_local}"
PREFIX="${PREFIX:-g1_l1_batch}"
BAG_DIR="${BAG_DIR:-/home/lry/catkin_ws/exp_bags}"
OUT_DIR="${OUT_DIR:-/home/lry/catkin_ws/logs/bag_metrics}"
LAUNCH_PKG="${LAUNCH_PKG:-my_gazebo_sim}"
LAUNCH_FILE="${LAUNCH_FILE:-ego_orb_gazebo_bridge.launch}"
LAUNCH_ARGS="${LAUNCH_ARGS:-use_auto_patrol:=true}"

TOPICS=(
  /clock
  /move_base_simple/goal
  /planner/ego_goal
  /planning/pos_cmd
  /planning/bspline
  /visual_slam/odom_aligned
  /gazebo/model_states
  /rosout_agg
)

mkdir -p "${BAG_DIR}" "${OUT_DIR}"
TS="$(date +%Y%m%d_%H%M%S)"
SUMMARY_TXT="${OUT_DIR}/batch_${TS}_summary.txt"

cat > "${SUMMARY_TXT}" <<EOF
Batch started: $(date '+%F %T')
runs=${RUNS}
timeout_sec=${TIMEOUT_SEC}
tail_sec=${TAIL_SEC}
startup_sec=${STARTUP_SEC}
method=${METHOD}
prefix=${PREFIX}
bag_dir=${BAG_DIR}
output_dir=${OUT_DIR}
launch=${LAUNCH_PKG} ${LAUNCH_FILE} ${LAUNCH_ARGS}
EOF

echo "" >> "${SUMMARY_TXT}"
echo "==== Per-run results ====" >> "${SUMMARY_TXT}"

stop_proc() {
  local pid="$1"
  if [[ -z "${pid}" ]]; then
    return 0
  fi
  if kill -0 "${pid}" 2>/dev/null; then
    kill -INT "${pid}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      if ! kill -0 "${pid}" 2>/dev/null; then
        break
      fi
      sleep 0.2
    done
  fi
  if kill -0 "${pid}" 2>/dev/null; then
    kill -TERM "${pid}" 2>/dev/null || true
  fi
}

any_leftover_alive() {
  pgrep -f "roslaunch ${LAUNCH_PKG} ${LAUNCH_FILE}" >/dev/null 2>&1 && return 0
  pgrep -f "${BAG_DIR}/${PREFIX}_run.*\\.bag" >/dev/null 2>&1 && return 0
  pgrep -x gzserver >/dev/null 2>&1 && return 0
  pgrep -x gzclient >/dev/null 2>&1 && return 0
  pgrep -x gazebo >/dev/null 2>&1 && return 0
  return 1
}

cleanup_leftovers() {
  local stage="$1"
  local deadline=$(( $(date +%s) + CLEANUP_WAIT_SEC ))

  # Graceful stop first
  pkill -INT -f "roslaunch ${LAUNCH_PKG} ${LAUNCH_FILE}" >/dev/null 2>&1 || true
  pkill -INT -f "${BAG_DIR}/${PREFIX}_run.*\\.bag" >/dev/null 2>&1 || true
  pkill -INT -x gzserver >/dev/null 2>&1 || true
  pkill -INT -x gzclient >/dev/null 2>&1 || true
  pkill -INT -x gazebo >/dev/null 2>&1 || true

  sleep 1

  # Escalate if still alive
  if any_leftover_alive; then
    pkill -TERM -f "roslaunch ${LAUNCH_PKG} ${LAUNCH_FILE}" >/dev/null 2>&1 || true
    pkill -TERM -f "${BAG_DIR}/${PREFIX}_run.*\\.bag" >/dev/null 2>&1 || true
    pkill -TERM -x gzserver >/dev/null 2>&1 || true
    pkill -TERM -x gzclient >/dev/null 2>&1 || true
    pkill -TERM -x gazebo >/dev/null 2>&1 || true
  fi

  while any_leftover_alive; do
    if [[ "$(date +%s)" -ge "${deadline}" ]]; then
      pkill -KILL -f "roslaunch ${LAUNCH_PKG} ${LAUNCH_FILE}" >/dev/null 2>&1 || true
      pkill -KILL -f "${BAG_DIR}/${PREFIX}_run.*\\.bag" >/dev/null 2>&1 || true
      pkill -KILL -x gzserver >/dev/null 2>&1 || true
      pkill -KILL -x gzclient >/dev/null 2>&1 || true
      pkill -KILL -x gazebo >/dev/null 2>&1 || true
      break
    fi
    sleep 1
  done

  echo "[$(date '+%F %T')] cleanup(${stage}) done" >> "${SUMMARY_TXT}"
}

for i in $(seq 1 "${RUNS}"); do
  RUN_ID="$(printf "%02d" "${i}")"
  RUN_NAME="${PREFIX}_run${RUN_ID}"
  BAG_PATH="${BAG_DIR}/${RUN_NAME}.bag"
  LAUNCH_LOG="${OUT_DIR}/${RUN_NAME}_launch.log"
  BAG_LOG="${OUT_DIR}/${RUN_NAME}_rosbag.log"
  PARSE_LOG="${OUT_DIR}/${RUN_NAME}_parse.log"

  echo "[${i}/${RUNS}] Starting ${RUN_NAME}"
  cleanup_leftovers "pre_run_${RUN_NAME}"

  # Start launch
  bash -lc "source /opt/ros/noetic/setup.bash && source /home/lry/catkin_ws/devel/setup.bash && roslaunch ${LAUNCH_PKG} ${LAUNCH_FILE} ${LAUNCH_ARGS}" \
    > "${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID=$!

  sleep "${STARTUP_SEC}"

  # Start rosbag
  bash -lc "source /opt/ros/noetic/setup.bash && source /home/lry/catkin_ws/devel/setup.bash && rosbag record --lz4 -O '${BAG_PATH}' ${TOPICS[*]}" \
    > "${BAG_LOG}" 2>&1 &
  BAG_PID=$!

  START_EPOCH="$(date +%s)"
  REASON="timeout"

  while true; do
    NOW_EPOCH="$(date +%s)"
    ELAPSED="$((NOW_EPOCH - START_EPOCH))"

    if grep -q "auto_goal_patrol finished route" "${LAUNCH_LOG}" 2>/dev/null; then
      REASON="route_completed"
      break
    fi

    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      REASON="launch_exited"
      break
    fi

    if [[ "${ELAPSED}" -ge "${TIMEOUT_SEC}" ]]; then
      REASON="timeout"
      break
    fi

    sleep 1
  done

  sleep "${TAIL_SEC}"
  stop_proc "${BAG_PID}"
  stop_proc "${LAUNCH_PID}"
  cleanup_leftovers "post_run_${RUN_NAME}"

  # Parse bag
  bash -lc "source /opt/ros/noetic/setup.bash && source /home/lry/catkin_ws/devel/setup.bash && /usr/bin/python3 /home/lry/catkin_ws/src/my_gazebo_sim/scripts/bag_experiment_metrics.py --bag '${BAG_PATH}' --method '${METHOD}' --output_dir '${OUT_DIR}'" \
    > "${PARSE_LOG}" 2>&1 || true

  READABLE_PATH="$(ls -1t "${OUT_DIR}/${RUN_NAME}"_*_readable.txt 2>/dev/null | head -n 1 || true)"
  if [[ -n "${READABLE_PATH}" && -f "${READABLE_PATH}" ]]; then
    ONE_LINE="$(head -n 1 "${READABLE_PATH}")"
  else
    ONE_LINE="解析失败（未找到 readable.txt）"
  fi

  {
    echo ""
    echo "${RUN_NAME}: stop_reason=${REASON}"
    echo "${ONE_LINE}"
    echo "bag=${BAG_PATH}"
    echo "readable=${READABLE_PATH}"
  } >> "${SUMMARY_TXT}"

  echo "[${i}/${RUNS}] Done ${RUN_NAME} (${REASON})"
  sleep "${COOLDOWN_SEC}"
done

{
  echo ""
  echo "Batch finished: $(date '+%F %T')"
} >> "${SUMMARY_TXT}"

echo ""
echo "All done. Summary:"
echo "${SUMMARY_TXT}"
