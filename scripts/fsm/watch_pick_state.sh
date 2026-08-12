#!/bin/bash
# 다음 pick 재현 시도 전에 먼저 백그라운드로 띄워둔다.
# /pick/state, /pick/robot_state_code, /pick/robot_state_text 를
# 타임스탬프와 함께 각각 파일로 남긴다 — RELEASE 직전에 정확히
# 어느 상태에서 멈췄는지, 그때 로봇 컨트롤러 코드가 뭐였는지 보려는 목적.
#
# 사용:
#   ./scripts/watch_pick_state.sh &
#   (pick 재현)
#   Ctrl-C 또는 kill 로 종료 후 /tmp/pick_watch_*.log 확인

set -eo pipefail
# ROS setup.bash 들은 자체적으로 -u 에 안 걸리게 짜여있지 않다(AMENT_TRACE_SETUP_FILES 등
# 미정의 변수 참조) — source 하는 동안만 -u 를 끈다.
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

OUTDIR=/tmp/pick_watch_$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUTDIR"
echo "로그 위치: $OUTDIR"

# moreutils `ts` 가 없는 환경이라 date 로 직접 타임스탬프를 붙인다.
stamp() { while IFS= read -r line; do echo "$(date '+%H:%M:%S.%3N') $line"; done; }

ros2 topic echo --full-length /pick/state --field data 2>/dev/null \
  | stamp >> "$OUTDIR/pick_state.log" &
ros2 topic echo --full-length /pick/robot_state_code --field data 2>/dev/null \
  | stamp >> "$OUTDIR/robot_state_code.log" &
ros2 topic echo --full-length /pick/robot_state_text --field data 2>/dev/null \
  | stamp >> "$OUTDIR/robot_state_text.log" &

wait
