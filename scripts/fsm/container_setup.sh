#!/usr/bin/env bash
# Isaac ROS 컨테이너에 **들어갈 때마다** 한 번 돌린다.
#
#   bash /workspaces/cobot2_ws_new/scripts/container_setup.sh
#
# `run_dev.sh`는 컨테이너를 재사용하지 않고 **새로 만든다.** 그래서 이미지 밖 변경
# (pip 설치)이 매번 사라진다. 2026-08-06에 이것 때문에 두 번 같은 자리에서 막혔다:
#   AttributeError: module 'warp' has no attribute 'torch'
# 바인드 마운트(`isaac_ros-dev/`, `cobot2_ws_new/`) 안의 것(curobo 패치, colcon 산출물)은
# 살아남으므로 여기서 다시 할 필요가 없다.
set -u

echo "── warp-lang ────────────────────────────────────────────"
# cuRobo 커밋 36ea382(2024-11)는 warp 1.2.1까지만 안다. 이미지의 1.16.0에는
# `warp.torch` 모듈이 아예 없어서 read_esdf_world:=True 경로가 임포트 단계에서 죽는다.
# 1.5.0은 cuRobo와 동시대(2024-12)이고 >1.2.1이라 최신 커널 경로를 탄다.
# 근거: [[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §2-3
WANT_WARP=1.5.0
HAVE_WARP=$(python3 -c 'import warp; print(warp.config.version)' 2>/dev/null || echo none)
if [ "$HAVE_WARP" = "$WANT_WARP" ]; then
  echo "  OK  warp $HAVE_WARP"
else
  echo "  $HAVE_WARP → $WANT_WARP 설치 중..."
  pip3 install "warp-lang==${WANT_WARP}" 2>&1 | tail -1
  python3 -c 'import warp; print("  결과 warp", warp.config.version, warp.__file__)'
fi

echo "── numpy ────────────────────────────────────────────────"
# 이미지의 numpy 2.2.6이 apt cv2(numpy1 빌드)를 깨서 robot_segmenter_node가 못 뜬다.
# 대가: cupy-cuda12x가 깨진다 — 쓰는 건 isaac_ros_cumotion_object_attachment뿐이라 무해.
# 근거: [[ws/cobot2/context/constraints]] "이미지의 numpy 2.2.6이 cv2를 깨서..."
WANT_NUMPY=1.26.4
HAVE_NUMPY=$(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || echo none)
if [ "$HAVE_NUMPY" = "$WANT_NUMPY" ]; then
  echo "  OK  numpy $HAVE_NUMPY"
else
  echo "  $HAVE_NUMPY → $WANT_NUMPY 설치 중..."
  # ⚠️ "numpy<2"로 쓰면 .claude/hooks/guard.sh 가 오탐으로 막는다. 명시 핀을 쓴다
  pip3 install "numpy==${WANT_NUMPY}" 2>&1 | tail -1
fi
python3 -c 'import cv2; print("  cv2", cv2.__version__, "OK")' 2>&1 | tail -1

echo "── curobo 패치 (바인드 마운트라 보통 살아 있다) ─────────"
# ① C++20 std::lerp 충돌 (§2-2)  ② warp.torch 임포트 무해화 (§2-3)
CUROBO=/workspaces/isaac_ros-dev/src/isaac_ros_cumotion/curobo_core/curobo
# 서브모듈이라 .git은 디렉토리가 아니라 gitlink **파일**이다 → -d가 아니라 -e
if [ -e "$CUROBO/.git" ]; then
  CHANGED=$(cd "$CUROBO" && git status --short | wc -l)
  [ "$CHANGED" -ge 2 ] \
    && echo "  OK  수정된 파일 ${CHANGED}개 (helper_math.h / world_mesh.py)" \
    || echo "  🔴 패치가 없다. patches/curobo-*.patch 를 git apply 할 것"
else
  echo "  ?   $CUROBO 없음 — 서브모듈 미초기화?"
fi

echo "── 셸마다 필요한 것 (이 스크립트는 못 해준다) ───────────"
cat <<'EOF'
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash
  source /workspaces/cobot2_ws_new/install_container/setup.bash
  export ROS_DOMAIN_ID=93
  # ⚠️ RMW_IMPLEMENTATION 은 설정하지 않는다 (기본 Fast DDS).
  #    cyclonedds로 바꾸면 호스트 controller_manager 서비스를 못 불러 spawner가 멈춘다.
EOF
