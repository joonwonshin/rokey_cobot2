#!/usr/bin/env bash
# Isaac ROS(nvblox) 소스를 release-3.2로 재현한다.
# 이 저장소는 isaac_ros-dev/를 커밋하지 않는다(.gitignore) — pull 후 이 스크립트로 복원한다.
#
#   사용법:  ./scripts/setup_isaac_ros.sh
#   경로 변경: ISAAC_ROS_WS=/원하는/경로 ./scripts/setup_isaac_ros.sh
#
# 버전 정책: release-4.x는 run_dev.sh가 Isaac ROS CLI로 이전되어 사라졌고 사실상 Jazzy 중심이다.
#           Humble을 유지해야 하므로 release-3.2로 고정한다.
set -euo pipefail

ISAAC_ROS_WS="${ISAAC_ROS_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/isaac_ros-dev}"
SRC="${ISAAC_ROS_WS}/src"

echo "==> ISAAC_ROS_WS = ${ISAAC_ROS_WS}"
mkdir -p "${SRC}"

clone_pinned() {  # $1=디렉토리명  $2=URL  $3...=추가 clone 옵션
    local dir="$1" url="$2"; shift 2
    if [ -d "${SRC}/${dir}/.git" ]; then
        echo "==> ${dir}: 이미 있음 — 건너뜀 (버전은 아래 확인 결과로 판단)"
        return
    fi
    echo "==> ${dir}: release-3.2 클론"
    git clone -b release-3.2 "$@" "${url}" "${SRC}/${dir}"
}

clone_pinned isaac_ros_common https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
clone_pinned isaac_ros_nvblox https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git --recursive

# RealSense 지원 도커 이미지 키
echo CONFIG_IMAGE_KEY=ros2_humble.realsense > "${SRC}/isaac_ros_common/scripts/.isaac_ros_common-config"

# realsense-ros는 클론하지 않는다 — apt ros-humble-realsense2-camera로 이미 설치돼 있다.
if ! dpkg -l ros-humble-realsense2-camera >/dev/null 2>&1; then
    echo "!! ros-humble-realsense2-camera 없음 → sudo apt install ros-humble-realsense2-camera" >&2
fi

# 검증: run_dev.sh가 있어야 release-3.2가 맞다 (4.x에는 없다)
echo "==> 확인"
test -f "${SRC}/isaac_ros_common/scripts/run_dev.sh" \
    && echo "   run_dev.sh: OK" \
    || { echo "   run_dev.sh 없음 — 4.x가 클론된 것. ${SRC}/isaac_ros_common 지우고 다시 실행" >&2; exit 1; }
echo -n "   nvblox tag: "; git -C "${SRC}/isaac_ros_nvblox" describe --tags

cat <<EOF

다음 단계 (GPU PC에서만):
  nvidia-smi                       # GPU 확인
  docker info | grep -i runtime    # nvidia 런타임 확인
  cd ${SRC}/isaac_ros_common && ./scripts/run_dev.sh ${ISAAC_ROS_WS}
EOF
