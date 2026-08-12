#!/usr/bin/env bash
# 이 저장소에 안 들어간 외부 의존물을 받아온다. 새 PC 에서 **한 번** 실행한다.
#
#   ./scripts/fetch_externals.sh
#
# 왜 저장소에 안 넣었나 — 전부 남의 코드이고 합쳐서 20GB 가 넘는다.
#   GraspGenX      11G  (그 중 .venv 6.6G · ext 3.8G 는 uv 가 다시 만든다)
#   isaac_ros_*    4.8G
#   doosan-robot2  273M
# 우리가 쓴 코드는 40MB 뿐이라, 받아오는 쪽이 훨씬 싸다.
#
# 🔴 GPU·CUDA 가 있는 PC 여야 한다. GraspGenX 와 cuMotion 은 CPU 로 안 돈다.

set -uo pipefail
cd "$(dirname "$0")/.."
WS="$PWD"

say()  { printf '\n\033[1;36m== %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m!! %s\033[0m\n' "$*" >&2; }

clone_if_absent() {           # <경로> <URL> [브랜치]
  local dest="$1" url="$2" branch="${3:-}"
  if [ -d "$dest/.git" ]; then echo "  이미 있음  $dest"; return 0; fi
  mkdir -p "$(dirname "$dest")"
  if [ -n "$branch" ]; then git clone --depth 1 -b "$branch" "$url" "$dest"
  else                        git clone --depth 1 "$url" "$dest"; fi
}

# ── 1. 로봇 드라이버 ─────────────────────────────────────────────────────
# ⚠️ 개발 당시에는 팀 개인 저장소 안에 들어 있던 것을 썼다. 아래는 공개 upstream 이라
#    **버전이 다르면 서비스 이름·메시지(dsr_msgs2)가 갈릴 수 있다.**
say "1/3  로봇 드라이버 (Doosan M0609 · OnRobot RG2)"
clone_if_absent "$WS/src/cobot_rg2/doosan-robot2" \
  "https://github.com/DoosanRobotics/doosan-robot2.git" "humble"
clone_if_absent "$WS/src/cobot_rg2/onrobot-ros2" \
  "https://github.com/PickNikRobotics/onrobot-ros2.git" \
  || warn "onrobot-ros2 실패 — 그리퍼 없이도 MoveIt 검증까지는 된다"

# ── 2. GraspGenX ────────────────────────────────────────────────────────
say "2/3  GraspGenX (NVlabs)"
clone_if_absent "$WS/isaac_ros-dev/src/GraspGenX" \
  "https://github.com/NVlabs/GraspGenX.git"
echo "  의존성은 GraspGenX 자체 절차(uv sync)를 따른다."
echo "  그리퍼 메시(assets/, 158M)도 그 저장소가 갖고 있다."

# ── 3. Isaac ROS ────────────────────────────────────────────────────────
# 호스트에 직접 설치하지 않는다 — isaac_ros_common 의 run_dev.sh 가 컨테이너를 띄우고
# 그 안에서 빌드한다.
say "3/3  Isaac ROS (cuMotion · nvblox)"
for repo in isaac_ros_common isaac_ros_cumotion isaac_ros_nvblox isaac_ros_nitros; do
  clone_if_absent "$WS/isaac_ros-dev/src/$repo" \
    "https://github.com/NVIDIA-ISAAC-ROS/$repo.git"
done

# ── 확인 ────────────────────────────────────────────────────────────────
say "확인"
missing=0
for p in src/cobot_rg2/doosan-robot2 isaac_ros-dev/src/GraspGenX \
         isaac_ros-dev/src/isaac_ros_common isaac_ros-dev/src/isaac_ros_cumotion; do
  if [ -d "$WS/$p" ]; then echo "  OK    $p"; else echo "  없음  $p"; missing=1; fi
done
[ "$missing" = 1 ] && { warn "빠진 게 있다. 손으로 채운 뒤 다시 실행할 것."; exit 1; }
say "완료 — 다음은 README 의 '설치 ③ 컨테이너'"
