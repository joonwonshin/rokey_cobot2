#!/usr/bin/env bash
# 이 워크스페이스를 빌드한다.
#
#   ./scripts/build.sh              전부
#   ./scripts/build.sh fsm          로봇 쪽만 (pick_fsm · voice_processing · graspgenx)
#   ./scripts/build.sh vla          판단 쪽만 (vla_system · vla_interfaces)
#
# 🔴 왜 하나의 colcon build 가 아닌가
# ----------------------------------
# `vla_system` 은 openai·torch·ultralytics 가 필요하고 그것들은 **.venv 안에만** 있다
# (~/.local 에 넣으면 apt pytest 와 충돌해 전 패키지 테스트가 깨진 이력이 있다).
# 그런데 apt 로 깔린 /usr/bin/colcon 은 venv 를 활성화해도 **항상 /usr/bin/python3 로**
# 돌아서, 생성되는 console_scripts 셰뱅에 /usr/bin/python3 가 박힌다. 그러면 노드가
# 런타임에 `ModuleNotFoundError: torch` 로 죽는다 (2026-08-10 재현).
#
# 그래서 vla 쪽만 **venv 를 활성화한 뒤 `python3 -m colcon`** 으로 빌드한다.
# 확인: head -1 install/vla_system/lib/vla_system/agent_node → .venv 를 가리켜야 한다.

set -uo pipefail
cd "$(dirname "$0")/.."
WS="$PWD"

TARGET="${1:-all}"

FSM_PKGS=(pick_fsm pick_fsm_msgs voice_processing graspgenx_perception cumotion object_detection)
VLA_PKGS=(vla_interfaces vla_system)

say() { printf '\n\033[1;36m== %s\033[0m\n' "$*"; }
die() { printf '\033[1;31m!! %s\033[0m\n' "$*" >&2; exit 1; }

[ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble 이 없다 (/opt/ros/humble)"
source /opt/ros/humble/setup.bash

# ── 로봇 쪽 ──────────────────────────────────────────────────────────────
if [[ "$TARGET" == "all" || "$TARGET" == "fsm" ]]; then
  say "FSM 쪽 빌드"
  # --symlink-install: yaml/py 를 고칠 때마다 재빌드하지 않기 위해서다. 다만
  # ament_python 패키지의 share 는 build/ 를 가리키므로 **yaml 은 여전히 재빌드가
  # 필요하다** (.py 는 반영돼서 착각하기 쉽다).
  colcon build --symlink-install --packages-select "${FSM_PKGS[@]}" \
    || die "FSM 빌드 실패"
fi

# ── 판단 쪽 ──────────────────────────────────────────────────────────────
if [[ "$TARGET" == "all" || "$TARGET" == "vla" ]]; then
  say "VLA 쪽 빌드 (.venv 로)"
  if [[ -f "$WS/.venv/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "$WS/.venv/bin/activate"
  else
    die ".venv 가 없다. 먼저:
    python3 -m venv --system-site-packages .venv
    source .venv/bin/activate && pip install -r requirements-vla.txt"
  fi
  python3 -m colcon build --symlink-install --packages-select "${VLA_PKGS[@]}" \
    || die "VLA 빌드 실패"

  SHEBANG=$(head -1 "$WS/install/vla_system/lib/vla_system/agent_node" 2>/dev/null || true)
  if [[ "$SHEBANG" != *".venv"* ]]; then
    die "셰뱅이 .venv 를 안 가리킨다: $SHEBANG
    노드가 런타임에 ModuleNotFoundError 로 죽는다. venv 를 켠 채 다시 빌드할 것."
  fi
  echo "  셰뱅 확인: $SHEBANG"
  deactivate 2>/dev/null || true
fi

say "완료 — source install/setup.bash"
