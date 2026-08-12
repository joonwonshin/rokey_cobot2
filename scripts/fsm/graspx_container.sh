#!/usr/bin/env bash
# 컨테이너(od_kimkh) 안에서 graspx 파이프라인을 띄운다. 인자는 launch 로 그대로 넘어간다.
#
#   scripts/graspx_container.sh run_bridge:=false device:=0 publish_overlay:=true
#   탐지 대상은 ws 루트 `config/objects.yaml` 의 detect 가 정본이다 (아래 COBOT2_OBJECTS 로
#   넘긴다). 그 파일을 고쳤으면 이 스크립트를 다시 실행해야 반영된다.
#   scripts/graspx_container.sh --no-clean ...   # 기존 인스턴스를 죽이지 않는다
#
# 왜 스크립트인가 — `docker exec` 에는 `--sig-proxy` 가 없다 (docker 29.7.0 확인).
# `-t` 없이 띄우면 컨테이너 안에 제어 터미널이 없어서 호스트 Ctrl-C 는 호스트 쪽 docker
# 클라이언트만 죽이고, 컨테이너 안 `ros2 launch` 는 containerd-shim 밑에 고아로 남는다.
# README 가 2026-08-08 까지 "-it 없이 띄우면 Ctrl-C 로 끝낼 수 있다"고 정반대로 적어 뒀고,
# 그 결과 yolo_seg_node 가 재실행마다 1개씩 쌓여 10개까지 갔다 (실측).
#
# 동시성 정책 — **"먼저 뜬 것이 이긴다"가 정본이다.** 이 스크립트의 기동 전 정리는 그
# 정책의 예외가 아니라, "내가 조금 전에 남긴 고아를 치운다"는 좁은 용도다. 그래서
#   - 정리는 **기동 전에 한 번만** 한다. 종료 시(EXIT)에는 하지 않는다.
#     EXIT 에서 정리하면: 터미널 B 가 뜨면서 A 를 죽임 → A 의 docker exec 이 반환 →
#     A 의 EXIT 트랩이 **방금 뜬 B 를 죽인다.** 실제로 성립하는 시나리오다.
#   - 무엇을 죽였는지 반드시 출력한다. 조용히 죽이면 남의 실행을 지운 걸 아무도 모른다.
#   - Ctrl-C(INT/TERM/HUP)로 끝날 때만 뒷정리한다 — `-t` 가 시그널을 못 넘긴 경우의 보험.
# 마지막 방어선은 yolo_seg_node 자신의 flock 이다 (yolo_seg_node.py:acquire_singleton).
#
# `set -e` 는 일부러 안 쓴다. 정리(pkill)는 대상이 없을 때 exit 1 이 정상이고, 트랩 안에서
# 죽으면 뒷정리가 반쯤 돌다 만다. 실패를 봐야 하는 지점은 아래에서 개별로 확인한다.
set -uo pipefail

CONTAINER=${GRASPX_CONTAINER:-od_kimkh}
WS=${GRASPX_WS:-/home/kimkh/cobot2_ws_new}
# 정리 대상 범위. `graspgenx_perception` 은 이 컨테이너의 graspx 노드 전부(yolo_seg_node,
# 컨테이너에서 띄운 grasp_bridge_node, ros2 launch 부모)를 잡는다. 호스트에서 도는
# grasp_bridge_node 는 docker exec 범위 밖이라 영향이 없다. 컨테이너를 남과 공유하게 되면
# 이 패턴을 `yolo_seg_node` 로 좁힐 것.
KILL_PATTERN=${GRASPX_KILL_PATTERN:-graspgenx_perception}

CLEAN=1
[ "${1:-}" = '--no-clean' ] && { CLEAN=0; shift; }

# pkill 후 프로세스가 실제로 사라질 때까지 기다린다. 안 기다리면 구 인스턴스가 flock 을
# 아직 쥔 채인 동안 새 노드가 락을 잡으려다 "이미 돌고 있다"로 죽는다.
kill_stale() {
    local left
    left=$(docker exec "$CONTAINER" pgrep -f "$KILL_PATTERN" 2>/dev/null | tr '\n' ' ')
    [ -z "${left// /}" ] && return 0
    echo "[graspx] 컨테이너 안 잔존 프로세스를 정리한다 (PID: ${left%% })" >&2
    docker exec "$CONTAINER" pkill -f "$KILL_PATTERN" >/dev/null 2>&1
    for _ in 1 2 3 4 5 6 7 8 9 10; do
        docker exec "$CONTAINER" pgrep -f "$KILL_PATTERN" >/dev/null 2>&1 || return 0
        sleep 1
    done
    echo "[graspx] SIGTERM 10초 후에도 남아 있다 — SIGKILL 한다" >&2
    docker exec "$CONTAINER" pkill -9 -f "$KILL_PATTERN" >/dev/null 2>&1
    sleep 1
}

docker start "$CONTAINER" >/dev/null || exit 1
[ "$CLEAN" = 1 ] && kill_stale

# Ctrl-C 로 끝날 때만 뒷정리한다. 트랩을 해제하고 같은 시그널로 자살해, 호출자가
# "중단됨"과 "정상 종료"를 종료코드로 구분할 수 있게 한다.
on_signal() { trap - INT TERM HUP; kill_stale; kill -INT $$; }
trap on_signal INT TERM HUP

# 호출자에 tty 가 없으면(스크립트·CI) `-t` 는 "input device is not a TTY" 로 실패한다.
TTY_FLAGS=(-i)
[ -t 0 ] && TTY_FLAGS=(-i -t)

# printf %q 로 감싼다. classes:='[46,47]' 처럼 원격 bash -lc 문자열 안에서 다시 파싱되는
# 인자가 있어서, 따옴표 없이 넘기면 대괄호가 글롭으로 먹힌다.
# 인자가 0개일 때 `printf '%q ' ` 는 빈 문자열이 아니라 `''` 를 출력하고, 그러면 원격에서
# `ros2 launch ... graspx.launch.py ''` 가 되어 "malformed launch argument ''" 로 죽는다.
ARGS=''
[ $# -gt 0 ] && ARGS=$(printf '%q ' "$@")

# 탐지 대상의 정본. 컨테이너가 ws 를 같은 절대경로로 마운트하므로 호스트와 같은 파일이다
# (2026-08-09 `docker exec ls` 로 확인). 넘겨주지 않으면 컨테이너 안 런치가 자기 share 에서
# ws 루트를 되짚어야 하는데, 컨테이너 안의 install 배치가 다를 수 있어 여기서 못 박는다.
docker exec "${TTY_FLAGS[@]}" \
  -e FASTRTPS_DEFAULT_PROFILES_FILE="$WS/fastdds_udp_only.xml" \
  -e COBOT2_OBJECTS="${COBOT2_OBJECTS:-$WS/config/objects.yaml}" \
  "$CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash
    source '$WS/install/setup.bash'
    exec ros2 launch graspgenx_perception graspx.launch.py $ARGS"
