"""상태 정의와 허용 전이표.

src/PACKAGES.md#pick_fsm §1(상태) 의 stateDiagram 을 코드로 옮긴 것이다.
(옛 설계 문서는 2026-08-08 ws-cleanup 으로 삭제됐다 — PACKAGES.md#pick_fsm 이 단일 출처.)
**전이표를 여기 한 곳에만 둔다** — task_manager 안에 흩뿌리면 다이어그램과 코드가 조용히 갈라진다.

문서와 다른 점 2가지 (의도적):
  1. `WAIT_APPROVAL` 을 별도 상태로 뺐다. 문서에서는 `PLAN --> APPROACH : 사용자 승인 ✋` 라는
     전이 라벨이었는데, 승인 대기는 **시간이 흐르는 구간**이라 상태가 아니면 관측이 안 된다
     (`/pick/state` 를 보고 "지금 사람을 기다리는 중"인지 알 수 없다).
  2. 문서의 `APPROACH` 안쪽 서브상태(EXECUTING/MONITOR/STOP_REPLAN/WAITING)는 상태로 만들지 않았다.
     move_group 이 `planning_options.replan` 으로 그 루프를 이미 돌린다 (README "STOP_REPLAN" 절).
     FSM 은 그 바깥의 재시도만 센다 — 같은 일을 두 겹으로 구현하면 서로 싸운다.
"""

from enum import Enum, auto


class State(Enum):
    IDLE = auto()            # 아무것도 안 함. /pick/start 를 기다린다
    LISTENING = auto()       # get_keyword 호출 중 (음성 경로)
    PERCEIVE = auto()        # ComputeGrasp 호출 중 (촬영→세그→GPU→선택은 전부 저쪽)
    SCENE_PREP = auto()      # 대상을 CollisionObject 로 등록 + ACM 조정
    PLAN = auto()            # pre-grasp / grasp / lift 3점 IK
    NEXT_CANDIDATE = auto()  # alternatives 에서 다음 후보 꺼내기 (GPU 재호출 없음)
    WAIT_APPROVAL = auto()   # ✋ 사람이 /pick/approve 를 부를 때까지 정지
    STOW = auto()            # APPROACH 이동 전 그리퍼를 닫는다 — 이동 중 폭을 줄여 주변과의 충돌을 피한다
    APPROACH = auto()        # pre-grasp 로 이동 (실행 구간, 그리퍼는 닫힌 채)
    REGRASP = auto()         # (스캐폴드) pre-grasp 도착 후 eye-in-hand 재-graspgenx + 승인
    OPEN_GRIPPER = auto()    # pre-grasp 도착 후, 하강 전에 그리퍼를 연다
    DESCEND = auto()         # grasp pose 로 하강
    CLOSE = auto()           # 그리퍼 닫기
    VERIFY = auto()          # 잡혔는지 확인
    RELEASE_RETRY = auto()   # 놓치면 열고 재시도
    LIFT = auto()            # 들어올리기 (물체 attach 된 상태)
    WAIT_PLACE_TARGET = auto()  # 들어올린 뒤 place 미지정 — 물체 문 채 set_place 대기
    PLACE = auto()           # 놓을 자세로 이동
    PLACE_RETRY = auto()     # 놓기 계획/이동 실패 — 물체 문 채 정지, 사람이 위치 바꿔 재시도
    RELEASE = auto()         # 그리퍼 열기 + detach
    HOME = auto()            # 홈 복귀
    PAUSED = auto()          # ✋ 사람이 "멈춰" — 되돌릴 수 있는 일시정지. 사람 명령으로만 나간다
    SPEAK_FAIL = auto()      # 실패 통보
    ABORT = auto()           # 중단 결정
    SAFE_STOP = auto()       # 정지 유지. /pick/reset 전까지 아무 명령도 안 낸다


#: 허용 전이. {현재 상태: {갈 수 있는 상태들}}
TRANSITIONS: dict[State, set[State]] = {
    # HOME: /pick/home(음성/VLA·rqt)이 IDLE 에서 홈 관절자세로 보낸다. 도착 후 _home_next
    # (=IDLE)로 돌아온다 — SAFE_STOP 복구의 HOME 경유와 같은 뼈대다(_srv_home 참고).
    State.IDLE:           {State.LISTENING, State.PERCEIVE, State.HOME},
    State.LISTENING:      {State.IDLE, State.PERCEIVE, State.SPEAK_FAIL, State.ABORT},
    State.PERCEIVE:       {State.SCENE_PREP, State.SPEAK_FAIL, State.ABORT},
    State.SCENE_PREP:     {State.PLAN, State.ABORT},
    State.PLAN:           {State.WAIT_APPROVAL, State.NEXT_CANDIDATE, State.ABORT},
    State.NEXT_CANDIDATE: {State.PLAN, State.SPEAK_FAIL, State.ABORT},
    State.WAIT_APPROVAL:  {State.STOW, State.ABORT},
    State.STOW:           {State.APPROACH, State.ABORT},
    # APPROACH -> REGRASP: regrasp_enabled=true 면 pre-grasp 도착 후 eye-in-hand 재파지
    # 인식 + 승인을 거친다(스캐폴드, 2026-08-11). false 면 지금처럼 곧장 OPEN_GRIPPER.
    State.APPROACH:       {State.OPEN_GRIPPER, State.REGRASP, State.NEXT_CANDIDATE, State.ABORT},
    State.REGRASP:        {State.OPEN_GRIPPER, State.ABORT},
    State.OPEN_GRIPPER:   {State.DESCEND, State.ABORT},
    State.DESCEND:        {State.CLOSE, State.NEXT_CANDIDATE, State.ABORT},
    State.CLOSE:          {State.VERIFY, State.ABORT},
    # VERIFY -> CLOSE: 파지 실패 시 놓았다 재인식(RELEASE_RETRY)하기 전에, 같은 자세에서
    # 더 좁게 한 번 더 닫아본다(grip_narrow_retries). 병처럼 기입된 폭(최대 단면)이 실제
    # 잡으려는 부위(목처럼 얇은 곳)보다 넓어 손가락이 접촉 전에 멈추는 경우를 구제한다.
    State.VERIFY:         {State.LIFT, State.CLOSE, State.RELEASE_RETRY, State.ABORT},
    # RELEASE_RETRY, SAFE_STOP 모두 곧장 PERCEIVE/IDLE 로 가지 않고 HOME 을 거친다 —
    # 팔이 작업공간 박스 안(물체 높이)에 남은 채 재촬영하면 그리퍼 자신이 물체로
    # 오인식된다 (2026-08-07 발견: capture_graspgenx_scene.py 의 세그멘테이션은 팔을
    # XY 로 못 빼고 높이로만 구분한다). HOME 도착 후 어디로 갈지는
    # task_manager._home_next 가 정한다.
    State.RELEASE_RETRY:  {State.HOME, State.ABORT},
    # LIFT -> WAIT_PLACE_TARGET: 이번 pick 이 place 를 지정 안 했으면(cmd:pick place 생략,
    # vla_command_node 가 /pick/place_pending=true 로 알림) 자동으로 basket 에 놓지 않고
    # 물체를 문 채 set_place 를 기다린다. place 를 지정했으면 지금처럼 곧장 PLACE(하위호환).
    State.LIFT:           {State.PLACE, State.WAIT_PLACE_TARGET, State.ABORT},
    # set_place(/pick/place_location)가 오면 PLACE, 안 오면 타임아웃 후 기본 위치로 PLACE.
    # WAIT_PLACE_TARGET -> RELEASE: /pick/release_now("놓을 데 필요 없다, 그냥 여기 둬라").
    # 목적지로 **이동하지 않고** 지금 자리에서 연다 — 재인식도 계획도 없다. 이게 없으면
    # "집어서 들고만 있어줘" 다음에 사람이 "됐어 놔"라고 할 때 표현할 말이 없어서,
    # 유일한 탈출구가 basket 으로 보내거나 ABORT 하는 것뿐이었다.
    State.WAIT_PLACE_TARGET: {State.PLACE, State.RELEASE, State.ABORT},
    # PLACE 가 motion_retries 까지 소진하면 곧장 ABORT 가 아니라 PLACE_RETRY 로 간다 —
    # 물체를 이미 들고 있어서(HOLDING_STATES) 재인식부터 다시 하는 SAFE_STOP 복구보다
    # "다른 위치로 다시 계획"이 훨씬 싸고 안전하다(재촬영 없음). 사람이 /pick/retry_place
    # 를 부르기 전까진 SAFE_STOP 처럼 정지 유지.
    State.PLACE:          {State.RELEASE, State.PLACE_RETRY, State.ABORT},
    State.PLACE_RETRY:    {State.PLACE, State.ABORT},
    State.RELEASE:        {State.HOME, State.ABORT},
    State.HOME:           {State.IDLE, State.PERCEIVE, State.ABORT},
    # PAUSED 탈출 — **사람의 다음 명령으로만** 나간다. 이 표에 없는 것으로는 절대 안 빠져나온다.
    # 판정 기준 하나: **움직이라는 뜻이 담긴 명령이면 즉시 실행한다.** 재개를 위해 두 번
    # 말하게 하지 않는다(2026-08-12 사용자 결정 ②).
    #   resume(보유 X)      → PERCEIVE   멈춘 사이 테이블이 바뀌었을 수 있어 처음부터 다시 본다
    #   resume(보유 O·목적지 O) → PLACE   재인식하지 않는다(자기 물체를 오인식한다)
    #   resume(보유 O·목적지 X) → WAIT_PLACE_TARGET
    #   release_now         → RELEASE    "그냥 거기 놔"
    #   home(비보유만)      → HOME
    #   stow                → PLACE|RELEASE|HOME  (§ _srv_stow)
    #   abort / E-Stop      → ABORT → SAFE_STOP  (기존 파괴적 경로 그대로 남는다)
    State.PAUSED:         {State.PERCEIVE, State.PLACE, State.WAIT_PLACE_TARGET,
                           State.RELEASE, State.HOME, State.IDLE, State.ABORT},
    State.SPEAK_FAIL:     {State.IDLE, State.LISTENING},
    State.ABORT:          {State.SAFE_STOP},
    State.SAFE_STOP:      {State.HOME},
}

#: PAUSED **진입**은 손으로 적지 않고 여기서 한 번에 넣는다.
#:
#: "멈춰"는 거부하면 안 되는 유일한 명령이라, 뭔가 진행 중인 상태 전부에서 갈 수 있어야 한다.
#: 17줄을 위 표에 손으로 흩뿌리면 (a) 표가 안 읽히고 (b) **새 상태를 추가한 사람이 pause 를
#: 빠뜨려 "거기서만 안 멈추는" 구멍**이 생긴다. 여기서 넣으면 그 누락이 구조적으로 불가능하다.
#: `* -> ABORT` 를 다이어그램에서 note 로 묶은 것과 같은 취급이다(그리면 못 읽는다).
#:
#: 제외 대상의 근거:
#:   IDLE        멈출 게 없다. pause 는 성공으로 응답하고 상태를 안 바꾼다(멱등)
#:   SPEAK_FAIL  말만 하고 즉시 빠지는 통과 상태
#:   ABORT/SAFE_STOP  이미 정지다. 여기서 PAUSED 로 가면 정지가 **약해진다**
#:   PAUSED      자기 자신 (is_allowed 가 항상 허용한다)
PAUSE_EXEMPT = frozenset({State.IDLE, State.SPEAK_FAIL, State.ABORT,
                          State.SAFE_STOP, State.PAUSED})

for _src in State:
    if _src not in PAUSE_EXEMPT:
        TRANSITIONS.setdefault(_src, set()).add(State.PAUSED)
del _src

#: 로봇이 실제로 움직일 수 있는 상태. ABORT 시 진행 중인 goal 을 취소해야 하는 구간이다.
MOTION_STATES = frozenset({State.APPROACH, State.DESCEND, State.LIFT, State.PLACE, State.HOME})

#: 물체를 물고 있을 수 있는 상태. 여기서 ABORT 가 나도 **그리퍼를 열지 않는다** —
#: 들고 있던 걸 떨어뜨리는 게 멈춰 있는 것보다 위험하다.
#:
#: `PAUSED` 가 여기 있는 이유: 물체를 든 채로도 멈출 수 있고, 그때 그리퍼를 놓으면
#: "일시정지"가 아니라 낙하다. PAUSED 는 어디서 멈췄는지에 따라 물고 있을 수도 아닐 수도
#: 있는데, **물고 있을 수 있으면 여기 넣는 게 맞다** — 이 집합은 "확실히 물고 있다"가
#: 아니라 "물고 있을 수 있으니 그리퍼를 건드리지 마라"는 뜻이다.
HOLDING_STATES = frozenset({State.VERIFY, State.LIFT, State.WAIT_PLACE_TARGET,
                            State.PLACE, State.PLACE_RETRY, State.PAUSED})


def is_allowed(src: State, dst: State) -> bool:
    """전이표에 있는 전이인가. 자기 자신으로의 전이(= 상태 유지)는 항상 허용."""
    return dst is src or dst in TRANSITIONS.get(src, set())
