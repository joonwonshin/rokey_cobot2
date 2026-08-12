"""OnRobot RG2 래퍼. 이 ws 의 드라이버가 실제로 받는 형식으로만 명령한다.

근거는 전부 실물 소스다:
  - `onrobot_rg_control/OnRobotRGControllerServer.py:294` sendCommandCallback
  - 같은 파일 `:258` genCommand — 'o'=열기, 'c'=닫기, 'i'/'d'=힘 ±25, **숫자문자열=rgwd**
  - `onrobot_rg_msgs/msg/OnRobotRGOutput.msg` — rgwd 단위는 mm 가 아니라 **1/10 mm**

⚠️ 실기 드라이버와 가상 노드가 숫자 문자열을 다르게 해석한다.
   real  (`OnRobotRGControllerServer.py:289`): `int(char)` → rgwd, 1/10 mm
   virtual(`m0609_rg2_bringup/scripts/gripper_virtual_node.py:52`): `float(char)` → **관절각(rad)**
   같은 "480" 이 한쪽은 48 mm, 한쪽은 상한 클램프된 0.785 rad 이다.
   → backend 를 명시적으로 받는다. 추측으로 하나만 지원하면 시뮬에서 통과하고 실기에서 으깬다.
"""

from onrobot_rg_msgs.srv import SetCommand
from std_msgs.msg import Bool

#: 드라이버 유효범위 (`OnRobotRGControllerServer.py:143` max_width). 1/10 mm.
RG2_MAX_RGWD = 1100
#: 하드웨어 물리 한계 [m]. 위 값과 같은 것을 m 로 적은 것뿐이다.
RG2_MAX_WIDTH_M = 0.110
#: GraspGenX 가 conditioning 한 개구 폭 [m] (`onrobot_RG2/config.json` sweep_volume.extents[0]).
#: 하드웨어 한계(0.110)보다 **좁다**. 후보 선별 기준은 좁은 쪽을 쓴다.
RG2_MODEL_WIDTH_M = 0.102

#: 2026-08-07 실측(`kimkh`) — 펜던트에서 TCP 설정 후 그리퍼를 수직정렬하고 천천히 내려
#: 접촉 지점까지 잰 값. **완전히 닫힌(0 mm) 상태**의 tool0 플랜지면 -> 손끝 거리.
#: 브라켓+퀵커넥터 22 mm + 그리퍼 자체(닫힘) 218 mm = 240 mm.
#: GraspGenX 의 상수 0.18 m(`config.json` fingertip[2])보다 6 cm 길다 — 그 값을 그대로 쓰면
#: 실기에서 손끝이 6 cm 짧게 나가 물체 앞에서 헛짚는다(md/context/constraints.md 2026-08-05).
RG2_CLOSED_FINGERTIP_LENGTH_M = 0.240

#: 브라켓 + 퀵커넥터 두께 [m], 2026-08-07 실측. `tool0 -> rg2_base_link` 평행이동이며
#: `onrobot_rg2.xacro:40` 의 `xyz="0.022 0 0"` 과 **같은 값이어야 한다**(tool0 의 +X 가
#: 그리퍼 접근축이라 X 성분에 들어가 있다). 폭과 무관한 고정 오프셋이다.
RG2_BRACKET_LENGTH_M = 0.022

#: 개구 폭이 커질수록 손가락이 힌지를 축으로 벌어지며 손끝이 짧아진다 — 회전 링크라 폭에
#: **선형이 아니다**. {개구 폭[m]: 닫힘(0.240 m) 대비 길이 감소[m]}, 2026-08-07 실측 2점.
RG2_LENGTH_SHRINK_CAL_M = {
    0.000: 0.000,
    0.070: 0.017,
    0.100: 0.041,
}


def fingertip_length_m(width_m: float) -> float:
    """개구 폭 [m] -> tool0 플랜지면부터 손끝까지 실측 거리 [m] (보정표 선형보간).

    `RG2_LENGTH_SHRINK_CAL_M` 3점을 구간별로 선형보간한다. 힌지 회전 구조라 실제로는
    비선형이지만(구간 기울기가 0.24 -> 0.8 mm/mm 로 뛴다), 점이 3개뿐이라 그 이상의
    곡선을 지어내지 않는다. 표 밖(>100 mm, RG2 최대 110 mm 까지)은 마지막 구간 기울기로
    외삽한다 — 그 구간은 실측 안 됐으니 결과를 그대로 신뢰하지 말 것.

    ⚠️ 기준면이 **`tool0` 플랜지면**이다. grasp 포즈의 원점은 `rg2_base_link`(플랜지면에서
    브라켓 22 mm 앞)이므로 **grasp 포즈에 이 값을 그대로 더하면 22 mm 더 나간다.**
    그 경우엔 `fingertip_from_rg2_base_m()` 을 쓴다.

    이 값은 IK 목표 자체를 바꾸지 않는다 — 쓰는 곳은 손끝 위치 **추정**뿐이다:
    CollisionObject 배치, 로그, RViz 시각화.
    """
    xs = sorted(RG2_LENGTH_SHRINK_CAL_M)
    w = max(0.0, float(width_m))
    if w >= xs[-1]:
        x0, x1 = xs[-2], xs[-1]
    else:
        x0, x1 = next((xs[i], xs[i + 1]) for i in range(len(xs) - 1) if w <= xs[i + 1])
    y0, y1 = RG2_LENGTH_SHRINK_CAL_M[x0], RG2_LENGTH_SHRINK_CAL_M[x1]
    shrink = y0 + (y1 - y0) * (w - x0) / (x1 - x0)
    return RG2_CLOSED_FINGERTIP_LENGTH_M - shrink


def fingertip_from_rg2_base_m(width_m: float) -> float:
    """개구 폭 [m] -> **`rg2_base_link` 원점**부터 손끝까지 거리 [m].

    grasp 포즈(원점 = 그리퍼 base)에 얹을 때 쓰는 값이다. 플랜지면 기준
    `fingertip_length_m()` 에서 브라켓 22 mm 를 뺀 것 — 닫힘 상태에서 218 mm 다.
    """
    return fingertip_length_m(width_m) - RG2_BRACKET_LENGTH_M


def grip_target_width_m(object_width_m: float, clearance_m: float,
                        max_width_m: float = RG2_MODEL_WIDTH_M) -> float:
    """**물체 폭** [m] -> 그리퍼에 명령할 **목표 개구 폭** [m].

    🔴 조임 여유는 **뺀다**(2026-08-09 부호 수정). `close_async` 가 보내는 값은 "여기까지
       닫아라"는 목표 개구 폭이다 — 물체 폭보다 **넓게** 명령하면 손가락이 물체에 닿기 전에
       멈추고, 힘이 안 걸려 `grip_detected` 가 false 로 남는다(= 잡은 줄 알았는데 빈 손).
       더하는 게 맞아 보이는 건 "물체를 넣을 틈"을 떠올려서인데, 그 틈은 접근 전에 보내는
       `open_async()`('o' = 완전 개방)가 이미 만든다. 이 값은 **닫은 뒤** 폭이다.

    과하게 빼는 쪽은 드라이버 힘 제한(rgfr)이 막아준다 — 접촉하면 거기서 스톨한다.
    반대로 덜 빼면 아무 힘도 안 걸린다. 그래서 **비대칭 위험**이고, 부호를 틀리면 안 된다.
    """
    return max(0.0, min(float(max_width_m), float(object_width_m) - float(clearance_m)))


def width_to_rgwd(width_m: float) -> int:
    """개구 폭 [m] → 드라이버 명령값 [1/10 mm], 유효범위로 클램프.

    **m → mm 로만 바꾸면 10배 좁게 명령해 물체를 으깬다.** 0.048 m 는 48 이 아니라 480 이다.
    """
    return max(0, min(RG2_MAX_RGWD, int(round(float(width_m) * 10000.0))))


class Rg2Client:
    """`/onrobot/sendCommand` 호출과 파지 판정 구독만 한다. 블로킹하지 않는다."""

    def __init__(self, node, callback_group, backend: str = 'real',
                 service: str = '/onrobot/sendCommand',
                 grip_topic: str = '/onrobot/grip_detected'):
        self._node = node
        self._backend = backend
        self._cli = node.create_client(SetCommand, service, callback_group=callback_group)
        self._grip = None          # None = 한 번도 못 받음 (가상 노드는 아예 발행하지 않는다)
        self._grip_stamp = None
        node.create_subscription(Bool, grip_topic, self._on_grip, 10,
                                 callback_group=callback_group)

    # ── 상태 ────────────────────────────────────────────────
    def _on_grip(self, msg: Bool):
        # gSTA(register 268) bit1. 힘 센서 없이 "물렸다"를 아는 유일한 신호다.
        self._grip = bool(msg.data)
        self._grip_stamp = self._node.get_clock().now()

    def grip_detected(self, max_age_sec: float = 1.0):
        """(판정, 사유). 판정이 None 이면 **모름** — 실패로 읽으면 안 된다."""
        if self._grip is None or self._grip_stamp is None:
            return None, 'grip_detected 토픽을 한 번도 못 받았다 (가상 그리퍼이거나 드라이버 미기동)'
        age = (self._node.get_clock().now() - self._grip_stamp).nanoseconds * 1e-9
        if age > max_age_sec:
            return None, f'grip_detected 가 {age:.1f}s 째 갱신되지 않았다'
        return self._grip, f'grip_detected={self._grip}'

    def service_ready(self) -> bool:
        return self._cli.service_is_ready()

    # ── 명령 ────────────────────────────────────────────────
    def _send(self, command: str):
        req = SetCommand.Request()
        req.command = command
        return self._cli.call_async(req)

    def open_async(self):
        return self._send('o')

    def close_async(self, width_m: float):
        """`width_m` 까지 닫는다. backend 에 따라 명령 문자열의 의미가 다르다."""
        if self._backend == 'real':
            return self._send(str(width_to_rgwd(width_m)))
        # 가상 노드는 폭을 모른다. 폭↔관절각 변환을 여기서 새로 짜지 않는다
        # (`_OnRobotRGIsaacSimController.py:131 widthToJointValue()` 가 이미 있고,
        #  상수를 베껴오면 드라이버가 바뀔 때 조용히 갈라진다) → 완전히 닫는다.
        return self._send('c')

    def lower_force_async(self, steps: int):
        """'d' 를 `steps` 번 보내 목표 힘을 25 (=2.5 N) 씩 낮춘다.

        ⚠️ 드라이버는 기동 시 `rgfr = max_force` = 400 = **40 N (RG2 최대)** 로 시작하고
        (`OnRobotRGControllerServer.py:57`), 서비스로 힘을 직접 지정할 방법이 없다.
        사과 같은 것을 40 N 으로 물면 으깬다. 이 함수가 유일한 감쇄 경로다.
        steps=0 이면 아무것도 안 보낸다(드라이버 기본값 유지).
        """
        return [self._send('d') for _ in range(max(0, int(steps)))]
