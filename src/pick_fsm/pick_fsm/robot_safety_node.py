#!/usr/bin/env python3
"""로봇 안전상태 브릿지 — Doosan 드라이버의 안전 서비스를 얇게 감싼다.

    ros2 run pick_fsm robot_safety_node

**`task_manager`와 별도 노드로 뒀다.** 픽 FSM이 에러 루프에 갇히거나 죽어도 안전 조작
(즉시정지·backdrive)은 별도 프로세스라 여전히 먹어야 한다는 게 이유다 — 안전 기능을
복잡한 상위 애플리케이션 로직의 건강 상태에 기대게 하지 않는다.

발행:
    /pick/robot_state_code (std_msgs/Int8)   — GetRobotState.srv 의 원본 정수
    /pick/robot_state_text (std_msgs/String) — 사람이 읽는 이름

서비스 (전부 std_srvs/Trigger, **fire-and-forget** — task_manager 의 `/pick/start` 와 같은
계약이다. 결과를 기다리지 않고 바로 응답하고, 실제 결과는 로그와 `/pick/robot_state_text`
로 나중에 드러난다. 서비스 콜백 안에서 `spin_until_future_complete` 를 부르면 재진입으로
엉킨다는 게 이 워크스페이스에서 이미 겪은 함정이라 — `task_manager.py` 의 `_service()` 주석
참고 — 아예 블로킹을 안 하는 쪽으로 맞췄다):
    /safety/stop            — MoveStop(DR_HOLD) 즉시 정지
    /safety/enter_backdrive — SetSafetyMode(BACKDRIVE) — 사람이 손으로 팔을 밀 수 있게
    /safety/exit_backdrive  — SetSafetyMode(AUTONOMOUS) + SAFE_STOP 이면 리셋까지

⚠️ **`CONTROL_RECOVERY_BACKDRIVE`(SetRobotControl.srv 값 6)는 절대 안 쓴다.** 이름이
비슷해서 헷갈리기 쉬운데, 그건 STATE_SAFE_OFF2 전용 H/W 복구 경로라 쓰면 **컨트롤러 전원을
재부팅해야 STATE_STANDBY 로 돌아온다**(dsr_msgs2/srv/system/SetRobotControl.srv:15 주석).
여기서 쓰는 건 SetSafetyMode.srv 의 safety_mode=BACKDRIVE 뿐이고, 그건 그런 경고가 없다.

⚠️ **backdrive 진입/해제는 이 코드베이스에서 실기로 검증된 적이 없다(2026-08-07 작성 시점).**
`dsr_controller2.cpp`의 `OnMonitoringStateCB`가 `STATE_SAFE_STOP`에서는 스스로
`set_robot_control(CONTROL_RESET_SAFET_STOP)`으로 자동 복귀를 **시도한다**(`g_bHasControlAuthority`
가 있을 때, 소스 라인 3024-3029) — 그래서 `/safety/exit_backdrive`가 하는 `CONTROL_RESET_SAFET_STOP`
호출은 드라이버가 이미 하고 있을 자동 복구의 보험이지 유일한 경로가 아니다. 하지만 backdrive
**진입**은 어떤 상태에서도 드라이버가 자동으로 하지 않으므로 이 서비스가 유일한 경로다. 처음
쓸 때는 반드시 비상정지 버튼을 손 닿는 곳에 두고, 팔이 실제로 손으로 밀리는지 저속·저위험
자세에서 먼저 확인할 것.
"""

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger

from dsr_msgs2.srv import GetRobotState, MoveStop, SetRobotControl, SetSafetyMode

# GetRobotState.srv 주석 그대로. 정의 안 된 코드(11~14)는 안 넣는다 — 나오면 오히려
# UNKNOWN(n) 으로 보이는 게 "이 코드 처리 안 해놨다"를 알려준다.
ROBOT_STATE_NAMES = {
    0: 'INITIALIZING', 1: 'STANDBY', 2: 'MOVING', 3: 'SAFE_OFF', 4: 'TEACHING',
    5: 'SAFE_STOP', 6: 'EMERGENCY_STOP', 7: 'HOMMING', 8: 'RECOVERY',
    9: 'SAFE_STOP2', 10: 'SAFE_OFF2', 15: 'NOT_READY',
}
#: task_manager 가 진행 중 작업을 중단해야 하는 상태(충돌·안전정지류). MOVING/STANDBY/
#: TEACHING/HOMMING/RECOVERY/INITIALIZING 은 안 건드린다 — TEACHING 은 사람이 펜던트로
#: 쓰는 중일 수 있어 이것만으로 "안전"이라 단정하지 않는다.
UNSAFE_STATES = frozenset({3, 5, 6, 9, 10})

SAFETY_MODE_AUTONOMOUS = 1
SAFETY_MODE_BACKDRIVE = 3
SAFETY_MODE_EVENT_ENTER = 0

CONTROL_RESET_SAFET_STOP = 2
CONTROL_RESET_SAFET_OFF = 3

#: MoveStop.srv 의 DR_HOLD. ServoOff.srv 는 이 값을 STOP_TYPE_EMERGENCY 라고도 부른다 —
#: Doosan 자신이 "이게 비상정지급"이라고 표시해둔 값이라 즉시정지 버튼에 이걸 쓴다.
DR_HOLD = 3


class RobotSafetyNode(Node):
    def __init__(self):
        super().__init__('robot_safety_node')
        self.declare_parameter('robot_ns', 'dsr01')
        self.declare_parameter('poll_hz', 2.0)
        ns = self.get_parameter('robot_ns').value
        self._prefix = f'/{ns}' if ns else ''
        cb = ReentrantCallbackGroup()

        self.cli_state = self.create_client(
            GetRobotState, f'{self._prefix}/system/get_robot_state', callback_group=cb)
        self.cli_control = self.create_client(
            SetRobotControl, f'{self._prefix}/system/set_robot_control', callback_group=cb)
        self.cli_safety_mode = self.create_client(
            SetSafetyMode, f'{self._prefix}/system/set_safety_mode', callback_group=cb)
        self.cli_stop = self.create_client(
            MoveStop, f'{self._prefix}/motion/move_stop', callback_group=cb)

        self.pub_code = self.create_publisher(Int8, '/pick/robot_state_code', 10)
        self.pub_text = self.create_publisher(String, '/pick/robot_state_text', 10)

        # get_robot_state 는 task_manager._service() 와 같은 폴링 방식으로 부른다 —
        # 타이머 콜백 안에서 spin_until_future_complete 를 부르면 재진입으로 엉킨다
        # (task_manager.py 의 같은 주석 참고). 여기도 매 tick 에 done() 만 확인한다.
        self._poll_fut = None
        hz = float(self.get_parameter('poll_hz').value)
        self.create_timer(1.0 / hz, self._poll, callback_group=cb)

        self.create_service(Trigger, '/safety/stop', self._srv_stop, callback_group=cb)
        self.create_service(Trigger, '/safety/enter_backdrive', self._srv_enter_backdrive,
                            callback_group=cb)
        self.create_service(Trigger, '/safety/exit_backdrive', self._srv_exit_backdrive,
                            callback_group=cb)

        self.get_logger().info(
            f"준비됨 — 로봇 네임스페이스 '{ns or '(없음)'}' 기준 Doosan 안전 서비스에 연결 시도")

    # ── 상태 폴링 ────────────────────────────────────────────
    def _poll(self):
        if self._poll_fut is None:
            if not self.cli_state.service_is_ready():
                return
            self._poll_fut = self.cli_state.call_async(GetRobotState.Request())
            return
        if not self._poll_fut.done():
            return
        res = self._poll_fut.result()
        self._poll_fut = None
        if res is None or not res.success:
            return
        code = int(res.robot_state)
        self.pub_code.publish(Int8(data=code))
        self.pub_text.publish(String(data=ROBOT_STATE_NAMES.get(code, f'UNKNOWN({code})')))

    # ── fire-and-forget 안전 명령 ────────────────────────────
    def _fire(self, client, request, name, response):
        """서비스 준비됐으면 비동기로 쏘고 바로 응답. 완료는 로그로만 확인한다."""
        if not client.service_is_ready():
            response.success, response.message = False, f'{name} 서비스 없음 (dsr_controller2 미기동?)'
            return response
        fut = client.call_async(request)

        def _done(f):
            res = f.result()
            if res is None:
                self.get_logger().error(f'{name}: 응답 없음')
            elif not res.success:
                self.get_logger().error(f'{name}: 드라이버가 실패 응답')
            else:
                self.get_logger().info(f'{name}: 완료')
        fut.add_done_callback(_done)
        response.success, response.message = True, f'{name} 요청 전송함 — 결과는 로그·상태 토픽에서 확인'
        return response

    def _srv_stop(self, _req, res):
        req = MoveStop.Request()
        req.stop_mode = DR_HOLD
        return self._fire(self.cli_stop, req, 'move_stop(HOLD)', res)

    def _srv_enter_backdrive(self, _req, res):
        req = SetSafetyMode.Request()
        req.safety_mode = SAFETY_MODE_BACKDRIVE
        req.safety_event = SAFETY_MODE_EVENT_ENTER
        return self._fire(self.cli_safety_mode, req, 'set_safety_mode(BACKDRIVE)', res)

    def _srv_exit_backdrive(self, _req, res):
        req = SetSafetyMode.Request()
        req.safety_mode = SAFETY_MODE_AUTONOMOUS
        req.safety_event = SAFETY_MODE_EVENT_ENTER
        if not self.cli_safety_mode.service_is_ready():
            res.success, res.message = False, 'set_safety_mode 서비스 없음'
            return res
        fut = self.cli_safety_mode.call_async(req)

        def _done(f):
            r = f.result()
            if r is not None and r.success:
                self.get_logger().info('set_safety_mode(AUTONOMOUS): 완료')
            else:
                self.get_logger().error('set_safety_mode(AUTONOMOUS): 실패')
            # SAFE_STOP/SAFE_OFF 로 남아 있을 수 있으니 리셋도 시도한다. 드라이버가 이미
            # 자동으로 했을 수도 있다(OnMonitoringStateCB) — 이건 그 보험이지 유일한 경로가
            # 아니다. 실패해도 조용히 넘어간다: 애초에 그 상태가 아니면 드라이버가 거부하는
            # 게 정상이다.
            if self.cli_control.service_is_ready():
                for val, label in ((CONTROL_RESET_SAFET_STOP, 'RESET_SAFET_STOP'),
                                   (CONTROL_RESET_SAFET_OFF, 'RESET_SAFET_OFF')):
                    creq = SetRobotControl.Request()
                    creq.robot_control = val
                    cfut = self.cli_control.call_async(creq)
                    cfut.add_done_callback(
                        lambda f, label=label: self.get_logger().info(
                            f'set_robot_control({label}): '
                            f'{"완료" if f.result() and f.result().success else "실패/불필요"}'))
        fut.add_done_callback(_done)
        res.success, res.message = True, 'exit_backdrive 요청 전송함 — 결과는 로그·상태 토픽에서 확인'
        return res


def main(args=None):
    rclpy.init(args=args)
    node = RobotSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
