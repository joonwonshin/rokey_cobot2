#!/usr/bin/env python3
r"""`WAIT_APPROVAL` 동안 사람이 **음성으로** `/pick/approve` 를 부를 수 있게 한다.

    ros2 run voice_processing approve_listener_node

## 이게 왜 필요한가

`pick_fsm.rqt_panel` 의 '승인' 버튼은 이미 `/pick/approve` 를 직접 부른다 — 그건 이 노드가
없어도 된다. 이 노드는 **그 옆에 음성 경로를 하나 더 놓는 것뿐**이다: graspgenx 판단 화면을
사람이 직접 보면서, 마우스로 버튼까지 갈 필요 없이 "헬로 로키, 승인"처럼 말해서 같은 승인을
내릴 수 있게 한다.

## 🔴 VLA 의 `cmd:"approve"` 차단과는 무관하다 — 오히려 그 반대다

`vla_command_node.BLOCKED_CMDS`(`= ('approve',)`)는 **외부 PC(VLA/LLM)**가 승인을 자동으로
보내는 경로를 막는다(계획 §0-B) — 이 노드는 그걸 우회하는 것이 아니라 **정반대**다: 이
노드가 듣는 마이크는 로봇 앞에 있는 **사람**이고, "승인"이라고 말하는 행위 자체가 버튼을
누르는 것과 똑같은 **사람의 결정**이다. `_srv_approve` 쪽 코드는 호출자가 사람인지 VLA인지
구분하지 못하므로, 이 노드가 있어도 VLA 경로의 차단은 전혀 약해지지 않는다 — 이 노드 자체가
`/vla/pick_command` 를 구독하지 않고 `vla_command_node` 와 아무 것도 주고받지 않는다.

## `WAIT_APPROVAL` 동안만 듣는다 — 그 전에도 그 후에도 마이크를 열지 않는다

`get_keyword`(LISTENING 상태에서 타겟 이름을 뽑는 노드)와 같은 `hello_rokey` 웨이크워드를
재사용한다 — 상시 듣는 게 아니라 "헬로 로키"가 감지된 뒤에만 STT 로 넘긴다. 이렇게 해도
**듣는 시점 자체는 `WAIT_APPROVAL` 상태에만 한정한다**: 옆에서 나누는 잡담이 우연히
"헬로 로키"를 포함해도, 로봇이 승인을 기다리는 중이 아니면 이 노드는 마이크 스트림 자체를
열지 않는다. 이중 방어다.

## `get_keyword` 와 같은 마이크를 쓴다 — 동시에 듣지 않는다

`LISTENING`(get_keyword 가 마이크를 쓴다)과 `WAIT_APPROVAL`(이 노드가 쓴다)은 같은 pick
사이클 안에서 **순서대로** 일어나는 상태라 정상 경로에서는 겹치지 않는다. 그래도
`get_keyword.py` 는 스트림을 연 뒤 **닫지 않는 기존 버그**가 있다(`close_stream()` 호출이
없다) — 이 노드는 반대로 `WAIT_APPROVAL` 을 벗어날 때마다 반드시 스트림을 닫는다. 그래도
`get_keyword` 호출 직후 곧바로 승인 단계로 넘어가는 흐름에서 장치 점유가 겹칠 수 있다 —
겹치면 `pyaudio` 가 장치 오류를 낸다(로그로 보인다). 재발하면 `get_keyword.py` 의 스트림을
닫는 것이 근본 수정이다(이 노드의 책임 밖).

## 승인 문구는 일부러 좁게 잡았다

기본값(`approve_phrases`)에 "네"/"응"/"오케이"처럼 일상 대화에 흔한 말을 넣지 않았다 —
실기 로봇을 움직이는 신호이므로, 대화 도중 우연히 일치하는 확률을 낮추는 쪽을 택했다
(`~/.claude/CLAUDE.md` "실기 안전"). 필요하면 `approve_phrases` 파라미터로 바꾼다.
"""

import os
import threading
import time

import pyaudio
import rclpy
from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from voice_processing.MicController import MicConfig, MicController
from voice_processing.stt import STT
from voice_processing.wakeup_word import WakeupWord

#: `vla_command_node.COMMAND_QOS`·`task_manager` 의 `/pick/state` 발행자와 같은 프로파일
#: 이라야 매칭된다 — 어긋나면 이 노드는 상태를 영원히 못 받고 조용히 `IDLE` 로만 남는다.
STATE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)

WAIT_APPROVAL_STATE = 'WAIT_APPROVAL'

PACKAGE_PATH = get_package_share_directory('voice_processing')
ENV_PATH = os.path.join(PACKAGE_PATH, 'resource', '.env')
load_dotenv(dotenv_path=ENV_PATH)


def normalize(text: str) -> str:
    """STT 결과에서 공백·구두점을 지운다. '승인!'/'승인.'/'승인 '이 전부 같게."""
    return ''.join(ch for ch in text if not ch.isspace() and ch not in '.,!?~')


def matches_approve(text: str, phrases) -> bool:
    """`phrases` 중 하나라도 정규화된 `text` 안에 부분문자열로 있으면 승인."""
    norm = normalize(text)
    return any(normalize(p) in norm for p in phrases if p.strip())


class ApproveListenerNode(Node):
    """`WAIT_APPROVAL` 동안만 웨이크워드→STT→문구매칭→`/pick/approve` 를 돈다."""

    def __init__(self):
        super().__init__('approve_listener_node')

        self.declare_parameter('state_topic', '/pick/state')
        self.declare_parameter('approve_service', '/pick/approve')
        # 콤마 문자열이다 — `vla_command_node.allowed_classes` 와 같은 이유
        # (빈 리스트 기본값은 rclpy 가 BYTE_ARRAY 로 추론해 문자열을 못 담는다).
        self.declare_parameter('approve_phrases', '승인,그립해,그립,진행해,진행,컨펌')
        self.declare_parameter('mic_device_index', 10)
        self.declare_parameter('record_seconds', 4)

        openai_api_key = os.getenv('OPENAI_API_KEY')
        if not openai_api_key:
            raise RuntimeError(
                f'OPENAI_API_KEY 가 없다 ({ENV_PATH}) — get_keyword 와 같은 .env 를 쓴다')
        self._stt = STT(openai_api_key=openai_api_key)

        mic_config = MicConfig(
            chunk=12000, rate=48000, channels=1,
            record_seconds=int(self.get_parameter('record_seconds').value),
            fmt=pyaudio.paInt16,
            device_index=int(self.get_parameter('mic_device_index').value),
            buffer_size=24000,
        )
        self._mic = MicController(config=mic_config)
        self._wakeword = WakeupWord(mic_config.buffer_size)
        self._stream_open = False

        self._state = ''
        self._state_lock = threading.Lock()
        self.create_subscription(
            String, str(self.get_parameter('state_topic').value),
            self._on_state, STATE_QOS)
        self.approve_cli = self.create_client(
            Trigger, str(self.get_parameter('approve_service').value))

        self.get_logger().info(
            f"준비됨 — '{WAIT_APPROVAL_STATE}' 동안만 듣는다. "
            f"승인 문구: {self._phrases()}")

    def _phrases(self):
        raw = str(self.get_parameter('approve_phrases').value)
        return [p.strip() for p in raw.split(',') if p.strip()]

    def _on_state(self, msg):
        with self._state_lock:
            self._state = msg.data

    def _current_state(self) -> str:
        with self._state_lock:
            return self._state

    # ── 마이크 스트림 수명 — WAIT_APPROVAL 동안만 연다 ──
    def _ensure_stream(self) -> bool:
        if self._stream_open:
            return True
        try:
            self._mic.open_stream()
            self._wakeword.set_stream(self._mic.stream)
            self._stream_open = True
            self.get_logger().info('마이크 스트림 열림 (승인 대기)')
            return True
        except OSError:
            self.get_logger().error(
                '오디오 스트림을 못 열었다 — device_index 나 다른 노드(get_keyword)와의 '
                '장치 점유 충돌을 확인할 것', throttle_duration_sec=10.0)
            return False

    def _close_stream(self):
        if not self._stream_open:
            return
        self._mic.close_stream()
        self._stream_open = False
        self.get_logger().info('마이크 스트림 닫음 (승인 대기 종료)')

    # ── 승인 호출 — call_async, 이 스레드는 rclpy 콜백이 아니라 자유롭게 기다려도 된다 ──
    def _call_approve(self):
        if not self.approve_cli.service_is_ready():
            self.get_logger().warn('/pick/approve 서비스가 아직 없다')
            return
        fut = self.approve_cli.call_async(Trigger.Request())
        while rclpy.ok() and not fut.done():
            time.sleep(0.05)
        res = fut.result()
        if res is None:
            self.get_logger().warn('승인 요청 응답 없음')
        elif res.success:
            self.get_logger().info(f'🎙️ 음성 승인 완료: {res.message}')
        else:
            self.get_logger().warn(f'승인 요청 거부됨: {res.message}')

    def run(self):
        """블로킹 루프. 별도 스레드에서 `rclpy.spin(self)` 가 이미 돌고 있어야 상태가 갱신된다."""
        phrases = self._phrases()
        while rclpy.ok():
            if self._current_state() != WAIT_APPROVAL_STATE:
                self._close_stream()
                time.sleep(0.5)
                continue

            if not self._ensure_stream():
                time.sleep(2.0)
                continue

            # is_wakeup() 은 버퍼 한 청크(48kHz/24000 ≈ 0.5s)만 읽고 돌아온다 — 그래서
            # 이 while 이 "블로킹 한 번"이 아니라 상태 변화를 놓치지 않고 자주 재확인한다.
            if not self._wakeword.is_wakeup():
                continue
            if self._current_state() != WAIT_APPROVAL_STATE:
                # 웨이크워드가 감지되는 그 짧은 순간에 상태가 이미 바뀌었을 수 있다
                # (타임아웃 ABORT 등) — STT 로 넘기지 않고 다음 루프에서 스트림을 닫는다.
                continue

            text = self._stt.speech2text()
            self.get_logger().info(f"들은 말: '{text}'")
            if matches_approve(text, phrases):
                if self._current_state() == WAIT_APPROVAL_STATE:
                    self._call_approve()
                else:
                    self.get_logger().warn(
                        'STT 왕복 중 상태가 바뀌어 승인을 보내지 않았다 (안전 재확인)')
            else:
                self.get_logger().info(
                    f"승인 문구 아님 — {phrases} 중 하나를 말하거나 rqt 패널 버튼을 쓸 것")


def main():
    rclpy.init()
    node = ApproveListenerNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node._close_stream()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
