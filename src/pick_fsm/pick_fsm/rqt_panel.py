"""pick_fsm 상태 감시 + 타겟 선택 + 승인/안전 조작용 rqt 패널.

    rqt --standalone pick_fsm

타겟은 `/pick/target`(String)으로 나가고 task_manager 가 그걸 `grasp_bridge_node` 의
`target_classes` 로 밀어 넣는다. 여기서 브리지 파라미터를 직접 건드리지 않는 이유:
타겟의 정본이 두 군데가 되는 순간 어긋나도 아무도 모른다(2026-08-09 실기 실패).

`rqt_gui` 가 플러그인마다 공유 rclpy 노드를 하나씩 준다(`context.node`) — 여기서 새로
`rclpy.init()`/스레드를 만들지 않는다. 버튼 클릭은 전부 `call_async()`로 쏘고 바로 리턴한다.
`spin_until_future_complete`류 블로킹 호출은 rqt 의 spin 스레드를 막아 패널 전체가 멈춘다
(`task_manager.py`/`robot_safety_node.py`가 같은 이유로 폴링 방식을 쓴다) — 여기서도 QTimer 로
매 200ms 마다 대기 중인 future 의 `done()` 만 확인한다. ROS 콜백 스레드에서 Qt 위젯을 직접
건드리지 않는다(Qt 는 GUI 스레드 밖에서 위젯을 만지면 크래시할 수 있다) — 최신값을 변수에만
넣어두고 같은 QTimer 가 라벨에 반영한다.
"""

import json
import time

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from qt_gui.plugin import Plugin
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger

from pick_fsm.robot_safety_node import ROBOT_STATE_NAMES, UNSAFE_STATES
from pick_fsm.task_manager import TARGET_QOS

#: 눌렀을 때 로봇에 물리적 영향이 있어 확인창을 띄우는 버튼들
CONFIRM = {
    '즉시정지': '지금 진행 중인 모든 계획/실행을 즉시 멈춥니다. 계속할까요?',
    '안전모드 진입(backdrive)': ('사람이 손으로 팔을 밀 수 있는 모드로 들어갑니다.\n'
                                '전환 직후 팔이 갑자기 무거움/가벼움이 바뀔 수 있습니다 — '
                                '준비된 상태에서 누르세요.'),
    '안전모드 해제': '정상 모드로 복귀합니다. 팔 주변에 손이 없는지 확인하세요.',
    '그리퍼 파워사이클': ('RG2 Compute Box 전원을 1~2초 껐다 켭니다(OnRobot 매뉴얼 p.62 — '
                        '손끝 안쪽 안전스위치가 눌려 그리퍼가 멈췄을 때 쓰는 공식 복구 절차).\n'
                        '전원이 끊기는 동안 잡고 있던 물체가 떨어질 수 있습니다 — 물체를 쥔 '
                        '상태라면 누르기 전에 내려놓으세요.'),
}

#: task_manager.PLACE_LOCATIONS 중 아직 teach 안 된 것들 — 고를 때 확인창을 띄운다.
#: (placeholder 가 home_joints_deg 복사이던 시절엔 확인창 없이 고르면 홈 자세에서 그리퍼가
#:  열려 물체가 떨어졌다 — cross-review 2026-08-10.)
#: 2026-08-11: 'table'/'discard' 실기 teach 완료(pick_fsm.yaml)로 비웠다 — 남은 위치 없음.
#: 앞으로 teach 안 된 위치를 추가하면 그 이름을 여기 넣으면 확인창이 되살아난다.
UNVERIFIED_PLACE_LOCATIONS = set()

#: 서비스가 이 시간 안에 응답 안 하면 결과 표시에서 "시간초과"로 걷어낸다.
#: (버튼 자체가 fire-and-forget 라 로봇 명령이 취소되는 건 아니다 — UI 표시만 정리한다)
PENDING_TIMEOUT_SEC = 5.0

#: `/yolo_seg/classes` 를 이 시간 넘게 못 받으면 목록을 "옛날 것"으로 표시한다.
#: 카메라나 yolo_seg_node 가 죽어도 마지막 목록이 그대로 남아 "지금 보이는 물체"처럼
#: 읽히는 게 제일 나쁘다 — 없는 사과를 고르고 왜 실패하는지 찾게 된다.
CLASSES_STALE_SEC = 3.0

#: 자동 선택. 빈 target_classes = 브리지가 본 물체 전부에서 점수 최고를 고른다.
AUTO_LABEL = '(자동 — 점수 최고)'

#: 속도 스케일을 바꿀 대상 노드 (`pick_fsm.launch.py:125` 에서 name='task_manager', 네임스페이스 없음).
#: 절대 이름이라 이 패널이 어느 네임스페이스에 떠 있든 같은 노드를 가리킨다.
TASK_MANAGER_NODE = '/task_manager'

#: 여기서 바꾸는 파라미터. task_manager 는 이동을 시작할 때마다 `self.p('vel_scale')` 로
#: 다시 읽으므로(`task_manager.py:773`) set_parameters 는 **다음 이동 구간부터** 먹는다.
#: 지금 실행 중인 구간은 이미 move_group 에 goal 이 나가 있어 속도가 안 바뀐다.
SPEED_PARAMS = ('vel_scale', 'acc_scale')

#: 이 값을 **넘겨 올릴 때만** 확인창을 띄운다. 내리는 건 언제나 안전하므로 안 묻는다.
#: (yaml 기본값은 0.05 — `pick_fsm.yaml:23`. 실기에서 속도는 되돌리기 어려운 실수다)
FAST_CONFIRM_SCALE = 0.3


class PickFsmPanel(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('PickFsmPanel')
        node = context.node

        self._fsm_state = '(수신 대기)'
        self._robot_state = '(수신 대기)'
        self._robot_state_code = None
        self._pending = []          # [(label, future, deadline)] — 완료되면 QTimer 가 걷어간다
        self._clients = []          # 버튼마다 하나씩, __init__ 에서만 만든다 (재사용, 안 새로 안 만듦)
        self._active_target = None  # /pick/target_active 로 FSM 이 되돌려주는 현재 타겟
        self._active_place = None   # /pick/place_location_active 로 FSM 이 되돌려주는 현재 위치
        self._detected = []         # [(클래스이름, 개수)] — /yolo_seg/classes 최신 프레임
        self._detected_at = 0.0     # 그 프레임을 받은 시각 (monotonic)
        self._shown_classes = []    # 콤보에 지금 들어있는 목록. 같으면 다시 안 채운다
        self._param_futs = []       # [(종류, future, deadline)] — set/get 응답 대기
        self._speed_synced = False  # task_manager 에서 현재값을 한 번이라도 읽어왔는지

        self._widget = QWidget()
        self._widget.setWindowTitle('pick_fsm 상태/제어')
        outer = QVBoxLayout(self._widget)

        self._lbl_fsm = QLabel('FSM 상태: (수신 대기)')
        self._lbl_robot = QLabel('로봇 상태: (수신 대기)')
        self._lbl_result = QLabel('')
        self._lbl_result.setWordWrap(True)
        for lbl in (self._lbl_fsm, self._lbl_robot):
            f = lbl.font()
            f.setPointSize(f.pointSize() + 2)
            f.setBold(True)
            lbl.setFont(f)
        outer.addWidget(self._lbl_fsm)
        outer.addWidget(self._lbl_robot)

        # ── 타겟 ────────────────────────────────────────────
        # 잡을 대상은 여기서 고른다. FSM 이 이 값을 PERCEIVE 마다 grasp_bridge_node 의
        # target_classes 로 밀어 넣으므로, 브리지를 따로 `ros2 param set` 할 필요가 없다.
        self.target_pub = node.create_publisher(String, '/pick/target', TARGET_QOS)
        target_box = QGroupBox('타겟 — 무엇을 잡을지')
        target_col = QVBoxLayout(target_box)
        target_row = QHBoxLayout()
        # editable 이다: YOLO 가 아직 못 본 클래스도 미리 적어둘 수 있어야 한다
        # (탐지는 프레임마다 깜빡이는데 목록에 없다고 못 고르면 쓸모가 없다).
        self._combo = QComboBox()
        self._combo.setEditable(True)
        self._combo.setToolTip('YOLO 가 지금 보고 있는 클래스. 직접 입력해도 되고 '
                               "콤마로 여러 개도 된다 ('apple,orange')")
        target_row.addWidget(QLabel('클래스'))
        target_row.addWidget(self._combo, 1)
        btn_apply = QPushButton('적용')
        btn_apply.clicked.connect(lambda: self._set_target(self._combo.currentText().strip()))
        target_row.addWidget(btn_apply)
        btn_auto = QPushButton('자동')
        btn_auto.setToolTip('클래스를 지정하지 않는다 — 브리지가 본 물체 전부 중 '
                            'grasp 점수가 가장 높은 것을 잡는다')
        btn_auto.clicked.connect(lambda: self._set_target(''))
        target_row.addWidget(btn_auto)
        target_col.addLayout(target_row)
        self._lbl_target = QLabel('현재 타겟: (수신 대기)')
        self._lbl_target.setWordWrap(True)
        target_col.addWidget(self._lbl_target)
        outer.addWidget(target_box)

        # ── 내려놓을 위치 ────────────────────────────────────
        # basket(장바구니) / table(작업테이블 지정 자리) / discard(테이블 밖 폐기) 중 하나.
        # task_manager 의 PLACE_LOCATIONS 키와 그대로 맞아야 한다 — 편집 가능한 콤보가
        # 아닌 이유는(타겟과 달리) 여기 세 값 밖은 FSM 이 그냥 무시하기 때문이다.
        self.place_pub = node.create_publisher(String, '/pick/place_location', TARGET_QOS)
        place_box = QGroupBox('내려놓을 위치')
        place_col = QVBoxLayout(place_box)
        place_row = QHBoxLayout()
        self._place_combo = QComboBox()
        self._place_combo.addItems(['basket', 'table', 'discard'])
        self._place_combo.setToolTip('basket=장바구니 · table=작업테이블 지정 자리 · '
                                     'discard=작업테이블 바깥(폐기)')
        place_row.addWidget(QLabel('위치'))
        place_row.addWidget(self._place_combo, 1)
        btn_place_apply = QPushButton('적용')
        btn_place_apply.clicked.connect(
            lambda: self._set_place(self._place_combo.currentText()))
        place_row.addWidget(btn_place_apply)
        place_col.addLayout(place_row)
        self._lbl_place = QLabel('현재 위치: (수신 대기)')
        self._lbl_place.setWordWrap(True)
        place_col.addWidget(self._lbl_place)
        # PLACE_RETRY 전용: 놓기 모션이 실패해 물체를 문 채 정지했을 때만 먹는다
        # (task_manager._srv_retry_place). 위 콤보+[적용]으로 위치를 먼저 바꾸면 이
        # 상태에서는 즉시 반영되므로(_on_place_location 의 PLACE_RETRY 분기), 다른
        # 위치를 고르고 이 버튼을 누르면 그 위치로 재시도한다. 다른 상태에서 누르면
        # task_manager 가 거부 메시지를 준다 — 안전하게 그냥 실패로 표시된다.
        retry_row = QHBoxLayout()
        btn_place_retry = self._make_button(node, '놓기 재시도(PLACE_RETRY)', Trigger,
                                             '/pick/retry_place')
        btn_place_retry.setToolTip('그립까지 성공했는데 놓을 위치로 계획/이동이 실패해 '
                                   "물체를 문 채 정지(PLACE_RETRY)했을 때만 동작한다. "
                                   '위 콤보에서 다른 위치를 고르고 [적용] 한 뒤 눌러도 '
                                   '되고, 같은 위치로 그냥 재시도해도 된다.')
        retry_row.addWidget(btn_place_retry)
        place_col.addLayout(retry_row)
        outer.addWidget(place_box)

        # ── 속도 ────────────────────────────────────────────
        # MoveIt 의 max_velocity/acceleration_scaling_factor. 계획된 궤적을 시간축으로
        # 늘리는 배율이라 경로 자체는 그대로고 빠르기만 바뀐다.
        self.set_param_cli = node.create_client(
            SetParameters, f'{TASK_MANAGER_NODE}/set_parameters')
        self.get_param_cli = node.create_client(
            GetParameters, f'{TASK_MANAGER_NODE}/get_parameters')
        self._clients += [self.set_param_cli, self.get_param_cli]   # shutdown 에서 같이 정리
        speed_box = QGroupBox('속도 — 계획된 궤적을 얼마나 빠르게 실행할지')
        speed_col = QVBoxLayout(speed_box)
        speed_row = QHBoxLayout()
        self._spins = {}
        for key, label in (('vel_scale', '속도(vel)'), ('acc_scale', '가속(acc)')):
            spin = QDoubleSpinBox()
            spin.setRange(0.01, 1.0)    # 0 이면 궤적 시간이 무한대다 — 하한을 둔다
            spin.setSingleStep(0.05)
            spin.setDecimals(2)
            spin.setToolTip(f'task_manager 의 {key}. 1.0 = 로봇 최대속도. '
                            'yaml 기본값은 0.05 (실기 첫 시도용)')
            self._spins[key] = spin
            speed_row.addWidget(QLabel(label))
            speed_row.addWidget(spin)
        btn_speed = QPushButton('적용')
        btn_speed.clicked.connect(self._apply_speed)
        speed_row.addWidget(btn_speed)
        btn_speed_read = QPushButton('현재값 읽기')
        btn_speed_read.setToolTip('CLI 로 바꿨거나 FSM 을 다시 띄웠을 때 이 패널을 맞춘다')
        btn_speed_read.clicked.connect(self._read_speed)
        speed_row.addWidget(btn_speed_read)
        speed_col.addLayout(speed_row)
        self._lbl_speed = QLabel('현재 속도: (읽는 중 — task_manager 가 떠 있는지 확인)')
        self._lbl_speed.setWordWrap(True)
        speed_col.addWidget(self._lbl_speed)
        outer.addWidget(speed_box)

        task_box = QGroupBox('작업')
        task_row = QHBoxLayout(task_box)
        # '홈'은 IDLE 에서만 먹는다(/pick/home) — '리셋'(SAFE_STOP 전용)과 상태가 다르다.
        for text, srv in (('시작', '/pick/start'), ('승인', '/pick/approve'),
                          ('리셋', '/pick/reset'), ('홈', '/pick/home')):
            task_row.addWidget(self._make_button(node, text, Trigger, srv))
        outer.addWidget(task_box)

        stop_box = QGroupBox('정지')
        stop_row = QHBoxLayout(stop_box)
        for text, srv in (('중단(ABORT)', '/pick/abort'), ('즉시정지', '/safety/stop')):
            btn = self._make_button(node, text, Trigger, srv)
            btn.setStyleSheet('background-color:#c0392b; color:white; font-weight:bold;')
            stop_row.addWidget(btn)
        outer.addWidget(stop_box)

        safe_box = QGroupBox('안전모드 (사람이 팔을 손으로 옮길 때)')
        safe_row = QHBoxLayout(safe_box)
        safe_row.addWidget(self._make_button(
            node, '안전모드 진입(backdrive)', Trigger, '/safety/enter_backdrive'))
        safe_row.addWidget(self._make_button(
            node, '안전모드 해제', Trigger, '/safety/exit_backdrive'))
        outer.addWidget(safe_box)
        warn = QLabel('⚠️ backdrive는 이 도구에서 실기 검증된 적이 없다 — 처음 쓸 때는 '
                      '비상정지 버튼을 손 닿는 곳에 두고 저속·저위험 자세에서 먼저 확인할 것.')
        warn.setWordWrap(True)
        warn.setStyleSheet('color:#c0392b;')
        outer.addWidget(warn)

        # 로봇 bringup/MoveIt을 안 건드리고 그리퍼만 복구한다. OnRobotRGControllerServer.py
        # 가 이미 만들어둔 /onrobot/restartPower 를 쓴다(mode:=real 일 때만 뜬다) — Modbus로
        # Compute Box 전원만 끊었다 붙이는 명령이라 드라이버 노드 자체는 안 죽는다. 노드를
        # 껐다 켜는 방식(재프로세스 실행)은 일부러 안 썼다: rqt 플러그인에서 launch 가 관리하는
        # 프로세스를 개별로 죽이려면 별도 프로세스 관리가 필요해지는데, 이미 하드웨어가
        # 이 상황을 위한 명령을 노출해두고 있어 그걸로 충분하다.
        gripper_box = QGroupBox('그리퍼 (안전스위치로 멈췄을 때)')
        gripper_row = QHBoxLayout(gripper_box)
        gripper_row.addWidget(self._make_button(
            node, '그리퍼 파워사이클', Trigger, '/onrobot/restartPower'))
        outer.addWidget(gripper_box)

        outer.addWidget(self._lbl_result)
        outer.addStretch(1)
        context.add_widget(self._widget)

        self._sub_fsm = node.create_subscription(String, '/pick/state', self._on_fsm, 10)
        self._sub_robot = node.create_subscription(
            Int8, '/pick/robot_state_code', self._on_robot, 10)
        self._sub_target = node.create_subscription(
            String, '/pick/target_active', self._on_target, TARGET_QOS)
        self._sub_place = node.create_subscription(
            String, '/pick/place_location_active', self._on_place, TARGET_QOS)
        # yolo_seg_node 가 프레임마다 내는 "라벨 -> 클래스 이름". 컨테이너 쪽에서 뜨므로
        # 이 패널만 켜고 YOLO 를 안 띄우면 목록은 비어 있는 게 정상이다.
        self._sub_classes = node.create_subscription(
            String, '/yolo_seg/classes', self._on_classes, 10)

        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh)
        self._timer.start(200)

    # ── ROS 콜백 (ROS 스레드에서 실행 — 변수에만 쓴다) ──────────
    def _on_fsm(self, msg):
        self._fsm_state = msg.data

    def _on_robot(self, msg):
        self._robot_state_code = int(msg.data)
        self._robot_state = ROBOT_STATE_NAMES.get(self._robot_state_code,
                                                   f'UNKNOWN({self._robot_state_code})')

    def _on_target(self, msg):
        self._active_target = msg.data

    def _on_place(self, msg):
        self._active_place = msg.data

    def _on_classes(self, msg):
        """`{"objects": [{"label":101,"class":"apple","conf":0.9}, ...]}` -> 이름별 개수.

        깨진 JSON 에 패널이 죽으면 안 된다 — 이건 표시용이고, 못 읽으면 목록만 비면 된다.
        """
        try:
            objs = json.loads(msg.data).get('objects', [])
            counts = {}
            for o in objs:
                name = str(o.get('class', '?'))
                counts[name] = counts.get(name, 0) + 1
        except (ValueError, AttributeError):
            return
        self._detected = sorted(counts.items())
        self._detected_at = time.monotonic()

    # ── Qt 타이머 (GUI 스레드에서 실행 — 위젯은 여기서만 건드린다) ──
    def _refresh(self):
        self._lbl_fsm.setText(f'FSM 상태: {self._fsm_state}')
        unsafe = self._robot_state_code in UNSAFE_STATES
        self._lbl_robot.setText(f'로봇 상태: {self._robot_state}' + ('  ⚠️ 안전정지류' if unsafe else ''))
        self._lbl_robot.setStyleSheet('color:#c0392b; font-weight:bold;' if unsafe else '')
        self._refresh_target()
        self._lbl_place.setText('현재 위치: ' + (
            '(수신 대기 — task_manager 가 떠 있는지 확인)' if self._active_place is None
            else self._active_place))
        # task_manager 가 우리보다 늦게 떠도 한 번은 현재값을 맞춘다. 성공하면 다시 안 부른다
        # (주기적으로 폴링하면 서비스 호출만 쌓인다 — 사람이 바꾸면 [현재값 읽기]를 쓴다).
        if not self._speed_synced and self.get_param_cli.service_is_ready():
            self._read_speed()
        self._pump_params()

        now = time.monotonic()
        still_pending = []
        for label, fut, deadline in self._pending:
            if fut.done():
                res = fut.result()
                if res is None:
                    self._lbl_result.setText(f'{label}: 응답 없음')
                else:
                    self._lbl_result.setText(
                        f'{label}: {"OK" if res.success else "실패"} — {res.message}')
                continue
            if now > deadline:
                self._lbl_result.setText(f'{label}: {PENDING_TIMEOUT_SEC:.0f}s 시간초과 — 응답 없음')
                continue
            still_pending.append((label, fut, deadline))
        self._pending = still_pending

    def _refresh_target(self):
        """콤보 목록·타겟 표시를 갱신한다. GUI 스레드(QTimer)에서만 부른다."""
        stale = (time.monotonic() - self._detected_at) > CLASSES_STALE_SEC
        names = [] if (stale or not self._detected) else [n for n, _ in self._detected]
        # 🔴 사람이 상자에 손을 대고 있는 동안에는 목록을 갈아끼우지 않는다.
        #    YOLO 탐지는 프레임마다 깜빡여서(사과가 한 컷 빠지면 목록이 바뀐다) 이 갱신이
        #    초당 여러 번 돈다. clear()+setEditText() 는 커서를 끝으로 보내므로, 안 막으면
        #    'app|le' 처럼 중간을 고치는 동안 커서가 계속 튀어 글자를 못 친다.
        #    드롭다운이 열려 있을 때 갈아끼우면 고르려던 항목이 손에서 사라지기도 한다.
        #    editable 콤보는 내부 QLineEdit 이 포커스를 받는다 — 둘 다 본다.
        edit = self._combo.lineEdit()
        busy = (self._combo.hasFocus() or (edit is not None and edit.hasFocus())
                or self._combo.view().isVisible())
        if names != self._shown_classes and not busy:
            typed = self._combo.currentText()
            self._combo.blockSignals(True)
            self._combo.clear()
            self._combo.addItems(names)
            self._combo.setEditText(typed)
            self._combo.blockSignals(False)
            self._shown_classes = names

        if self._detected_at == 0.0:
            seen = '탐지: (/yolo_seg/classes 수신 없음 — YOLO 가 떠 있는지 확인)'
        elif stale:
            seen = f'탐지: (끊김 — {time.monotonic() - self._detected_at:.0f}s 째 프레임 없음)'
        elif not self._detected:
            seen = '탐지: 없음'
        else:
            seen = '탐지: ' + ', '.join(f'{n}×{c}' for n, c in self._detected)
        if self._active_target is None:
            cur = '(수신 대기 — task_manager 가 떠 있는지 확인)'
        else:
            cur = self._active_target or AUTO_LABEL
        self._lbl_target.setText(f'현재 타겟: {cur}\n{seen}')

    def _set_target(self, text: str):
        """`/pick/target` 발행. FSM 은 다음 /pick/start 부터 이 값을 쓴다."""
        self.target_pub.publish(String(data=text))
        self._lbl_result.setText(
            f"타겟 지정: {text or AUTO_LABEL} — 진행 중인 작업에는 적용되지 않는다. "
            '다음 [시작] 부터다')

    def _set_place(self, value: str):
        """`/pick/place_location` 발행. FSM 은 다음 /pick/start 부터 이 값을 쓴다."""
        if value in UNVERIFIED_PLACE_LOCATIONS and QMessageBox.question(
                self._widget, f"'{value}' 선택",
                f"place_{value}_joints_deg 는 아직 teach 된 적 없다(홈 자세를 임시로 복사해둔 "
                '값). 다음 [시작]부터 홈 자세에서 그리퍼가 열려 물체가 떨어질 수 있다.\n'
                '계속할까요?') != QMessageBox.Yes:
            return
        self.place_pub.publish(String(data=value))
        self._lbl_result.setText(
            f'내려놓을 위치 지정: {value} — 진행 중인 작업에는 적용되지 않는다. '
            '다음 [시작] 부터다')

    # ── 속도 ────────────────────────────────────────────────
    def _read_speed(self):
        """task_manager 의 현재 vel/acc 를 읽어 스핀박스에 채운다 (비동기)."""
        if not self.get_param_cli.service_is_ready():
            self._lbl_speed.setText(
                f'현재 속도: (읽기 실패 — {TASK_MANAGER_NODE} 가 떠 있는지 확인)')
            return
        req = GetParameters.Request(names=list(SPEED_PARAMS))
        fut = self.get_param_cli.call_async(req)
        self._param_futs.append(('get', fut, time.monotonic() + PENDING_TIMEOUT_SEC))

    def _apply_speed(self):
        """스핀박스 값을 set_parameters 로 밀어 넣는다. 다음 이동 구간부터 적용된다."""
        vals = {k: float(self._spins[k].value()) for k in SPEED_PARAMS}
        # 올릴 때만 묻는다. 지금 값을 모르면(아직 못 읽었으면) 안전한 쪽으로 물어본다.
        raising = any(v > FAST_CONFIRM_SCALE for v in vals.values())
        if raising and QMessageBox.question(
                self._widget, '속도 상향',
                f'vel={vals["vel_scale"]:.2f}, acc={vals["acc_scale"]:.2f} '
                f'— {FAST_CONFIRM_SCALE:.2f} 를 넘습니다.\n'
                '다음 이동 구간부터 팔이 더 빠르게 움직입니다 — 비상정지 버튼을 손 닿는 곳에 '
                '두고 진행하세요. 계속할까요?') != QMessageBox.Yes:
            return
        if not self.set_param_cli.service_is_ready():
            self._lbl_result.setText(f'속도 적용: {TASK_MANAGER_NODE}/set_parameters 서비스 없음')
            return
        req = SetParameters.Request(parameters=[
            Parameter(name=k, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=v))
            for k, v in vals.items()])
        fut = self.set_param_cli.call_async(req)
        self._param_futs.append(('set', fut, time.monotonic() + PENDING_TIMEOUT_SEC))
        self._lbl_result.setText('속도 적용: 요청 전송함…')

    def _pump_params(self):
        """set/get 응답 처리. GUI 스레드(QTimer)에서만 부른다."""
        now = time.monotonic()
        still = []
        for kind, fut, deadline in self._param_futs:
            if fut.done():
                self._on_param_done(kind, fut.result())
                continue
            if now > deadline:
                self._lbl_result.setText(f'속도 {kind}: {PENDING_TIMEOUT_SEC:.0f}s 시간초과')
                continue
            still.append((kind, fut, deadline))
        self._param_futs = still

    def _on_param_done(self, kind, res):
        if res is None:
            self._lbl_result.setText(f'속도 {kind}: 응답 없음')
            return
        if kind == 'get':
            # 선언 안 된 이름은 PARAMETER_NOT_SET(0) 으로 돌아온다 — 0.0 으로 읽으면 안 된다.
            got = {}
            for name, pv in zip(SPEED_PARAMS, res.values):
                if pv.type == ParameterType.PARAMETER_DOUBLE:
                    got[name] = pv.double_value
            if len(got) != len(SPEED_PARAMS):
                self._lbl_speed.setText(
                    f'현재 속도: (읽기 실패 — {TASK_MANAGER_NODE} 에 '
                    f'{"/".join(SPEED_PARAMS)} 가 없다)')
                return
            self._speed_synced = True
            for name, v in got.items():
                spin = self._spins[name]
                # 사람이 지금 손대고 있는 칸은 덮지 않는다 (타겟 콤보와 같은 이유).
                if not spin.hasFocus():
                    spin.blockSignals(True)
                    spin.setValue(v)
                    spin.blockSignals(False)
            self._lbl_speed.setText(
                '현재 속도: ' + ', '.join(f'{k}={v:.2f}' for k, v in got.items()))
            return
        ok = all(r.successful for r in res.results)
        why = '; '.join(r.reason for r in res.results if not r.successful)
        self._lbl_result.setText(
            '속도 적용: OK — 지금 실행 중인 구간이 아니라 **다음 이동부터** 적용된다'
            if ok else f'속도 적용: 실패 — {why}')
        self._read_speed()      # 노드가 실제로 뭘 갖게 됐는지로 표시를 맞춘다

    # ── 버튼 ────────────────────────────────────────────────
    def _make_button(self, node, label, srv_type, srv_name):
        # 클라이언트는 버튼당 한 번만 만든다. 클릭마다 새로 만들면 DDS 리소스가
        # 계속 쌓이고 아무도 destroy 하지 않는다 — 이 패널은 반복 클릭이 정상 사용이다.
        client = node.create_client(srv_type, srv_name)
        self._clients.append(client)
        btn = QPushButton(label)

        def on_click():
            if label in CONFIRM:
                if QMessageBox.question(
                        self._widget, label, CONFIRM[label]) != QMessageBox.Yes:
                    return
            if not client.service_is_ready():
                self._lbl_result.setText(f'{label}: {srv_name} 서비스 없음')
                return
            fut = client.call_async(srv_type.Request())
            self._pending.append((label, fut, time.monotonic() + PENDING_TIMEOUT_SEC))
            self._lbl_result.setText(f'{label}: 요청 전송함…')
        btn.clicked.connect(on_click)
        return btn

    # ── rqt 생명주기 ─────────────────────────────────────────
    def shutdown_plugin(self):
        self._timer.stop()
        try:
            self._sub_fsm.destroy()
            self._sub_robot.destroy()
            self._sub_target.destroy()
            self._sub_place.destroy()
            self._sub_classes.destroy()
            self.target_pub.destroy()
            self.place_pub.destroy()
            for client in self._clients:
                client.destroy()
        except Exception:                                  # noqa: BLE001
            pass
