#!/usr/bin/env python3
"""Console for talking to the robot: camera, chat, scene table, stop button.

The GUI holds exactly one piece of decision logic, and it is here because it
must not wait for anything: the stop keyword. A "정지" typed or spoken here is
matched locally and published straight to the robot, bypassing STT-to-LLM
round-trips entirely. Since 2026-08-12 it lands on **pause**, not e-stop --
reversible, so being broad about what counts as "멈춰" costs nothing.

Everything else -- what the words meant, which object was meant, whether to ask
back -- is the agent's to decide.

Topics
------
publish:
  /vla/user_utterance   what the user said
  /vla/robot/pause      the stop *word* -- reversible (contract §15)
  /vla/robot/stow       tidy up and finish, before shutting the pipeline down
  /vla/estop            the red button / ESC -- destructive
subscribe:
  /vla/perception/annotated_image
  /vla/scene
  /vla/agent/reply
  /vla/robot/state
  /vla/pick_status        cobot2_ws rqt 패널 미러링 (vla-bridge-contract.md #12)
"""

from __future__ import annotations

import base64
import json
import os
import queue
import re
import signal
import subprocess
import threading
import time
import tkinter as tk
import webbrowser
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vla_interfaces.msg import AgentReply, RobotState, SceneSnapshot

from vla_system.process_guard import (
    PIPELINE_PATTERN,
    REALSENSE_PATTERN,
    escalate_termination,
    find_existing_pipeline_pids,
)

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------

UTTERANCE_TOPIC = "/vla/user_utterance"
# Two stops, deliberately separate (2026-08-12, contract §15).
#
#   PAUSE  "멈춰" -- reversible. The arm stops where it is, keeps holding
#          whatever it is holding, and waits. "계속해" carries on.
#   ESTOP  the red button / ESC -- destructive. abort -> SAFE_STOP, needing
#          /pick/reset and a HOME round trip to come back.
#
# Both bypass the LLM entirely (invariant I4). The split exists because they
# used to be one thing: every "그만" a user muttered cost the whole cycle, so
# people learned not to say it -- which is the opposite of what a stop word is
# for. Keep the two buttons visually different for the same reason.
PAUSE_TOPIC = "/vla/robot/pause"
# Tidy up and finish (contract §15). Not a stop -- it *moves*: puts down
# what is held at the place location, then goes home. The "VLA 정지" button
# calls this before killing the processes.
STOW_TOPIC = "/vla/robot/stow"
ESTOP_TOPIC = "/vla/estop"
ANNOTATED_IMAGE_TOPIC = "/vla/perception/annotated_image"
SCENE_TOPIC = "/vla/scene"
REPLY_TOPIC = "/vla/agent/reply"
ROBOT_STATE_TOPIC = "/vla/robot/state"
# cobot2_ws's vla_command_node mirrors its rqt panel here (String/JSON), on
# every /pick/state change + a 1 Hz heartbeat (vla-bridge-contract.md #12).
# Read-only -- this GUI never publishes to it.
PICK_STATUS_TOPIC = "/vla/pick_status"
# Contract #12: heartbeat is 1 Hz, so treat >3 missed beats as "cobot2_ws
# stopped answering", not a stamp_ns comparison (no clock sync assumed
# between the two PCs).
PICK_STATUS_STALE_S = 3.0

# cobot2_ws의 grasp_bridge_node가 기본으로 띄우는 viser 웹뷰어 (live_viz_port 기본값,
# graspgenx_perception/grasp_bridge_node.py). ROS 토픽이 아니라 WebSocket/WebGL
# 렌더러라 이 GUI에 임베드할 방법이 없다 -- 브라우저 새 창으로만 연다(2026-08-11).
GRASPGENX_VIZ_URL = "http://localhost:8080"

# Matched before anything else happens to the user's words. Deliberately broad:
# a stop that fires when the user did not quite mean it costs one interrupted
# motion, while a stop that fails to fire costs a collision.
#
# 2026-08-12: this now goes to PAUSE_TOPIC, not ESTOP_TOPIC. Being broad was
# only defensible while a false positive cost "one interrupted motion" -- it
# actually cost the whole cycle, because the word landed on abort and dropped
# the arm into SAFE_STOP, needing /pick/reset + a HOME round trip to come back.
# A pause is genuinely cheap to be wrong about: "계속해" resumes it.
STOP_PATTERN = re.compile(
    r"정지|멈춰|멈춤|그만|중지|스톱|스탑|\bstop\b|\bhalt\b", re.IGNORECASE
)

SAMPLE_RATE = 16_000
MIN_RECORD_SECONDS = 0.35
AUDIO_PATH = Path.home() / ".ros" / "vla_system" / "gui_last_command.wav"
# Every line the pipeline prints, kept on disk. The chat only shows the lines
# matching _INTERESTING_LOG_TOKENS, and ros2 launch's own log directory holds
# nothing when its output is piped -- so without this file a grasp that missed
# leaves no record of what was commanded.
PIPELINE_LOG_PATH = Path.home() / ".ros" / "vla_system" / "pipeline.log"

STT_MODEL = os.getenv("OPENAI_STT_MODEL", "gpt-4o-transcribe")

BASE_LAUNCH_COMMAND = [
    "ros2",
    "launch",
    "vla_system",
    "vla_system.launch.py",
]


def build_launch_command(
    *, pick_bridge: bool, skill_tier: bool, perception: bool
) -> list[str]:
    """체크박스 상태 -> `ros2 launch` 인자.

    App 밖의 순수 함수인 이유: 여기서 인자 하나가 빠져도 예외가 나지 않는다.
    launch 기본값이 조용히 이기고, 증상은 한참 떨어진 곳에서 "규칙이 기억되지
    않는다"로만 보인다 -- 2026-08-11에 `skill_tier_enabled`가 정확히 그렇게
    빠져 있었다. 화면 없이 검사할 수 있어야 그걸 테스트가 잡는다.

    반대 방향도 마찬가지다. launch가 모르는 인자를 보내도 `ros2 launch`는
    오류를 내지 않고 무시한다 -- `enable_wrist_grasp`가 그렇게 남아 있었다
    (손목 파지 노드가 삭제된 뒤에도). test_gui_launch_arguments.py가 여기서
    나가는 이름을 launch 선언과 대조하는 이유다.
    """
    return BASE_LAUNCH_COMMAND + [
        f"enable_pick_bridge:={'true' if pick_bridge else 'false'}",
        # pick_bridge 켜짐 = cobot2_ws 쪽 launch가 카메라를 이미 잡고 있다는 전제
        # (README §4) -- 여기서 또 열면 V4L2 충돌 위험. 꺼짐 = 이 ws 단독 실행이니
        # 이 ws가 카메라를 연다(예전 기본값). "카메라 인식"을 끈 경우는 어느
        # 쪽이든 열지 않는다 -- 그 체크박스는 카메라가 아예 없는 상태(가짜 무대)를
        # 뜻하고, RealSense만 살아 있으면 launch가 장치를 못 찾고 죽는다.
        f"enable_realsense:={'true' if perception and not pick_bridge else 'false'}",
        f"skill_tier_enabled:={'true' if skill_tier else 'false'}",
        f"enable_perception:={'true' if perception else 'false'}",
    ]

_INTERESTING_LOG_TOKENS = (
    "error",
    "warn",
    # Grasp diagnostics. These are INFO, and without them the chat shows a bare
    # "완료했습니다" for a grasp that physically missed -- the match distance and
    # the commanded target are the only way to tell which calibration is off.
    "matched ",
    "grasp plan",
    "executing wrist",
    "observe from",
    "grasp at",
    "graspgenx ready",
    "planner ready",
    "robot motion",
    "stop:",
    "traceback",
    "exception",
    "died",
    "fatal",
    "critical",
    "cannot connect",
    "no module named",
)
_LOG_PREFIX_RE = re.compile(r"^\[([^\]]+)\]")

# UI palette
BG = "#17191c"
PANEL = "#22252a"
PANEL_2 = "#1c1f23"
FG = "#e8eaed"
MUTED = "#9aa0a6"
ACCENT = "#5cc8ff"
GREEN = "#64d98b"
YELLOW = "#f2c94c"
RED = "#ff6b6b"
USER_BG = "#243447"
AI_BG = "#24352d"
SYSTEM_BG = "#2b2d31"
ASK_BG = "#3a3320"


def ros_image_to_rgb(message: Image) -> np.ndarray:
    """Convert common ROS RGB images without cv_bridge."""
    if message.encoding not in ("bgr8", "rgb8"):
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    row = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step
    )
    image = row[:, : message.width * 3].reshape(message.height, message.width, 3)
    if message.encoding == "bgr8":
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image.copy()


def rgb_to_tk_photo(rgb: np.ndarray, max_width: int, max_height: int) -> tk.PhotoImage:
    """Resize an RGB array and create a Tk PhotoImage without Pillow."""
    height, width = rgb.shape[:2]
    if width <= 0 or height <= 0:
        raise ValueError("empty image")

    scale = min(max_width / width, max_height / height)
    if scale <= 0:
        scale = 1.0
    scale = min(scale, 1.5)
    target_w = max(1, int(width * scale))
    target_h = max(1, int(height * scale))

    if target_w != width or target_h != height:
        interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        rgb = cv2.resize(rgb, (target_w, target_h), interpolation=interpolation)

    # Tk 8.6 supports PNG. Encoding to PNG avoids a Pillow dependency.
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode(".png", bgr)
    if not ok:
        raise RuntimeError("failed to encode GUI image")
    return tk.PhotoImage(data=base64.b64encode(encoded.tobytes()), format="png")


def crop_object(rgb: np.ndarray, box, padding: int = 18) -> np.ndarray:
    """Cut one detection out of the frame, with a little context around it."""
    height, width = rgb.shape[:2]
    x_min = max(0, int(box[0]) - padding)
    y_min = max(0, int(box[1]) - padding)
    x_max = min(width, int(box[2]) + padding)
    y_max = min(height, int(box[3]) + padding)
    if x_max - x_min < 4 or y_max - y_min < 4:
        raise ValueError("crop is too small")
    return rgb[y_min:y_max, x_min:x_max].copy()


def robot_state_line(state: dict | None) -> str:
    if state is None:
        return "로봇 상태 대기 중"
    status = {
        "idle": "대기",
        "moving": "동작 중",
        "holding": "물체를 든 채 대기",
        # waiting_place: the FSM lifted the object and parked in
        # WAIT_PLACE_TARGET waiting for a destination. Distinct from "holding"
        # on purpose -- holding means the arm is still on its way somewhere,
        # this one means it has stopped and is waiting on *us*. Without the
        # entry the raw English string leaked to the operator (2026-08-12).
        "waiting_place": "놓을 위치 지정 대기 ✋",
        "waiting_approval": "사람 승인 대기 ✋",
        # 되돌릴 수 있는 정지(계약 §15). "오류"가 아니라는 게 중요하다 -- 사람이
        # 일부러 세운 것이고, 다음 지시 한 마디면 이어진다. 빨간 error 로 보이면
        # 사용자가 뭔가 고장난 줄 알고 리셋을 누른다(그건 파괴적 경로다).
        "paused": "✋ 멈춤 — '계속해' 또는 다음 지시 대기",
        "error": "오류",
    }.get(state["status"], state["status"])
    holding = state["holding_class"] or "없음"
    mode = "실제 모션" if state["motion_enabled"] else "DRY-RUN"
    tail = ""
    if state.get("current_action") and state.get("details"):
        # A pick is in flight -- details carries the live FSM step label
        # (vla_pick_bridge fsm_state_callback), which is what the user wants to
        # see instead of a bare "동작 중".
        status = f"{status} · {state['details']}"
    elif state["last_action"]:
        tail = f" | 최근: {state['last_action']} → {state['last_result']}"
    return f"{status} | 들고 있음: {holding} | {mode}{tail}"


def pick_status_line(status: dict | None, age_s: float) -> str:
    """cobot2_ws rqt panel mirror (/vla/pick_status, contract #12) -> one line.

    ``age_s`` is time since the last message, not a stamp_ns diff -- the two
    PCs' clocks aren't assumed to be in sync (same rule the contract puts on
    /vla/pick_result). Past PICK_STATUS_STALE_S with nothing received, this
    node cannot tell "cobot2_ws is idle" from "cobot2_ws died" -- say so
    instead of showing a frozen last-known state as if it were live.
    """
    if status is None:
        return "cobot2_ws 연결 대기 중"
    if age_s > PICK_STATUS_STALE_S:
        return f"cobot2_ws 응답 없음 ({age_s:.0f}s) — 마지막 상태: {status.get('fsm', '?')}"
    fsm = status.get("fsm", "?")
    robot = status.get("robot", "?")
    tail = ""
    if status.get("waiting_approval"):
        tail += " | 승인 대기 ✋ (cobot2_ws 로컬에서만 처리)"
    if status.get("unsafe"):
        tail += " | ⚠ 로봇 안전정지"
    return f"cobot2_ws: {fsm} | robot: {robot}{tail}"


# -----------------------------------------------------------------------------
# ROS bridge
# -----------------------------------------------------------------------------


class GuiRosBridge(Node):
    """ROS callbacks only store snapshots; Tk reads them from the GUI thread."""

    def __init__(self, events: queue.Queue):
        super().__init__("vla_gui")
        self.events = events
        self._lock = threading.Lock()

        self._latest_frame: np.ndarray | None = None
        self._keep_frame: np.ndarray | None = None
        self._latest_scene: dict | None = None
        self._latest_state: dict | None = None
        self._latest_pick_status: dict | None = None
        self._pick_status_monotonic: float = 0.0

        self.frame_count = 0
        self.last_frame_monotonic = 0.0

        stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.utterance_publisher = self.create_publisher(
            String, UTTERANCE_TOPIC, command_qos
        )
        self.estop_publisher = self.create_publisher(String, ESTOP_TOPIC, command_qos)
        self.pause_publisher = self.create_publisher(String, PAUSE_TOPIC, command_qos)
        self.stow_publisher = self.create_publisher(String, STOW_TOPIC, command_qos)

        self.create_subscription(
            Image, ANNOTATED_IMAGE_TOPIC, self._image_callback, stream_qos
        )
        self.create_subscription(
            SceneSnapshot, SCENE_TOPIC, self._scene_callback, stream_qos
        )
        self.create_subscription(
            AgentReply, REPLY_TOPIC, self._reply_callback, command_qos
        )
        self.create_subscription(
            RobotState, ROBOT_STATE_TOPIC, self._state_callback, latched_qos
        )
        # command_qos here matches cobot2_ws's COMMAND_QOS on the publishing
        # side (RELIABLE/VOLATILE/depth 10) -- a mismatch is silent, not an
        # error (same pitfall vla_pick_bridge_node's docstring calls out).
        self.create_subscription(
            String, PICK_STATUS_TOPIC, self._pick_status_callback, command_qos
        )

    # ------------------------------------------------------------- callbacks

    def _image_callback(self, message: Image) -> None:
        try:
            frame = ros_image_to_rgb(message)
        except Exception as exc:
            self.get_logger().warning(f"GUI image decode failed: {exc}")
            return
        with self._lock:
            self._latest_frame = frame
            # A second reference the video loop does not consume, so a
            # clarification crop can still be taken after the frame was drawn.
            self._keep_frame = frame
            self.frame_count += 1
            self.last_frame_monotonic = time.monotonic()

    def _scene_callback(self, message: SceneSnapshot) -> None:
        payload = {
            "calibration_ok": bool(message.calibration_ok),
            "objects": [
                {
                    "id": scene_object.id,
                    "class_name": scene_object.class_name,
                    "confidence": float(scene_object.confidence),
                    "color": scene_object.color or "unknown",
                    # This ws's own perception coordinate availability, for the
                    # debug panel only. NOT pickability: cobot2_ws computes the
                    # grasp coordinate itself, so every visible object is
                    # pickable regardless of this (conversation.py sends
                    # pickable=True to the model). Named accordingly so the two
                    # meanings don't get conflated again.
                    "has_position": bool(scene_object.position_valid),
                    "bbox": (
                        float(scene_object.x_min),
                        float(scene_object.y_min),
                        float(scene_object.x_max),
                        float(scene_object.y_max),
                    ),
                    "position": (
                        float(scene_object.position_base.x),
                        float(scene_object.position_base.y),
                        float(scene_object.position_base.z),
                    ),
                }
                for scene_object in message.objects
            ],
        }
        with self._lock:
            self._latest_scene = payload

    def _reply_callback(self, message: AgentReply) -> None:
        self.events.put(
            (
                "reply",
                {
                    "kind": message.kind,
                    "text": message.text,
                    "focus": list(message.focus_object_ids),
                },
            )
        )

    def _state_callback(self, message: RobotState) -> None:
        payload = {
            "status": message.status,
            "holding_id": message.holding_object_id,
            "holding_class": message.holding_class_name,
            "current_action": message.current_action,
            "last_action_id": message.last_action_id,
            "last_action": message.last_action,
            "last_result": message.last_result,
            "details": message.details,
            "motion_enabled": bool(message.motion_enabled),
        }
        with self._lock:
            self._latest_state = payload
        self.events.put(("robot_state", payload))

    def _pick_status_callback(self, message: String) -> None:
        try:
            doc = json.loads(message.data)
        except (ValueError, TypeError):
            self.get_logger().warning(f"could not parse /vla/pick_status: {message.data!r}")
            return
        if not isinstance(doc, dict):
            return
        with self._lock:
            self._latest_pick_status = doc
            self._pick_status_monotonic = time.monotonic()

    # ------------------------------------------------------------------ reads

    def take_frame(self) -> np.ndarray | None:
        with self._lock:
            frame = self._latest_frame
            self._latest_frame = None
            return frame

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "scene": self._latest_scene,
                "state": self._latest_state,
                "keep_frame": self._keep_frame,
                "frame_count": self.frame_count,
                "last_frame_monotonic": self.last_frame_monotonic,
                "pick_status": self._latest_pick_status,
                "pick_status_monotonic": self._pick_status_monotonic,
            }

    # --------------------------------------------------------------- publish

    def publish_utterance(self, text: str) -> None:
        self.utterance_publisher.publish(String(data=text))

    def publish_pause(self, reason: str) -> None:
        self.pause_publisher.publish(String(data=reason))

    def publish_stow(self, reason: str) -> None:
        self.stow_publisher.publish(String(data=reason))

    def publish_estop(self, reason: str) -> None:
        self.estop_publisher.publish(String(data=reason))


# -----------------------------------------------------------------------------
# GUI
# -----------------------------------------------------------------------------


class VLAApp:
    def __init__(self, root: tk.Tk, bridge: GuiRosBridge, events: queue.Queue):
        self.root = root
        self.bridge = bridge
        self.events = events

        self.transcriber = None
        self.busy = False

        self.audio_stream = None
        self.audio_frames: list[np.ndarray] = []
        self.recording = False
        self.record_started = 0.0

        self.pipeline_process: subprocess.Popen | None = None
        self.photo: tk.PhotoImage | None = None
        self.clarify_photos: list[tk.PhotoImage] = []

        self.last_frame_count = 0
        self.last_fps_time = time.monotonic()
        self.current_fps = 0.0
        self.last_state_key: tuple | None = None

        # 2026-08-11: 기본을 cobot2_ws 연동으로 바꿈 -- 이 GUI가 최종적으로 존재하는
        # 이유가 pick_fsm과 물려 돌리는 것이라, 단독 모드(vla_robot)가 예외가 되어야
        # 한다. 켜져 있으면 enable_pick_bridge:=true + enable_realsense:=false(카메라는
        # cobot2_ws 쪽 launch가 이미 잡고 있다는 전제, README §4)를 같이 보낸다.
        self.pick_bridge_var = tk.BooleanVar(value=True)
        # 규칙 계층(Tier 1). launch/system.yaml 기본값은 false지만 이 GUI에서는
        # 켜고 시작한다 -- 이 창이 존재하는 이유가 규칙 계층을 사람이 만져 보는
        # 것이고, 꺼졌을 때의 동작(전부 LLM으로 감)은 예전과 똑같아서 켜는 쪽이
        # 더 위험하지도 않다. 2026-08-11: 이 인자를 안 보내던 탓에 GUI로 켠
        # 파이프라인은 항상 Tier 1이 꺼진 채였고, "앞으로 사과 집지 마"가 재시작
        # 후 사라지는 것으로 나타났다(A2에는 세션을 넘는 기억이 없다).
        self.skill_tier_var = tk.BooleanVar(value=True)
        # 카메라 인식. GPU가 없는 개발 머신이나 eval/dryrun_stage.py로 무대를
        # 대신 세울 때는 꺼야 한다 -- perception_node는 YOLO를 cuda:0에 올리려다
        # 죽고, dryrun_stage와 동시에 뜨면 /vla/scene에 둘이 발행해 서로 덮는다.
        self.perception_var = tk.BooleanVar(value=True)

        self._configure_window()
        self._configure_style()
        self._build_ui()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.bind("<Escape>", lambda _event: self.emergency_stop("ESC 키"))
        self.root.after(30, self._drain_events)
        self.root.after(33, self._refresh_video)
        self.root.after(200, self._refresh_status)

        self.log_system(
            "준비 완료. VLA를 시작한 뒤 말을 걸어보세요. "
            "'멈춰'라고 말하면 LLM을 거치지 않고 로봇이 즉시 멈춥니다 — "
        "'계속해'로 이어서 합니다. ESC/비상정지는 되돌리려면 리셋이 필요합니다.",
        )

    # ------------------------------------------------------------------ UI

    def _configure_window(self) -> None:
        self.root.title("VLA Robot Console")
        self.root.geometry("1460x900")
        self.root.minsize(1160, 720)
        self.root.configure(bg=BG)

    def _configure_style(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TFrame", background=BG)
        style.configure("Panel.TFrame", background=PANEL)
        style.configure("TLabel", background=PANEL, foreground=FG)
        style.configure("Muted.TLabel", background=PANEL, foreground=MUTED)
        style.configure(
            "Title.TLabel",
            background=BG,
            foreground=ACCENT,
            font=("TkDefaultFont", 14, "bold"),
        )
        style.configure(
            "Section.TLabel",
            background=PANEL,
            foreground=FG,
            font=("TkDefaultFont", 11, "bold"),
        )
        style.configure("TButton", padding=7)
        style.configure("TCheckbutton", background=PANEL, foreground=FG)
        style.configure(
            "Treeview",
            background=PANEL_2,
            fieldbackground=PANEL_2,
            foreground=FG,
            rowheight=25,
        )
        style.configure("Treeview.Heading", background="#30343a", foreground=FG)

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=3)
        outer.columnconfigure(1, weight=2)
        outer.rowconfigure(1, weight=1)

        header = ttk.Frame(outer)
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))
        header.columnconfigure(1, weight=1)
        ttk.Label(header, text="VLA Robot Console", style="Title.TLabel").grid(
            row=0, column=0, sticky="w"
        )

        controls = ttk.Frame(header)
        controls.grid(row=0, column=1, sticky="e")

        # 두 버튼을 **눈에 띄게 다르게** 만든다 (contract §15). 하나는 되돌릴 수 있고
        # 하나는 아니다. 색·문구가 비슷하면 사람이 급할 때 빨간 쪽을 누르고, 그러면
        # 사이클을 버리게 된다 -- 그 습관이 붙으면 "멈춰"를 아예 안 쓰게 된다.
        self.pause_button = tk.Button(
            controls,
            text="⏸ 멈춰",
            bg="#8a6d1d",
            fg="#ffffff",
            activebackground="#c79a2a",
            activeforeground="#ffffff",
            relief="flat",
            padx=14,
            pady=6,
            font=("TkDefaultFont", 10, "bold"),
            command=lambda: self.pause_robot("멈춤 버튼"),
        )
        self.pause_button.grid(row=0, column=0, padx=(0, 8))

        self.stop_button = tk.Button(
            controls,
            text="🔴 비상정지 (ESC)",
            bg="#8f1d1d",
            fg="#ffffff",
            activebackground=RED,
            activeforeground="#ffffff",
            relief="flat",
            padx=14,
            pady=6,
            font=("TkDefaultFont", 10, "bold"),
            command=lambda: self.emergency_stop("비상정지 버튼"),
        )
        self.stop_button.grid(row=0, column=1, padx=(0, 12))

        # 2026-08-11 제거: "실제 로봇 모션" 체크박스는 영구 비활성(state="disabled")으로
        # 죽어있던 컨트롤이었다 -- cobot2_ws의 pick_fsm이 로봇을 전담하게 되면서 GUI가
        # enable_robot:=true를 보낼 일이 아예 없어졌고, real_robot_var는 항상 False였다.
        # (기존 팀원 코드, 2026-08-11 사용자 승인 후 제거)

        # 기본 켜짐: cobot2_ws pick_fsm이 카메라를 이미 잡고 있다는 전제로
        # enable_pick_bridge:=true + enable_realsense:=false를 함께 보낸다(README §4).
        # 이 ws 카메라로 단독 실행하려면 체크를 끈다 -- 그러면 enable_realsense:=true로
        # 되돌아간다(예전 기본 동작).
        self.pick_bridge_check = ttk.Checkbutton(
            controls,
            text="cobot2_ws FSM 연동",
            variable=self.pick_bridge_var,
        )
        self.pick_bridge_check.grid(row=0, column=1, padx=(0, 8))

        # 끄면 예전 경로 그대로 -- 모든 발화가 LLM으로 가고 규칙은 기억되지 않는다.
        self.skill_tier_check = ttk.Checkbutton(
            controls, text="규칙 계층 (Tier 1)", variable=self.skill_tier_var
        )
        self.skill_tier_check.grid(row=0, column=2, padx=(0, 8))

        # 끄면 카메라 없이 뜬다. eval/dryrun_stage.py로 무대를 대신 세울 때 쓴다.
        self.perception_check = ttk.Checkbutton(
            controls, text="카메라 인식", variable=self.perception_var
        )
        self.perception_check.grid(row=0, column=3, padx=(0, 8))

        # cobot2_ws 쪽 grasp_bridge_node가 띄우는 viser 웹뷰어는 ROS 이미지 토픽이
        # 아니라 WebSocket 렌더러라 이 GUI 안에 못 그린다 -- 새 브라우저 창으로만
        # 연다(2026-08-11, 사용자 확인).
        self.viz_button = ttk.Button(
            controls, text="GraspGenX 뷰어", command=self.open_graspgenx_viewer
        )
        self.viz_button.grid(row=0, column=4, padx=(0, 8))

        self.pipeline_button = ttk.Button(
            controls, text="VLA 시작", command=self.toggle_pipeline
        )
        self.pipeline_button.grid(row=0, column=5)

        # ------------------------------------------------- left: perception

        left = ttk.Frame(outer, style="Panel.TFrame", padding=10)
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(1, weight=4)
        left.rowconfigure(3, weight=2)

        ttk.Label(
            left, text="고정 RealSense / YOLO-seg  (LLM이 보는 화면)", style="Section.TLabel"
        ).grid(row=0, column=0, sticky="w")
        self.video_label = tk.Label(
            left,
            text=f"{ANNOTATED_IMAGE_TOPIC} 대기 중",
            bg="#090a0c",
            fg=MUTED,
            bd=0,
        )
        self.video_label.grid(row=1, column=0, sticky="nsew", pady=(7, 8))

        self.vision_status_var = tk.StringVar(value="장면 데이터 대기 중")
        ttk.Label(left, textvariable=self.vision_status_var, style="Muted.TLabel").grid(
            row=2, column=0, sticky="w", pady=(0, 5)
        )

        # 손목 RealSense 화면(예전 "손목 RealSense + YOLO-seg" 패널)과 GraspGenX
        # 손목 파지 기능(wrist_grasp_node) 모두 제거됨 -- cobot2_ws의 pick_fsm이
        # 유일한 실행 주체가 되면서 robot_node/wrist_grasp_node 스택 전체가
        # 죽은 코드였다(CLAUDE.md #3).

        table_frame = ttk.Frame(left, style="Panel.TFrame")
        table_frame.grid(row=3, column=0, sticky="nsew")
        table_frame.columnconfigure(0, weight=1)
        table_frame.rowconfigure(0, weight=1)

        self.scene_tree = ttk.Treeview(
            table_frame,
            columns=("id", "color", "conf", "coord", "position"),
            show="headings",
            height=8,
        )
        headings = {
            "id": "LLM이 부르는 이름",
            "color": "color",
            "conf": "conf",
            "coord": "좌표 있음",
            "position": "base 좌표 (m)",
        }
        widths = {"id": 150, "color": 90, "conf": 60, "coord": 80, "position": 210}
        for name, label in headings.items():
            self.scene_tree.heading(name, text=label)
            self.scene_tree.column(name, width=widths[name], anchor="center")
        self.scene_tree.grid(row=0, column=0, sticky="nsew")

        scrollbar = ttk.Scrollbar(
            table_frame, orient="vertical", command=self.scene_tree.yview
        )
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.scene_tree.configure(yscrollcommand=scrollbar.set)

        # ----------------------------------------------- right: conversation

        right = ttk.Frame(outer, style="Panel.TFrame", padding=10)
        right.grid(row=1, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)

        ttk.Label(right, text="대화", style="Section.TLabel").grid(
            row=0, column=0, sticky="w"
        )

        self.chat = tk.Text(
            right,
            bg=PANEL_2,
            fg=FG,
            insertbackground=FG,
            relief="flat",
            wrap="word",
            padx=12,
            pady=12,
            state="disabled",
            font=("TkDefaultFont", 11),
        )
        self.chat.grid(row=1, column=0, sticky="nsew", pady=(7, 8))
        self.chat.tag_configure(
            "user_name", foreground=ACCENT, font=("TkDefaultFont", 10, "bold")
        )
        self.chat.tag_configure(
            "assistant_name", foreground=GREEN, font=("TkDefaultFont", 10, "bold")
        )
        self.chat.tag_configure(
            "system_name", foreground=YELLOW, font=("TkDefaultFont", 10, "bold")
        )
        self.chat.tag_configure(
            "user_body", foreground=FG, background=USER_BG, spacing1=3, spacing3=10
        )
        self.chat.tag_configure(
            "assistant_body", foreground=FG, background=AI_BG, spacing1=3, spacing3=10
        )
        self.chat.tag_configure(
            "system_body", foreground=MUTED, background=SYSTEM_BG, spacing1=3, spacing3=10
        )
        self.chat.tag_configure(
            "ask_body", foreground=YELLOW, background=ASK_BG, spacing1=3, spacing3=10
        )

        # Clarification strip: shown only while the agent is waiting for the
        # user to pick between look-alike objects.
        self.clarify_frame = ttk.Frame(right, style="Panel.TFrame")
        self.clarify_frame.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        self.clarify_frame.grid_remove()

        input_box = ttk.Frame(right, style="Panel.TFrame")
        input_box.grid(row=3, column=0, sticky="ew")
        input_box.columnconfigure(0, weight=1)

        self.entry = ttk.Entry(input_box, font=("TkDefaultFont", 11))
        self.entry.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.entry.bind("<Return>", lambda _event: self.submit_text())

        self.send_button = ttk.Button(input_box, text="전송", command=self.submit_text)
        self.send_button.grid(row=0, column=1, padx=(0, 6))

        self.voice_button = tk.Button(
            input_box,
            text="🎙 누르고 말하기",
            bg="#343a40",
            fg=FG,
            activebackground="#4a5057",
            activeforeground="#ffffff",
            relief="flat",
            padx=10,
            pady=6,
        )
        self.voice_button.grid(row=0, column=2)
        self.voice_button.bind("<ButtonPress-1>", self._voice_press)
        self.voice_button.bind("<ButtonRelease-1>", self._voice_release)

        self.robot_status_var = tk.StringVar(value="로봇 상태 대기 중")
        ttk.Label(right, textvariable=self.robot_status_var, style="Muted.TLabel").grid(
            row=4, column=0, sticky="w", pady=(8, 0)
        )

        # cobot2_ws rqt 패널 미러 (vla-bridge-contract.md #12) -- FSM 단계 자체는
        # robot_status_var 가 이미 pick 진행 중일 때 보여준다(fsm_state_callback
        # 경유); 이 줄은 pick 이 없을 때도 살아있는 cobot2_ws↔VLA 연결 자체와
        # waiting_approval/unsafe 를 보여준다.
        self.pick_status_var = tk.StringVar(value="cobot2_ws 연결 대기 중")
        ttk.Label(right, textvariable=self.pick_status_var, style="Muted.TLabel").grid(
            row=5, column=0, sticky="w", pady=(2, 0)
        )

        self.bottom_status_var = tk.StringVar(value="ROS 연결됨 | VLA 파이프라인 대기")
        status = tk.Label(
            outer,
            textvariable=self.bottom_status_var,
            bg="#0f1114",
            fg=MUTED,
            anchor="w",
            padx=10,
            pady=5,
        )
        status.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0))

    # --------------------------------------------------------- conversation

    def log_system(self, text: str) -> None:
        """운영 로그. **대화창이 아니라 터미널로 나간다** (2026-08-12 사용자 결정).

        대화창은 사람과 AI가 주고받은 말만 남긴다. 프로세스 정리·마이크 오류·파이프라인
        기동 같은 것들이 섞이면, 정작 읽어야 할 "AI가 뭐라고 했나"가 스크롤에 묻힌다 --
        실기 중에는 그게 유일하게 봐야 하는 것이다.

        `print` 를 쓰는 이유: 이 클래스는 `rclpy` 노드가 아니라 Tk 앱이고(로거가 없다),
        GUI 는 `ros2 run` 으로 띄우므로 stdout 이 그대로 그 터미널이다. `flush=True` 는
        파이프로 넘길 때(로그 파일 tee) 버퍼에 갇히지 않게 한다.
        """
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {text}", flush=True)

    def append_chat(self, role: str, text: str, tag: str | None = None) -> None:
        role = role if role in {"user", "assistant", "system"} else "system"
        name = {"user": "사용자", "assistant": "AI", "system": "SYSTEM"}[role]
        stamp = datetime.now().strftime("%H:%M:%S")

        self.chat.configure(state="normal")
        self.chat.insert("end", f"{name}  {stamp}\n", f"{role}_name")
        self.chat.insert("end", f" {text.strip()} \n\n", tag or f"{role}_body")
        self.chat.see("end")
        self.chat.configure(state="disabled")

    def submit_text(self) -> None:
        text = self.entry.get().strip()
        if not text:
            return
        self.entry.delete(0, "end")
        self.handle_user_text(text)

    def handle_user_text(self, text: str) -> None:
        """The single funnel for everything the user says, typed or spoken."""
        self.append_chat("user", text)
        if STOP_PATTERN.search(text):
            # Pause, not e-stop (2026-08-12, contract §15). The word the user
            # says is reversible; the red button and ESC are not.
            self.pause_robot(text)
            return
        self.clear_clarification()
        self.bridge.publish_utterance(text)

    def pause_robot(self, reason: str) -> None:
        """"멈춰" -- reversible. Straight to the robot: no STT wait, no LLM
        round-trip, no queue (invariant I4).

        The arm stops where it is and keeps holding whatever it holds. Nothing
        happens afterwards on its own -- no auto-resume, no auto-place, no
        timeout. Saying "계속해" (or any other instruction) is what ends it.
        """
        self.bridge.publish_pause(reason)
        self.log_system(
            f"멈췄습니다. ({reason}) — '계속해'라고 하면 이어서 합니다.",
        )
        self.clear_clarification()

    def emergency_stop(self, reason: str) -> None:
        """비상정지 -- destructive. Straight to the robot, same as pause_robot.

        Drops the FSM into SAFE_STOP, which needs /pick/reset and a HOME round
        trip to come back from. Kept for the red button and ESC only; the stop
        *keyword* goes to pause_robot now (contract §15).
        """
        self.bridge.publish_estop(reason)
        self.log_system(
            f"🔴 비상정지를 전달했습니다. ({reason}) — 복구하려면 '리셋'이 필요합니다.",
        )
        self.clear_clarification()

    def open_graspgenx_viewer(self) -> None:
        """cobot2_ws의 viser 뷰어를 새 브라우저 창으로 연다.

        이 GUI 안에 임베드하지 않는 이유: viser는 WebSocket으로 브라우저에 접속시켜
        클라이언트 쪽(three.js/WebGL)에서 그리는 구조라, sensor_msgs/Image 토픽처럼
        받아서 tk.PhotoImage로 그릴 수 있는 정적 프레임이 아니다. cobot2_ws의
        grasp_bridge_node/graspgen_worker가 이미 떠 있고 live_viz(기본 켜짐)일 때만
        실제로 뭔가 보인다 -- 안 떠 있으면 브라우저가 연결 거부만 보여준다.
        """
        try:
            webbrowser.open(GRASPGENX_VIZ_URL)
            self.log_system(
                f"GraspGenX 뷰어를 새 창으로 열었습니다 ({GRASPGENX_VIZ_URL}). "
                "cobot2_ws의 grasp_bridge_node가 안 떠 있으면 빈 화면/연결 실패만 보입니다.",
            )
        except Exception as exc:
            self.log_system(f"GraspGenX 뷰어를 열지 못했습니다: {exc}")

    # ------------------------------------------------------- clarification

    def clear_clarification(self) -> None:
        for child in self.clarify_frame.winfo_children():
            child.destroy()
        self.clarify_photos.clear()
        self.clarify_frame.grid_remove()

    def show_clarification(self, object_ids: list[str]) -> None:
        """Crop each candidate out of the live frame and number it.

        Numbering is what makes the answer sayable: "왼쪽에서 두 번째 사과"
        is hard to say and harder to parse, "2번" is neither.
        """
        self.clear_clarification()
        if not object_ids:
            return

        snapshot = self.bridge.snapshot()
        frame = snapshot["keep_frame"]
        scene = snapshot["scene"]
        if frame is None or scene is None:
            return

        boxes = {obj["id"]: obj["bbox"] for obj in scene["objects"]}
        shown = 0
        for index, object_id in enumerate(object_ids, start=1):
            box = boxes.get(object_id)
            if box is None:
                continue
            try:
                photo = rgb_to_tk_photo(crop_object(frame, box), 130, 130)
            except Exception:
                continue
            self.clarify_photos.append(photo)

            cell = ttk.Frame(self.clarify_frame, style="Panel.TFrame")
            cell.grid(row=0, column=shown, padx=6, pady=4)
            tk.Label(cell, image=photo, bg=PANEL_2, bd=0).pack()
            tk.Button(
                cell,
                text=f"{index}번",
                bg="#343a40",
                fg=FG,
                activebackground="#4a5057",
                relief="flat",
                padx=8,
                command=lambda answer=f"{index}번": self.handle_user_text(answer),
            ).pack(fill="x", pady=(4, 0))
            shown += 1

        if shown:
            self.clarify_frame.grid()

    # --------------------------------------------------------------- voice

    def _voice_press(self, _event=None):
        if self.busy or self.recording:
            return "break"
        try:
            import sounddevice as sd

            self.audio_frames = []

            def callback(indata, _frames, _time_info, status):
                del status  # non-fatal; the audio callback must stay light
                self.audio_frames.append(indata.copy())

            self.audio_stream = sd.InputStream(
                samplerate=SAMPLE_RATE, channels=1, dtype="int16", callback=callback
            )
            self.audio_stream.start()
            self.record_started = time.monotonic()
            self.recording = True
            self.voice_button.configure(text="● 녹음 중", bg="#9e2a2b")
            self.bottom_status_var.set("음성 녹음 중 — 버튼을 떼면 인식")
        except Exception as exc:
            self.log_system(f"마이크 시작 실패: {exc}")
            self._cleanup_audio_stream()
        return "break"

    def _voice_release(self, _event=None):
        if not self.recording:
            return "break"

        elapsed = time.monotonic() - self.record_started
        self.recording = False
        self._cleanup_audio_stream()
        self.voice_button.configure(text="🎙 누르고 말하기", bg="#343a40")

        if elapsed < MIN_RECORD_SECONDS or not self.audio_frames:
            self.log_system("음성이 너무 짧아 입력을 취소했습니다.")
            self.bottom_status_var.set("음성 입력 대기")
            return "break"

        audio = np.concatenate(self.audio_frames, axis=0)
        AUDIO_PATH.parent.mkdir(parents=True, exist_ok=True)
        try:
            from scipy.io.wavfile import write

            write(str(AUDIO_PATH), SAMPLE_RATE, audio)
        except Exception as exc:
            self.log_system(f"음성 파일 저장 실패: {exc}")
            return "break"

        self._set_busy(True)
        self.bottom_status_var.set("음성을 텍스트로 변환하는 중...")

        def worker() -> None:
            try:
                self.events.put(("voice_text", self._get_transcriber().transcribe(AUDIO_PATH)))
            except Exception as exc:
                self.events.put(("error", f"음성 인식 실패: {exc}"))
            finally:
                self.events.put(("busy", False))

        threading.Thread(target=worker, name="vla-stt", daemon=True).start()
        return "break"

    def _get_transcriber(self):
        if self.transcriber is None:
            from vla_system.agent.llm import AgentLLM

            self.transcriber = AgentLLM(stt_model=STT_MODEL)
        return self.transcriber

    def _cleanup_audio_stream(self) -> None:
        stream = self.audio_stream
        self.audio_stream = None
        if stream is None:
            return
        for method in ("stop", "close"):
            try:
                getattr(stream, method)()
            except Exception:
                pass

    # ------------------------------------------------------------ pipeline

    def toggle_pipeline(self) -> None:
        if self.pipeline_process is None:
            self.start_pipeline()
        else:
            self.stop_pipeline()

    def _wait_repainting(self, seconds: float) -> None:
        """Sleep without letting the window go grey.

        `update_idletasks` redraws but does not deliver input events, so a click
        during cleanup cannot re-enter `start_pipeline`.
        """
        self.root.update_idletasks()
        time.sleep(seconds)

    def _clear_leftover_pipeline(self, *, include_realsense: bool) -> bool:
        """Kill every leftover pipeline process before starting a new one.

        Two `vla_pick_bridge_node`s racing cobot2_ws's pick_fsm is what this
        prevents: each takes orders from a different agent and they fight
        over /vla/pick_command. Leftovers
        are normal rather than exceptional -- closing or killing the GUI does not
        stop the `ros2 launch` it started, and a node that crashed out of a
        launch can outlive its siblings -- so this runs at every start instead of
        asking the user to go clean up in a terminal.

        ``include_realsense`` must be False whenever this GUI's own launch will
        pass ``enable_realsense:=false`` (cobot2_ws-integration mode, see
        `start_pipeline`) -- otherwise this kills a RealSense process this run
        never intended to touch, e.g. a camera the user started by hand
        (`reals1280` alias) or cobot2_ws's own launch (2026-08-11, real-hardware
        session: the GUI's leftover-cleanup was tearing down a manually-started
        camera and the manually-started pipeline it was supposed to leave
        alone).

        Returns False only when something refused to die, which is worth
        blocking on.
        """

        pattern = f"{PIPELINE_PATTERN}|{REALSENSE_PATTERN}" if include_realsense else PIPELINE_PATTERN
        leftover = find_existing_pipeline_pids(pattern)
        if not leftover:
            return True

        listing = "\n".join(f"  [{pid}] {command}" for pid, command in leftover[:8])
        if len(leftover) > 8:
            listing += f"\n  ... 외 {len(leftover) - 8}개"
        self.log_system(
            f"이전 vla_system 프로세스 {len(leftover)}개를 정리합니다:\n{listing}",
        )

        self.pipeline_button.configure(state="disabled")
        try:
            survivors = escalate_termination(
                [pid for pid, _ in leftover], wait=self._wait_repainting
            )
        finally:
            self.pipeline_button.configure(state="normal")

        if survivors:
            names = {pid: command for pid, command in leftover}
            detail = "\n".join(f"  [{pid}] {names.get(pid, '')}" for pid in survivors)
            self.log_system(f"정리하지 못한 프로세스가 있어 시작을 중단했습니다:\n{detail}")
            messagebox.showerror(
                "프로세스 정리 실패",
                f"다음 프로세스가 SIGKILL에도 종료되지 않았습니다:\n\n{detail}\n\n"
                "이 상태로 시작하면 vla_pick_bridge_node가 둘이 되어 cobot2_ws에 서로 "
                "다른 동작을 동시에 보낼 수 있습니다.\n"
                "터미널에서 직접 확인한 뒤 다시 시도하세요.",
                parent=self.root,
                icon="error",
            )
            return False

        self.log_system(f"이전 프로세스 {len(leftover)}개를 정리했습니다.")
        return True

    def start_pipeline(self) -> None:
        # pick_bridge 켜짐(기본값) = 이 launch가 enable_realsense:=false로 뜬다 =
        # 카메라는 남이 잡고 있다는 전제(cobot2_ws 쪽 launch나 사용자가 직접 켠
        # reals1280 같은 별도 alias) -- 그 카메라 프로세스는 우리 소유가 아니니
        # 정리 대상에서 뺀다. 이 판단이 leftover 정리보다 먼저 있어야 한다.
        pick_bridge = bool(self.pick_bridge_var.get())
        if not self._clear_leftover_pipeline(include_realsense=not pick_bridge):
            return

        # robot_node/wrist_grasp_node는 삭제됐다 -- 로봇 모션·손목 파지 모두
        # cobot2_ws pick_fsm이 전담이라 이 launch에 enable_robot/enable_wrist_grasp
        # 인자 자체가 더 이상 없다(CLAUDE.md #3).
        command = build_launch_command(
            pick_bridge=pick_bridge,
            skill_tier=bool(self.skill_tier_var.get()),
            perception=bool(self.perception_var.get()),
        )
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
        except FileNotFoundError:
            self.log_system(
                "ros2 실행 파일을 찾지 못했습니다. ROS 2 환경을 source한 터미널에서 "
                "GUI를 실행하세요.",
            )
            return
        except Exception as exc:
            self.log_system(f"VLA 파이프라인 시작 실패: {exc}")
            return

        self.pipeline_process = process
        self.pipeline_button.configure(text="VLA 정지")
        self.pick_bridge_check.configure(state="disabled")
        mode_note = (
            " cobot2_ws 연동(pick_bridge) ON -- 이 창의 '전송'/음성 발화가 곧 FSM"
            " 시작 트리거는 아님, cobot2_ws 쪽 auto_start:=true 또는 /pick/start가"
            " 별도로 필요함(README §3)."
            if pick_bridge
            else " 단독 모드(이 ws 카메라 직접 사용, cobot2_ws 미연동)."
        )
        self.log_system(
            f"VLA 파이프라인을 시작했습니다.{mode_note}"
            f"\n전체 로그: {PIPELINE_LOG_PATH}",
        )
        threading.Thread(
            target=self._read_pipeline_output,
            args=(process,),
            name="vla-launch-log",
            daemon=True,
        ).start()

    def _read_pipeline_output(self, process: subprocess.Popen) -> None:
        log_file = None
        try:
            PIPELINE_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            log_file = open(PIPELINE_LOG_PATH, "w", encoding="utf-8", buffering=1)
            log_file.write(
                f"# vla_system pipeline log, started {datetime.now().isoformat()}\n"
            )
        except OSError as exc:
            self.events.put(("pipeline_log", f"파이프라인 로그 파일 열기 실패: {exc}"))

        if process.stdout is not None:
            # Per-prefix ("[nodename-N]") set of streams currently mid-traceback,
            # so a crashing node's full stack reaches the chat instead of only
            # launch's one-line "process has died" summary.
            in_traceback: set[str] = set()
            for raw in process.stdout:
                line = raw.rstrip()
                if log_file is not None:
                    log_file.write(raw)
                if not line:
                    continue
                match = _LOG_PREFIX_RE.match(line)
                prefix = match.group(1) if match else None
                remainder = line[match.end() :] if match else line
                lowered = line.lower()

                if prefix is not None and prefix in in_traceback:
                    self.events.put(("pipeline_log", line))
                    if not remainder.startswith((" ", "\t")):
                        # Unindented line after "Traceback ...": the terminal
                        # "ExceptionType: message" line.
                        in_traceback.discard(prefix)
                    continue

                if any(token in lowered for token in _INTERESTING_LOG_TOKENS):
                    self.events.put(("pipeline_log", line))
                    if prefix is not None and "traceback (most recent call last)" in lowered:
                        in_traceback.add(prefix)
        if log_file is not None:
            log_file.close()
        self.events.put(("pipeline_exit", process.poll()))

    def stop_pipeline(self) -> None:
        process = self.pipeline_process
        self.pipeline_process = None
        if process is None:
            return

        # Tidy up *before* killing anything (contract §15). "닫으면 정리된다"는
        # 사람이 이 버튼에 기대하는 것이고, 그걸 종료 훅으로 흉내내면 안 된다 --
        # SIGINT 뒤에는 executor 가 이미 빠져나와 팔이 안 움직이고, 순서를 글자대로
        # 지키면(열고 나서 복귀) 물체를 지금 자리에 떨어뜨린다. 그래서 **버튼이**
        # 명시적으로 stow 를 부르고, 끝나기를 기다린 뒤에 프로세스를 내린다.
        self._stow_and_wait()

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
        except ProcessLookupError:
            pass

        self.pipeline_button.configure(text="VLA 시작")
        self.pick_bridge_check.configure(state="normal")
        self.log_system("GUI가 시작한 VLA 파이프라인을 정지했습니다.")

    def _stow_and_wait(self, timeout_s: float = 25.0) -> None:
        """Ask cobot2_ws to put down whatever it holds, then wait for IDLE.

        Blocking on purpose: this runs on the Tk thread from a button press,
        and the whole point is that the processes do not go down until the arm
        has finished. The cost is a frozen window for a few seconds, which is
        the correct thing to show -- something *is* happening.

        A timeout is not a failure to hide. If the arm did not reach IDLE we
        still tear down (the user asked to stop), but we say so, because the
        object may still be in the gripper.
        """
        state = self.bridge.snapshot().get("state") or {}
        if state.get("status") in ("idle", None, ""):
            return
        self.bridge.publish_stow("종료 정리")
        self.log_system("정리 중입니다 — 들고 있는 것을 놓고 홈으로 보냅니다.")
        self.root.update_idletasks()

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = (self.bridge.snapshot().get("state") or {}).get("status")
            if status == "idle":
                self.log_system("정리 완료.")
                return
            time.sleep(0.2)
            self.root.update_idletasks()
        self.log_system(
            "⚠ 정리가 시간 안에 끝나지 않았습니다. 그대로 종료합니다 — "
            "그리퍼에 물체가 남아 있을 수 있으니 확인하세요.",
        )

    # ------------------------------------------------------------- refresh

    def _drain_events(self) -> None:
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break

            if kind == "reply":
                self._apply_reply(payload)
            elif kind == "robot_state":
                self._apply_robot_state(payload)
            elif kind == "voice_text":
                self.handle_user_text(str(payload))
            elif kind == "error":
                self.log_system(str(payload))
            elif kind == "busy":
                self._set_busy(bool(payload))
            elif kind == "pipeline_log":
                self.log_system(str(payload))
            elif kind == "pipeline_exit":
                self.pipeline_process = None
                self.pipeline_button.configure(text="VLA 시작")
                self.pick_bridge_check.configure(state="normal")
                self.log_system(f"VLA launch 종료: returncode={payload}")

        self.root.after(30, self._drain_events)

    def _apply_reply(self, payload: dict) -> None:
        kind = payload["kind"]
        if kind == "ask_clarification":
            self.append_chat("assistant", payload["text"], tag="ask_body")
            self.show_clarification(payload["focus"])
        elif kind == "error":
            # 대화창에 남긴다. 이건 운영 로그가 아니라 **AI 가 사람에게 하는 말**이다
            # ("동작이 3번 연속 실패해서 멈췄습니다. 어떻게 할지 말씀해 주세요") --
            # 터미널로 빼면 사용자가 답해야 할 질문이 화면에서 사라진다.
            self.append_chat("assistant", payload["text"])
        else:
            self.append_chat("assistant", payload["text"])

    def _apply_robot_state(self, payload: dict) -> None:
        self.robot_status_var.set(robot_state_line(payload))
        key = (
            payload["last_action_id"],
            payload["last_action"],
            payload["last_result"],
            payload["details"],
        )
        if not payload["last_result"] or key == self.last_state_key:
            return
        self.last_state_key = key
        wording = {
            "succeeded": "완료했습니다",
            "failed": "실패했습니다",
            "cancelled": "중단했습니다",
            "rejected": "받아들이지 않았습니다",
        }.get(payload["last_result"], payload["last_result"])
        detail = f" — {payload['details']}" if payload["details"] else ""
        self.log_system(f"로봇: {payload['last_action']} {wording}{detail}")

    def _refresh_video(self) -> None:
        frame = self.bridge.take_frame()
        if frame is not None:
            try:
                max_w = max(320, self.video_label.winfo_width() - 4)
                max_h = max(240, self.video_label.winfo_height() - 4)
                self.photo = rgb_to_tk_photo(frame, max_w, max_h)
                self.video_label.configure(image=self.photo, text="")
            except Exception as exc:
                self.video_label.configure(image="", text=f"영상 표시 오류: {exc}")
        self.root.after(33, self._refresh_video)

    def _refresh_status(self) -> None:
        snapshot = self.bridge.snapshot()
        scene = snapshot["scene"]

        now = time.monotonic()
        elapsed = now - self.last_fps_time
        if elapsed >= 1.0:
            count = int(snapshot["frame_count"])
            self.current_fps = (count - self.last_frame_count) / elapsed
            self.last_frame_count = count
            self.last_fps_time = now

        frame_age = (
            now - float(snapshot["last_frame_monotonic"])
            if snapshot["last_frame_monotonic"]
            else float("inf")
        )

        pick_status_age = (
            now - float(snapshot["pick_status_monotonic"])
            if snapshot["pick_status_monotonic"]
            else float("inf")
        )
        self.pick_status_var.set(pick_status_line(snapshot["pick_status"], pick_status_age))

        self._render_scene_table(scene)
        if scene is None:
            self.vision_status_var.set("장면 데이터 대기 중")
        else:
            with_coord = sum(1 for obj in scene["objects"] if obj["has_position"])
            warning = (
                "" if scene["calibration_ok"]
                else " | ⚠ 테이블 보정 없음 (좌표 표시 안 됨, 집기는 가능)"
            )
            self.vision_status_var.set(
                f"물체 {len(scene['objects'])}개 (좌표 있음 {with_coord}개) | "
                f"GUI {self.current_fps:.1f} FPS{warning}"
            )

        pipeline_state = (
            "GUI launch ON" if self.pipeline_process is not None else "GUI launch OFF"
        )
        camera_state = "camera OK" if frame_age < 1.0 else "camera wait"
        self.bottom_status_var.set(f"{pipeline_state} | {camera_state}")
        self.root.after(200, self._refresh_status)

    def _render_scene_table(self, scene: dict | None) -> None:
        for item in self.scene_tree.get_children():
            self.scene_tree.delete(item)
        if scene is None:
            return
        for obj in sorted(scene["objects"], key=lambda o: o["id"]):
            x, y, z = obj["position"]
            self.scene_tree.insert(
                "",
                "end",
                values=(
                    obj["id"],
                    obj["color"],
                    f"{obj['confidence']:.2f}",
                    "O" if obj["has_position"] else "X",
                    f"({x:.3f}, {y:.3f}, {z:.3f})" if obj["has_position"] else "-",
                ),
            )

    def _set_busy(self, busy: bool) -> None:
        self.busy = busy
        state = "disabled" if busy else "normal"
        self.entry.configure(state=state)
        self.send_button.configure(state=state)
        if not busy:
            self.bottom_status_var.set("명령 입력 대기")

    # --------------------------------------------------------------- close

    def on_close(self) -> None:
        if self.recording:
            self.recording = False
            self._cleanup_audio_stream()
        if self.pipeline_process is not None:
            self.stop_pipeline()
        self.root.destroy()


def main() -> None:
    rclpy.init(args=None)
    events: queue.Queue = queue.Queue()
    bridge = GuiRosBridge(events)
    threading.Thread(
        target=rclpy.spin, args=(bridge,), name="vla-ros-spin", daemon=True
    ).start()

    root = tk.Tk()
    app = VLAApp(root, bridge, events)

    # rclpy.init()이 이미 자체 SIGINT 핸들러를 심어놨다 -- Ctrl+C가 오면 rclpy
    # 컨텍스트만 셧다운되고(spin 스레드가 ExternalShutdownException으로 죽음) Tk
    # mainloop()는 그 사실을 모른다. 그 다음 Python 기본 KeyboardInterrupt가 한 번
    # 더 올라오지만, Tkinter의 콜백 래퍼가 이걸 통째로 삼켜서(예외를 로그만 찍고
    # 계속 진행) mainloop() 밖으로 안 나간다 -- 그래서 Ctrl+C를 여러 번 눌러도 창이
    # 안 닫혔다(2026-08-11 확인). 여기서 SIGINT를 직접 잡아 우리 종료 경로
    # (on_close: 파이프라인 서브프로세스 정리 + destroy)로 보낸다. after(0, ...)로
    # 미루는 이유: 시그널 핸들러 자체는 다음 바이트코드 경계에서만 실행되므로,
    # Tk 위젯 조작은 이미 안전한 시점인 메인 루프 콜백 안에서 하도록 넘긴다.
    def _handle_sigint(_signum, _frame) -> None:
        root.after(0, app.on_close)

    signal.signal(signal.SIGINT, _handle_sigint)

    try:
        root.mainloop()
    finally:
        app._cleanup_audio_stream()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
