#!/usr/bin/env python3
"""Hands agent decisions to cobot2_ws's pick_fsm and reflects its results back.

This node is the only executor left in this ws (``robot_node.py`` and its own
Doosan/gripper control were removed -- cobot2_ws's ``pick_fsm`` is the sole
robot/gripper owner now, CLAUDE.md #3). It subscribes ``/vla/robot/action``/
``/vla/robot/stop``/``/vla/estop`` and publishes ``/vla/robot/state``, and
forwards to a *different process in a different git clone* (``~/cobot2_ws``'s
``vla_command_node``) over ``/vla/pick_command`` / ``/vla/pick_result``
(``std_msgs/String``, JSON). See ``docs/state.md`` "cobot2_ws 통합" for the
full checklist this implements, and ``~/cobot2_ws/md/vla-bridge-contract.md``
for the schema this node is bound by (not copied here on purpose --
CLAUDE.md #2).

What this node deliberately does not do:

- **Base-frame coordinates.** ``object_id`` never crosses, and neither does
  ``vla_perception``'s table homography output -- cobot2_ws's own D435i +
  ``T_cam2base`` produces the actual grasp pose. A pixel *does* cross now
  (``pixel``/``pixel_wh``, contract #2/#8) to disambiguate which instance of
  a class, since the camera the pixel was measured on and cobot2_ws's own
  camera are confirmed the same physical D435i (2026-08-10/11) -- see
  ``bbox_center()`` in ``bridge/pick_bridge.py``. cobot2_ws still ignores it
  today (no ``select_by_point()`` yet, contract #8).
- **Approval.** ``/pick/approve`` is never called, directly or indirectly --
  cobot2_ws's ``vla_command_node`` hard-rejects ``cmd:"approve"`` in code, and
  this bridge does not try to route around that (contract #4).

What used to be impossible and now is not:

- **``pick_and_hold``/``release_held``.** cobot2_ws's FSM used to carry every
  pick through to place, so "hold it" and "put it down right here" had nowhere
  to go and both tools were removed. Contract #13 added ``WAIT_PLACE_TARGET``
  (2026-08-11) and #14 added ``release_now`` (2026-08-12), so both are back:
  ``pick_and_hold`` is a pick with the destination left empty, and
  ``release_held`` opens the gripper where the arm already stands.
  Both are only meaningful while parked in that state, and both are gated on it
  here *and* on cobot2_ws's side -- a mistake should cost a rejected tool call,
  not an object dropped from wherever the arm happens to be.
"""

import json
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from vla_interfaces.msg import RobotAction, RobotState, SceneSnapshot

from vla_system.bridge.pick_bridge import (
    bbox_center,
    build_abort_command,
    build_pause_command,
    build_pick_command,
    build_release_now_command,
    build_reset_command,
    build_resume_command,
    build_set_place_command,
    build_stow_command,
    fsm_state_view,
    parse_pick_result,
    place_rejection_reason,
    resolve_scene_object,
    result_update,
)


class PickBridgeNode(Node):
    def __init__(self):
        super().__init__("vla_pick_bridge")

        self.declare_parameter("action_topic", "/vla/robot/action")
        self.declare_parameter("stop_topic", "/vla/robot/stop")
        self.declare_parameter("estop_topic", "/vla/estop")
        # The GUI's stop keyword (contract §15). Reversible: cobot2_ws parks in
        # PAUSED holding whatever it holds, and nothing happens on a timer.
        self.declare_parameter("pause_topic", "/vla/robot/pause")
        # Tidy-up-and-finish (contract §15). Unlike the three stop topics
        # this one *moves* the arm, so it is not part of that loop.
        self.declare_parameter("stow_topic", "/vla/robot/stow")
        # agent_node's reset_after_stop tool. Forwarded unconditionally, same
        # "cobot2_ws validates, this side does not re-check" pattern as
        # stop_topic/estop_topic -- cobot2_ws rejects it outright unless its
        # FSM is actually in SAFE_STOP (vla-bridge-contract.md #2).
        self.declare_parameter("reset_topic", "/vla/robot/reset")
        self.declare_parameter("state_topic", "/vla/robot/state")
        self.declare_parameter("scene_topic", "/vla/scene")
        # cobot2_ws's vla_command_node, a different process (different clone
        # entirely) -- these two must match its command_topic/result_topic
        # defaults exactly (voice_processing/vla_command_node.py).
        self.declare_parameter("pick_command_topic", "/vla/pick_command")
        self.declare_parameter("pick_result_topic", "/vla/pick_result")
        # cobot2_ws's pick_fsm publishes its live State enum name here on every
        # transition (task_manager.py, std_msgs/String). Subscribed so the GUI
        # can show which step the arm is in -- see
        # docs/context/fsm-state-integration.md.
        self.declare_parameter("fsm_state_topic", "/pick/state")
        self.declare_parameter("max_scene_age_s", 2.0)
        # cobot2_ws's own wait_timeout_sec default is 50s (vla_command_node);
        # stay above it so a slow-but-real answer is never mistaken for a
        # dead one. Bumped 60->120 (2026-08-11): real pick_and_place runs were
        # cutting off after arrival with no /vla/pick_result -- if the motion
        # itself is taking >50s, cobot2_ws's own wait_timeout_sec likely also
        # needs raising there (that side isn't checked from here).
        self.declare_parameter("result_timeout_s", 120.0)
        # RobotState forces an opinion on "is motion enabled" even though
        # that switch lives in cobot2_ws's pick_fsm.yaml now, not here. True
        # because the point of this bridge is that motion really does
        # happen -- just on the other clone's arm.
        self.declare_parameter("motion_enabled", True)
        # vla-bridge-contract.md #5: table/discard are placeholder joint poses
        # on real hardware as of 2026-08-10, not taught to a safe place yet.
        # Off by default -- flip this once cobot2_ws confirms they're teach-
        # complete, rather than changing code to re-enable them.
        self.declare_parameter("allow_unverified_place", False)

        self.max_scene_age_s = float(self.get_parameter("max_scene_age_s").value)
        self.result_timeout_s = float(self.get_parameter("result_timeout_s").value)
        self.motion_enabled = bool(self.get_parameter("motion_enabled").value)
        self.allow_unverified_place = bool(
            self.get_parameter("allow_unverified_place").value
        )

        self.scene: SceneSnapshot | None = None
        self.scene_monotonic = 0.0
        self.scene_lock = threading.Lock()

        # Only one request in flight, mirroring vla_robot's own rule: a
        # second action arriving while one is already out is rejected, not
        # queued -- a queue is how a stale target reaches the arm three
        # decisions after the user changed their mind.
        self.pending_request_id = ""
        self.pending_action: RobotAction | None = None
        self.pending_sent_monotonic = 0.0
        # Class of the pending pick, captured at send time so a HOLDING_STATES
        # /pick/state can fill RobotState.holding without cobot2_ws sending it
        # (contract §3 leaves holding out of /vla/pick_result).
        self.pending_class_name = ""

        self.status = "idle"
        self.last_action_id = ""
        self.last_action = ""
        self.last_result = ""
        self.details = ""
        # Populated from the pending pick while /pick/state says the gripper is
        # holding (FSM_HOLDING_STATES); cleared on any terminal/reject/timeout.
        self.holding_object_id = ""
        self.holding_class_name = ""

        # RobotAction.header.stamp is when the agent read the world, not when
        # it finished thinking -- an LLM round-trip is seconds long, so a
        # stop can land inside it. Same guard as vla_robot's action_callback.
        self.stop_time_ns = 0

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.state_publisher = self.create_publisher(
            RobotState, str(self.get_parameter("state_topic").value), latched_qos
        )
        # command_qos here matches cobot2_ws's own COMMAND_QOS for these two
        # topics exactly (vla_command_node.py) -- RELIABLE/VOLATILE on both
        # sides, or the two processes never see each other's messages at all
        # (no error, just silence -- the classic ROS 2 QoS-mismatch symptom).
        self.pick_command_publisher = self.create_publisher(
            String, str(self.get_parameter("pick_command_topic").value), command_qos
        )
        self.create_subscription(
            String,
            str(self.get_parameter("pick_result_topic").value),
            self.pick_result_callback,
            command_qos,
        )
        self.create_subscription(
            SceneSnapshot,
            str(self.get_parameter("scene_topic").value),
            self.scene_callback,
            stream_qos,
        )
        self.create_subscription(
            RobotAction,
            str(self.get_parameter("action_topic").value),
            self.action_callback,
            command_qos,
        )
        # Live FSM step. Same command_qos (RELIABLE/VOLATILE/depth 10) as
        # task_manager's default publisher -- a mismatch here means silence.
        self.create_subscription(
            String,
            str(self.get_parameter("fsm_state_topic").value),
            self.fsm_state_callback,
            command_qos,
        )
        # Two topics, one handler -- same split as vla_robot's for the same
        # reason: /vla/estop is the GUI's hardcoded keyword path, /vla/robot/
        # stop is the agent's cancel_current_action.
        for parameter in ("stop_topic", "estop_topic"):
            self.create_subscription(
                String,
                str(self.get_parameter(parameter).value),
                self.stop_callback,
                command_qos,
            )
        # Third stop, deliberately its own handler (contract §15). The GUI's
        # stop *word* comes here now; only the red button and ESC still take
        # the destructive path above.
        self.create_subscription(
            String,
            str(self.get_parameter("pause_topic").value),
            self.pause_callback,
            command_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("stow_topic").value),
            self.stow_callback,
            command_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("reset_topic").value),
            self.reset_callback,
            command_qos,
        )

        self.create_timer(1.0, self.check_result_timeout)

        self.publish_state()
        self.get_logger().info(
            "pick bridge ready: forwarding to cobot2_ws pick_fsm via "
            f"{self.get_parameter('pick_command_topic').value}"
        )

    # ------------------------------------------------------------ callbacks

    def scene_callback(self, message: SceneSnapshot) -> None:
        with self.scene_lock:
            self.scene = message
            self.scene_monotonic = time.monotonic()

    def action_callback(self, message: RobotAction) -> None:
        name = message.name.strip()

        if name == "set_place":
            self.handle_set_place(message)
            return

        if name == "release_held":
            self.handle_release_held(message)
            return

        if name == "resume":
            self.handle_resume(message)
            return

        # pick_and_hold is pick_and_place with the destination deliberately
        # left empty -- cobot2_ws then parks in WAIT_PLACE_TARGET instead of
        # finishing the cycle (contract #13). Same wire command, so it falls
        # through here; the place field is cleared below rather than trusted.
        if name not in ("pick_and_place", "pick_and_hold"):
            self.reject(message, f"알 수 없는 동작입니다: {name}")
            return
        hold_only = name == "pick_and_hold"

        decided_ns = Time.from_msg(message.header.stamp).nanoseconds
        if decided_ns and decided_ns <= self.stop_time_ns:
            self.reject(message, "정지 이전에 결정된 동작이라 실행하지 않습니다")
            return

        if self.pending_action is not None:
            self.reject(message, "이미 다른 동작을 수행하는 중입니다")
            return

        object_id = message.object_id.strip()
        with self.scene_lock:
            scene = self.scene
            age = time.monotonic() - self.scene_monotonic
        if scene is None:
            self.reject(message, "아직 카메라 장면을 받지 못했습니다")
            return
        if age > self.max_scene_age_s:
            self.reject(message, f"카메라 장면이 {age:.1f}초 전 것이라 사용할 수 없습니다")
            return
        # resolve_scene_object (not a bare id match): a borderline-confidence
        # detection flickers in/out of the tracker and its track_id churns
        # every time it's dropped for >tracker_max_missed_frames, so the id
        # the LLM read a couple of seconds ago can be gone by the time we get
        # here even though the same object never left the table -- see its
        # docstring. Falls back to "the one object of this class still on
        # screen" rather than rejecting a still-valid pick.
        scene_object = resolve_scene_object(scene.objects, object_id)
        if scene_object is None:
            self.reject(message, f"'{object_id}'는 지금 화면에 없습니다")
            return
        class_name = scene_object.class_name

        # Optional (contract #2/#8): omitted while vla_perception hasn't
        # reported a frame resolution yet (image_width/height still 0, e.g.
        # right after startup before the first camera frame). cobot2_ws
        # validates but ignores both today (no select_by_point() yet) -- sent
        # anyway so this side doesn't need a second change once that lands.
        # The pixel means something on cobot2_ws's side without reprojection
        # only because the camera is now confirmed the same physical D435i
        # (2026-08-10/11) -- see bbox_center()'s docstring.
        pixel = None
        pixel_wh = None
        if scene.image_width and scene.image_height:
            pixel = bbox_center(scene_object)
            pixel_wh = (scene.image_width, scene.image_height)

        # Empty now means "the model deliberately left it unset" (contract
        # #13, 2026-08-11) -- tools.py's pick_and_place makes `place` nullable
        # precisely so this reaches cobot2_ws as an omitted key and it parks
        # in WAIT_PLACE_TARGET instead of guessing basket on our behalf. Only
        # a non-empty value goes through the allow-list check below; an empty
        # one has nothing to validate.
        place = "" if hold_only else message.place.strip()
        if place:
            reason_for_place_rejection = place_rejection_reason(
                place, allow_unverified=self.allow_unverified_place
            )
            if reason_for_place_rejection is not None:
                self.reject(message, reason_for_place_rejection)
                return

        payload = build_pick_command(
            class_name=class_name,
            place=place,
            request_id=message.action_id,
            stamp_ns=decided_ns,
            pixel=pixel,
            pixel_wh=pixel_wh,
        )
        self.pending_request_id = message.action_id
        self.pending_action = message
        self.pending_class_name = class_name
        self.pending_sent_monotonic = time.monotonic()
        self.pick_command_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        self.get_logger().info(
            f"-> cobot2_ws pick_and_place class={class_name} place={place} "
            f"pixel={pixel} id={message.action_id}"
        )

        self.status = "moving"
        self.last_action_id = message.action_id
        self.last_action = message.name
        self.last_result = ""
        self.details = message.reason
        self.publish_state()

    def handle_set_place(self, message: RobotAction) -> None:
        """agent_node's ``set_place`` tool -> cobot2_ws's contract #13 path.

        Only valid while the currently-pending pick is parked in
        ``WAIT_PLACE_TARGET`` (mirrored here as ``self.status ==
        "waiting_place"`` by :meth:`fsm_state_callback`) -- unlike
        ``pick_and_place`` this never starts a new request, it fills in the
        destination for the one already in flight, so it reuses
        ``self.pending_request_id`` rather than minting a fresh id.
        """
        if self.pending_action is None or self.status != "waiting_place":
            self.reject(
                message,
                "지금은 놓을 위치를 지정할 수 있는 상태가 아닙니다",
                clear_holding=False,
            )
            return

        place = message.place.strip()
        reason_for_place_rejection = place_rejection_reason(
            place, allow_unverified=self.allow_unverified_place
        )
        if reason_for_place_rejection is not None:
            self.reject(message, reason_for_place_rejection, clear_holding=False)
            return

        payload = build_set_place_command(
            place=place,
            request_id=self.pending_request_id,
            stamp_ns=Time.from_msg(message.header.stamp).nanoseconds,
        )
        self.pick_command_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        self.get_logger().info(
            f"-> cobot2_ws set_place place={place} id={self.pending_request_id}"
        )
        # No local RobotState change here: still holding, still the same
        # pending_action -- the next real update is cobot2_ws's own
        # /pick/state moving to PLACE, which fsm_state_callback picks up.

    def handle_release_held(self, message: RobotAction) -> None:
        """agent_node's ``release_held`` tool -> cobot2_ws's ``/pick/release_now``.

        The same gate as :meth:`handle_set_place`, for the same reason: this
        answers the question ``WAIT_PLACE_TARGET`` asked, so it is only
        meaningful while the arm is actually parked there holding something.

        Outside that state it is not merely useless, it is dangerous -- opening
        the gripper mid-move drops the object from wherever the arm happens to
        be. cobot2_ws refuses it too (``_srv_release_now`` checks its own
        state); this is the near-side half of the same gate, so a mistake costs
        a rejected tool call rather than a round trip.
        """
        if self.pending_action is None or self.status != "waiting_place":
            self.reject(
                message,
                "지금은 그 자리에 놓을 수 있는 상태가 아닙니다 "
                "(물체를 들고 목적지를 기다리는 중일 때만 됩니다)",
                clear_holding=False,
            )
            return

        payload = build_release_now_command(
            request_id=self.pending_request_id,
            stamp_ns=Time.from_msg(message.header.stamp).nanoseconds,
        )
        self.pick_command_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        self.get_logger().info(
            f"-> cobot2_ws release_now id={self.pending_request_id}"
        )
        # Same as set_place: no local state change. cobot2_ws's /pick/state
        # moving to RELEASE is the next real update.

    def handle_resume(self, message: RobotAction) -> None:
        """agent_node's ``resume_mission`` tool -> cobot2_ws's ``/pick/resume``.

        Forwarded unconditionally -- same "cobot2_ws validates, this side does
        not re-check" pattern as abort/reset. It rejects the call outright
        unless its FSM is actually in PAUSED, and it alone knows where to
        resume *to* (holding + destination -> PLACE, holding without ->
        WAIT_PLACE_TARGET, empty gripper -> PERCEIVE).

        Deliberately not gated on ``pending_action``: the arm can be paused
        with nothing in flight from this side (a human hit the rqt button), and
        refusing to un-pause it then would leave the only recovery on cobot2_ws's
        local console.
        """
        payload = build_resume_command(
            request_id=uuid.uuid4().hex[:12],
            stamp_ns=Time.from_msg(message.header.stamp).nanoseconds,
        )
        self.pick_command_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        self.get_logger().info("-> cobot2_ws resume")

    def pause_callback(self, message: String) -> None:
        """"멈춰" -- reversible (contract §15).

        ``stop_time_ns`` is bumped here **as well as** in :meth:`stop_callback`.
        That field is what makes agent_node's "a stop landed during this
        decision" guard work: an action the model decided on *before* the pause
        must not go out afterwards. A pause that skipped it would let a stale
        pick slip through the moment the user says "멈춰" mid-turn -- which is
        the exact window the guard exists for.

        Fired unconditionally, same as the abort path. Deciding locally that
        "nothing is running, skip it" is the one mistake a stop is not allowed
        to make; cobot2_ws answers success even when there is nothing to stop.
        """
        reason = message.data.strip() or "정지"
        self.stop_time_ns = self.get_clock().now().nanoseconds
        self.get_logger().warning(f"PAUSE: {reason}")
        payload = build_pause_command(
            request_id=uuid.uuid4().hex[:12],
            reason=reason,
            stamp_ns=self.stop_time_ns,
        )
        self.pick_command_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        # No local RobotState change, same reasoning as handle_set_place: the
        # arm is still holding what it was holding, and cobot2_ws's own
        # /pick/state moving to PAUSED is the next real update.

    def stow_callback(self, message: String) -> None:
        """"정리하고 끝내" -> cobot2_ws's ``/pick/stow`` (contract §15).

        ``stop_time_ns`` is **not** bumped here. This is not a stop -- it is
        a motion, and it is the one the user just asked for. Bumping it
        would make the guard treat the stow itself as "decided before a
        stop" on the next turn.
        """
        reason = message.data.strip() or "종료 정리"
        self.get_logger().info(f"STOW: {reason}")
        payload = build_stow_command(
            request_id=uuid.uuid4().hex[:12],
            stamp_ns=self.get_clock().now().nanoseconds,
        )
        self.pick_command_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def stop_callback(self, message: String) -> None:
        reason = message.data.strip() or "정지"
        self.stop_time_ns = self.get_clock().now().nanoseconds
        self.get_logger().warning(f"STOP: {reason}")

        # Fired unconditionally, same reasoning as vla_robot's stop_callback:
        # guessing "nothing is in flight, skip it" is the one mistake a stop
        # path is not allowed to make.
        payload = build_abort_command(
            request_id=uuid.uuid4().hex[:12],
            reason=reason,
            stamp_ns=self.get_clock().now().nanoseconds,
        )
        self.pick_command_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

        # No RobotState to publish here: with nothing pending there is
        # nothing to cancel, and with something pending, only cobot2_ws's own
        # terminal /vla/pick_result (pick_result_callback) can say what
        # actually happened to it -- inventing "cancelled" ahead of that
        # would be a guess this path is not allowed to make either.

    def reset_callback(self, message: String) -> None:
        """agent_node's reset_after_stop tool -> cobot2_ws's ``/pick/reset``.

        Forwarded unconditionally (same reasoning as ``stop_callback``): this
        node does not track SAFE_STOP reliably enough on its own to gate the
        send locally (``fsm_state_callback`` only updates while a pick is
        still ``pending_action``, which a completed abort may have already
        cleared) -- cobot2_ws is the source of truth and rejects the command
        itself outside SAFE_STOP (contract #2). Success moves the arm to HOME
        without a human approval step.
        """
        reason = message.data.strip() or "reset"
        self.get_logger().warning(f"RESET requested: {reason}")
        payload = build_reset_command(
            request_id=uuid.uuid4().hex[:12],
            stamp_ns=self.get_clock().now().nanoseconds,
        )
        self.pick_command_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def fsm_state_callback(self, message: String) -> None:
        """cobot2_ws's live FSM step -> RobotState, for GUI visibility only.

        Published with current_action still set and last_result empty on
        purpose: agent_node's robot_state_callback drops any state where
        current_action is set (``if not last_result or current_action:
        return``), so these updates reach the GUI without waking the agent for
        a fresh decision. The decision point stays the terminal
        /vla/pick_result. Ignored entirely when no pick is in flight -- a late
        HOME after a concluded pick must not resurrect stale status.
        """
        if self.pending_action is None:
            return
        view = fsm_state_view(message.data.strip())
        self.status = view.status
        self.details = view.label
        if view.holding:
            self.holding_object_id = self.pending_action.object_id
            self.holding_class_name = self.pending_class_name
        else:
            self.holding_object_id = ""
            self.holding_class_name = ""
        self.publish_state()

    def pick_result_callback(self, message: String) -> None:
        doc = parse_pick_result(message.data)
        if doc is None:
            self.get_logger().warning(
                f"could not parse /vla/pick_result: {message.data!r}"
            )
            return
        request_id = str(doc.get("request_id", ""))
        if request_id != self.pending_request_id or not request_id:
            # A control-cmd (abort/start/reset) ack, or a late reply for a
            # request already given up on (see check_result_timeout). Not
            # this node's concern.
            self.get_logger().debug(f"ignoring unmatched pick_result id={request_id!r}")
            return

        action = self.pending_action
        update = result_update(str(doc.get("result", "")))

        if not update.terminal:
            self.status = update.status
            self.publish_state()
            return

        self.pending_request_id = ""
        self.pending_action = None
        self.pending_sent_monotonic = 0.0
        self.pending_class_name = ""
        self.holding_object_id = ""
        self.holding_class_name = ""
        self.status = update.status
        self.last_action_id = action.action_id if action else self.last_action_id
        self.last_action = action.name if action else self.last_action
        self.last_result = update.last_result
        self.details = str(doc.get("reason", ""))
        self.publish_state()

    def check_result_timeout(self) -> None:
        if self.pending_action is None:
            return
        age = time.monotonic() - self.pending_sent_monotonic
        if age <= self.result_timeout_s:
            return
        self.get_logger().error(
            f"cobot2_ws did not answer /vla/pick_command within "
            f"{self.result_timeout_s:.0f}s (id={self.pending_request_id}); giving up"
        )
        # Giving up here is purely local bookkeeping unless cobot2_ws is also
        # told to stop -- pick_fsm has no idea this side stopped waiting, so
        # without this it keeps running the stale pick/place cycle in the
        # background. The next action_callback then sees pending_action is
        # None and happily accepts + sends a *second* /vla/pick_command while
        # the FSM is still mid-cycle on the first one -- observed 2026-08-12:
        # FSM stuck in PLACE still carrying the first request's place value
        # while the GUI/new command already said "table". Same abort path
        # stop_callback uses, just triggered by timeout instead of an
        # explicit user stop.
        self.pick_command_publisher.publish(
            String(
                data=json.dumps(
                    build_abort_command(
                        request_id=uuid.uuid4().hex[:12],
                        reason="응답 타임아웃 -- VLA 측에서 포기",
                        stamp_ns=self.get_clock().now().nanoseconds,
                    ),
                    ensure_ascii=False,
                )
            )
        )
        action = self.pending_action
        self.pending_request_id = ""
        self.pending_action = None
        self.pending_sent_monotonic = 0.0
        self.pending_class_name = ""
        self.holding_object_id = ""
        self.holding_class_name = ""
        self.status = "idle"
        self.last_action_id = action.action_id
        self.last_action = action.name
        self.last_result = "failed"
        self.details = "cobot2_ws로부터 응답이 없습니다"
        self.publish_state()

    # -------------------------------------------------------------- outputs

    def reject(
        self, action: RobotAction, reason: str, *, clear_holding: bool = True
    ) -> None:
        self.get_logger().warning(f"action rejected ({action.name}): {reason}")
        # clear_holding=False for a rejected set_place: the gripper is still
        # physically holding the pending pick's object regardless of whether
        # this particular follow-up command was accepted (handle_set_place) --
        # wiping holding_* here would tell the GUI/LLM the arm let go when it
        # never did.
        if clear_holding:
            self.holding_object_id = ""
            self.holding_class_name = ""
        self.last_action_id = action.action_id
        self.last_action = action.name
        self.last_result = "rejected"
        self.details = reason
        self.publish_state()

    def publish_state(self) -> None:
        message = RobotState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base"
        message.status = self.status
        # cobot2_ws's /vla/pick_result carries no holding field
        # (vla-bridge-contract.md #3). These are filled instead from the
        # pending pick while /pick/state reports a HOLDING_STATES step, and
        # cleared on any terminal/reject/timeout -- see fsm_state_callback and
        # docs/context/fsm-state-integration.md.
        message.holding_object_id = self.holding_object_id
        message.holding_class_name = self.holding_class_name
        message.current_action_id = (
            self.pending_action.action_id if self.pending_action else ""
        )
        message.current_action = (
            self.pending_action.name if self.pending_action else ""
        )
        message.last_action_id = self.last_action_id
        message.last_action = self.last_action
        message.last_result = self.last_result
        message.details = self.details
        message.motion_enabled = self.motion_enabled
        self.state_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PickBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
