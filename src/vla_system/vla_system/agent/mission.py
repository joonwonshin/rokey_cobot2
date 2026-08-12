"""Mission state machine: several objects, one at a time, revisable mid-flight.

Why this exists as its own module
---------------------------------
Mission state used to live in ``SkillTier`` as three private attributes
(``_mission``, ``_acted``, ``_mission["picked"]``). That worked while the rule
layer was the only thing that ran missions, and it broke the moment the
conversational layer needed to *change* one: Tier 2 could not see what Tier 1
had already done, so the only correction it could express was "throw the whole
mission away and start over" -- which is what ``handle()`` actually did.

Both tiers now drive the same supervisor, so a correction is a patch on shared
state instead of a restart.

This module knows nothing about ROS, and nothing about the rule store either.
It talks to a ``MissionHost`` (implemented by ``agent_node``) exactly the way
``SkillTier`` talks to ``SkillHost``. That is what makes it testable without a
live graph -- see ``test_mission.py``.

The five rules the executor must not lose
-----------------------------------------
R1  Never dispatch while an action is in flight. ``accepted`` is not terminal;
    only succeeded/failed/cancelled/rejected are. One arm, one action (I6).
R2  Command Barrier. Do not dispatch straight out of the result callback --
    apply any waiting user command first. Without it, "나머지는 테이블로" said
    while the first object is coming down lands *after* the next object has
    already been dispatched to the basket.
R3  Never reuse coordinates. Object ids survive; positions do not. Re-read the
    scene at every dispatch.
R4  SNAPSHOT by default. "보이는 물건 모두" means the set that was visible when
    the order was given, not a queue that grows as things appear.
R5  A failed object does not kill the mission. Retry, then move on, then report
    honestly at the end.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Protocol

#: Results that mean the executor is done with the action, one way or another.
#: ``accepted`` is deliberately absent -- it means "handed off, still running",
#: and treating it as terminal is exactly how two objects get dispatched at once
#: (R1). ``pick_bridge.result_update()`` already draws this line; this set must
#: agree with it.
TERMINAL_RESULTS = frozenset({"succeeded", "failed", "cancelled", "rejected"})

#: Mission-level status.
RUNNING = "RUNNING"
PAUSED = "PAUSED"
COMPLETED = "COMPLETED"
CANCELLED = "CANCELLED"
FAILED = "FAILED"

#: Which objects the mission covers.
SNAPSHOT = "SNAPSHOT"   # fixed at the moment the order was given (default, R4)
DYNAMIC = "DYNAMIC"     # keep taking whatever shows up ("앞으로 보이는 건 계속")

#: How far a correction reaches.
CURRENT_AND_REMAINING = "CURRENT_AND_REMAINING"
REMAINING_ONLY = "REMAINING_ONLY"


@dataclass(frozen=True)
class SceneItem:
    """One pickable thing, as the mission layer needs to see it.

    ``rank`` breaks ties when several candidates match; the host decides what it
    means (track id, distance, whatever it can order by).

    Defined here rather than in ``skill_tier`` because both tiers now need it
    and the mission layer is the lower one. ``skill_tier`` re-exports it so the
    existing ``from ...skill_tier import SceneItem`` keeps working.
    """

    object_id: str
    class_name: str
    color: str
    pickable: bool = True
    rank: int = 0


@dataclass
class MissionState:
    """Everything both tiers need to agree on. Plain data, safe to publish."""

    mission_id: str
    revision: int = 0
    original_instruction: str = ""
    status: str = RUNNING
    scope: str = SNAPSHOT
    pending_ids: list[str] = field(default_factory=list)
    current_object_id: str = ""
    completed_ids: list[str] = field(default_factory=list)
    failed_ids: list[str] = field(default_factory=list)
    skipped_ids: list[str] = field(default_factory=list)
    destination: str = "basket"      # basket | table | discard | "" (=hold)
    current_action_id: str = ""
    continue_on_failure: bool = True
    retry_limit: int = 2

    @property
    def in_flight(self) -> bool:
        """R1 gate. True between dispatch and the terminal result."""
        return bool(self.current_action_id)

    @property
    def done(self) -> bool:
        return self.status in (COMPLETED, CANCELLED, FAILED)

    def summary(self) -> str:
        """The line the user hears when the mission ends.

        Reporting only the successes is how "5개 중 1개만 계속 실패" turns into a
        silent partial job (R5).
        """
        parts = [f"{len(self.completed_ids)}개 완료"]
        if self.failed_ids:
            parts.append(f"{len(self.failed_ids)}개 실패")
        if self.skipped_ids:
            parts.append(f"{len(self.skipped_ids)}개 건너뜀")
        return ", ".join(parts)


class MissionHost(Protocol):
    """What the supervisor needs from whoever runs it."""

    def dispatch_pick(self, object_id: str, place: str, reason: str) -> str:
        """Send one object to the executor. Returns the action id."""

    def say(self, text: str) -> None: ...

    def scene_items(self) -> list[SceneItem]: ...

    def pending_user_commands(self) -> bool:
        """True if an utterance is waiting to be applied (R2, Command Barrier)."""

    def on_mission_state(self, state: MissionState) -> None:
        """State changed -- publish it for the GUI."""


class MissionSupervisor:
    """Runs one mission at a time. Not thread-safe; the caller serialises."""

    def __init__(self, host: MissionHost, blocked=None) -> None:
        self.host = host
        #: ``(class_name, color) -> bool``. The rule layer's prohibitions and
        #: hazard gate, injected rather than imported: the supervisor must not
        #: grow a dependency on RuleStore, or it stops being testable on its own.
        self._blocked = blocked or (lambda class_name, color: False)
        self.state: MissionState | None = None
        self._retries: dict[str, int] = {}
        self._reacquired: set[str] = set()

    # ------------------------------------------------------------- lifecycle

    def start(self, *, mission_id: str, object_ids: list[str], instruction: str = "",
              destination: str = "basket", scope: str = SNAPSHOT,
              continue_on_failure: bool = True, retry_limit: int = 2) -> MissionState:
        """Begin a mission over an already-filtered list of object ids.

        Filtering (class/color/exclude/prohibit/hazard) happens upstream in the
        rule layer -- it needs the RuleStore and this layer deliberately does
        not have it. What arrives here is "these, in this order".
        """
        self.state = MissionState(
            mission_id=mission_id,
            original_instruction=instruction,
            pending_ids=list(object_ids),
            destination=destination,
            scope=scope,
            continue_on_failure=continue_on_failure,
            retry_limit=retry_limit,
        )
        self._retries = {}
        self._reacquired = set()
        self._publish()
        self._advance()
        return self.state

    def cancel(self, reason: str = "") -> None:
        """Stop for good. ``completed_ids`` survives -- the user asked to stop,
        not to forget what was already done."""
        if self.state is None or self.state.done:
            return
        self.state.status = CANCELLED
        self.state.current_object_id = ""
        self.state.current_action_id = ""
        self._publish()
        self.host.say(f"작업을 취소했습니다. {self.state.summary()}."
                      + (f" ({reason})" if reason else ""))

    def pause(self, reason: str = "") -> None:
        """Stop dispatching. Nothing else changes.

        Note what this does *not* do: it does not cancel the action already in
        flight, does not open the gripper, does not go home, and does not start
        a timer. Stopping the arm is the FSM's job (``/pick/pause``); this only
        stops the supervisor from handing it the *next* object.
        """
        if self.state is None or self.state.done:
            return
        self.state.status = PAUSED
        self._publish()

    def resume(self) -> None:
        """Carry on from the latest scene, not from the plan made before the stop."""
        if self.state is None or self.state.status != PAUSED:
            return
        self.state.status = RUNNING
        self._publish()
        self._advance()

    # -------------------------------------------------------------- revision

    def apply_patch(self, *, apply_scope: str = REMAINING_ONLY,
                    destination: str | None = None,
                    remove_object_ids: list[str] | None = None,
                    add_object_ids: list[str] | None = None) -> MissionState | None:
        """Change the mission in place. Bumps ``revision``.

        ``completed_ids`` is never touched: a correction applies to what is left
        to do, never retroactively to what is already in the basket.

        ``CURRENT_AND_REMAINING`` also redirects the object in flight. Whether
        the FSM can honour that depends on how far along it is -- if it is still
        holding and waiting, ``set_place`` catches it; once it is moving to
        place, it does not (that is PLACE_REDIRECT, deliberately deferred).
        Either way the supervisor's own bookkeeping is correct.
        """
        if self.state is None or self.state.done:
            return None
        state = self.state
        if destination is not None:
            state.destination = destination
        for object_id in remove_object_ids or ():
            if object_id in state.pending_ids:
                state.pending_ids.remove(object_id)
                state.skipped_ids.append(object_id)
        for object_id in add_object_ids or ():
            if (object_id not in state.pending_ids
                    and object_id not in state.completed_ids
                    and object_id != state.current_object_id):
                state.pending_ids.append(object_id)
        state.revision += 1
        self._publish()
        # No _advance() here on purpose. A patch is not a resume -- see the
        # PAUSED contract: only a human's command restarts motion.
        if state.status == RUNNING and not state.in_flight:
            self._advance()
        return state

    def redirect_scope(self, apply_scope: str) -> bool:
        """True if the destination change should also chase the object in flight."""
        return apply_scope == CURRENT_AND_REMAINING and bool(self.state and self.state.in_flight)

    # ---------------------------------------------------------------- results

    def on_action_result(self, action_id: str, result: str) -> None:
        """One dispatched action reported back.

        Non-terminal results (``accepted``) are ignored: the action is still
        running and R1 says nothing new goes out until it stops.
        """
        state = self.state
        if state is None or state.done:
            return
        if result not in TERMINAL_RESULTS:
            return
        # A result for an action we are no longer tracking is stale -- it
        # belongs to a mission that was cancelled or replaced. Acting on it
        # would advance the *new* mission by one object for free.
        if action_id and state.current_action_id and action_id != state.current_action_id:
            return

        object_id = state.current_object_id
        state.current_action_id = ""
        state.current_object_id = ""

        if result == "succeeded":
            if object_id:
                state.completed_ids.append(object_id)
        elif result == "cancelled":
            # The user stopped it. Not a failure, and not consumed either --
            # put it back at the front so "계속해" picks up where it left off.
            if object_id:
                state.pending_ids.insert(0, object_id)
        else:                                            # failed | rejected
            self._record_failure(state, object_id)

        self._publish()

        if state.status != RUNNING:
            return
        # R2 -- Command Barrier. Something the user said while this action was
        # running has to be applied before the next object goes out, or the
        # correction lands one object too late.
        if self.host.pending_user_commands():
            return
        self._advance()

    def _record_failure(self, state: MissionState, object_id: str) -> None:
        if not object_id:
            return
        tries = self._retries.get(object_id, 0) + 1
        self._retries[object_id] = tries
        if tries <= state.retry_limit:
            state.pending_ids.insert(0, object_id)       # retry it first
            return
        state.failed_ids.append(object_id)
        if not state.continue_on_failure:
            state.status = FAILED
            self.host.say(f"'{object_id}' 를 {tries}번 실패해서 멈췄습니다. "
                          f"{state.summary()}.")

    # ------------------------------------------------------------- dispatch

    def _advance(self) -> None:
        """Send the next object, if we are allowed to and there is one."""
        state = self.state
        if state is None or state.done or state.status != RUNNING:
            return
        if state.in_flight:                              # R1
            return

        while state.pending_ids:
            # Re-read inside the loop, not once outside it. The re-look below
            # is worthless against a snapshot taken before the first look --
            # and on a real graph the scene is written by a different callback,
            # so the second read genuinely can differ from the first.
            visible = {item.object_id: item for item in self.host.scene_items()}
            object_id = state.pending_ids.pop(0)
            item = visible.get(object_id)
            # R3 -- the id survives, the position does not. If the object is no
            # longer in the scene we do not fall back to where it used to be.
            if item is None or not item.pickable:
                if object_id not in self._reacquired:
                    # One re-look. The scene republishes on the camera's clock,
                    # so an object can be missing from the snapshot we happen to
                    # hold without being gone from the table.
                    self._reacquired.add(object_id)
                    state.pending_ids.append(object_id)
                    continue
                state.skipped_ids.append(object_id)
                continue
            if self._blocked(item.class_name, item.color):
                state.skipped_ids.append(object_id)
                continue

            state.current_object_id = object_id
            state.current_action_id = self.host.dispatch_pick(
                object_id, state.destination, "mission")
            self._publish()
            return

        self._finish()

    def _finish(self) -> None:
        state = self.state
        if state is None or state.done:
            return
        state.status = FAILED if (state.failed_ids and not state.completed_ids) else COMPLETED
        self._publish()
        self.host.say(f"작업을 마쳤습니다. {state.summary()}.")

    # ---------------------------------------------------------------- output

    def _publish(self) -> None:
        if self.state is None:
            return
        # A *deep enough* copy. ``replace()`` alone is shallow, so the published
        # snapshot would share the very lists the supervisor keeps mutating --
        # the GUI would show pending_ids emptying itself and a "completed" list
        # that grew after the fact. Copy the list fields explicitly.
        self.host.on_mission_state(replace(
            self.state,
            pending_ids=list(self.state.pending_ids),
            completed_ids=list(self.state.completed_ids),
            failed_ids=list(self.state.failed_ids),
            skipped_ids=list(self.state.skipped_ids),
        ))
