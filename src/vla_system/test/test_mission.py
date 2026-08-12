"""MissionSupervisor -- the five rules, tested without ROS.

Each test here maps to one of R1..R5 in mission.py. They are not style
preferences; each one is a way the multi-object path breaks on a real graph
that a single-object test cannot show.
"""

import unittest

from vla_system.agent.mission import (
    CANCELLED,
    COMPLETED,
    CURRENT_AND_REMAINING,
    PAUSED,
    RUNNING,
    MissionSupervisor,
    SceneItem,
)


class FakeHost:
    """Records what the supervisor asked for. No robot, no ROS, no clock."""

    def __init__(self, items=(), pending=False):
        self.items = list(items)
        self.pending = pending
        self.dispatched = []            # [(object_id, place, reason)]
        self.said = []
        self.states = []
        self._next_id = 0

    def dispatch_pick(self, object_id, place, reason):
        self.dispatched.append((object_id, place, reason))
        self._next_id += 1
        return f"a{self._next_id}"

    def say(self, text):
        self.said.append(text)

    def scene_items(self):
        return list(self.items)

    def pending_user_commands(self):
        return self.pending

    def on_mission_state(self, state):
        self.states.append(state)

    # test helpers -------------------------------------------------------
    def remove(self, object_id):
        self.items = [i for i in self.items if i.object_id != object_id]

    @property
    def dispatched_ids(self):
        return [object_id for object_id, _place, _reason in self.dispatched]


def apples(n=3):
    return [SceneItem(object_id=f"apple_{i}", class_name="apple", color="red", rank=i)
            for i in range(n)]


def start(host, ids=None, **kwargs):
    supervisor = MissionSupervisor(host, blocked=kwargs.pop("blocked", None))
    supervisor.start(
        mission_id="m1",
        object_ids=ids if ids is not None else [i.object_id for i in host.items],
        instruction="사과 다 담아줘",
        **kwargs,
    )
    return supervisor


class R1OneAtATime(unittest.TestCase):
    """Never dispatch while an action is in flight."""

    def test_only_the_first_object_goes_out(self):
        host = FakeHost(apples(3))
        start(host)
        self.assertEqual(host.dispatched_ids, ["apple_0"])

    def test_accepted_is_not_terminal(self):
        """The single most dangerous confusion: `accepted` means the executor
        took the job, not that it finished. Advancing on it puts two objects in
        flight with one arm."""
        host = FakeHost(apples(3))
        supervisor = start(host)
        supervisor.on_action_result("a1", "accepted")
        self.assertEqual(host.dispatched_ids, ["apple_0"])
        self.assertTrue(supervisor.state.in_flight)

    def test_terminal_result_releases_the_gate(self):
        host = FakeHost(apples(3))
        supervisor = start(host)
        host.remove("apple_0")
        supervisor.on_action_result("a1", "succeeded")
        self.assertEqual(host.dispatched_ids, ["apple_0", "apple_1"])

    def test_stale_result_from_a_replaced_action_is_ignored(self):
        host = FakeHost(apples(3))
        supervisor = start(host)
        supervisor.on_action_result("some-other-action", "succeeded")
        self.assertEqual(host.dispatched_ids, ["apple_0"])


class R2CommandBarrier(unittest.TestCase):
    """Apply what the user said before sending the next object."""

    def test_pending_utterance_holds_the_next_dispatch(self):
        host = FakeHost(apples(3), pending=True)
        supervisor = start(host)
        host.remove("apple_0")
        supervisor.on_action_result("a1", "succeeded")
        # "나머지는 테이블로" is waiting to be applied. Sending apple_1 to the
        # basket now would apply the correction one object too late.
        self.assertEqual(host.dispatched_ids, ["apple_0"])
        self.assertFalse(supervisor.state.in_flight)

    def test_it_resumes_once_the_command_is_applied(self):
        host = FakeHost(apples(3), pending=True)
        supervisor = start(host)
        host.remove("apple_0")
        supervisor.on_action_result("a1", "succeeded")
        host.pending = False
        supervisor.apply_patch(destination="table")
        self.assertEqual(host.dispatched_ids, ["apple_0", "apple_1"])
        self.assertEqual(host.dispatched[-1][1], "table")


class R3NoStaleCoordinates(unittest.TestCase):
    """Ids survive, positions do not."""

    def test_missing_object_is_retried_once_then_skipped(self):
        host = FakeHost(apples(2))
        supervisor = start(host)
        host.remove("apple_0")
        host.remove("apple_1")          # gone from the scene entirely
        supervisor.on_action_result("a1", "succeeded")
        self.assertEqual(host.dispatched_ids, ["apple_0"])
        self.assertIn("apple_1", supervisor.state.skipped_ids)
        self.assertEqual(supervisor.state.status, COMPLETED)

    def test_object_that_reappears_on_the_re_look_is_taken(self):
        """The scene republishes on the camera's clock, so 'not in the snapshot
        I happen to hold' is not the same as 'not on the table'.

        The host here returns an empty scene once and the object afterwards --
        which is what a camera callback landing between two reads looks like.
        """
        host = FakeHost(apples(2))
        supervisor = start(host)
        later = [SceneItem("apple_1", "apple", "red", rank=1)]
        reads = {"n": 0}

        def flaky_scene():
            reads["n"] += 1
            return [] if reads["n"] == 1 else list(later)

        host.scene_items = flaky_scene
        supervisor.on_action_result("a1", "succeeded")
        self.assertEqual(host.dispatched_ids, ["apple_0", "apple_1"])
        self.assertEqual(supervisor.state.skipped_ids, [])

    def test_unpickable_object_is_not_dispatched(self):
        host = FakeHost([SceneItem("apple_0", "apple", "red", pickable=False)])
        supervisor = start(host)
        self.assertEqual(host.dispatched_ids, [])
        self.assertIn("apple_0", supervisor.state.skipped_ids)


class R4Snapshot(unittest.TestCase):
    """The set is fixed when the order is given."""

    def test_new_objects_do_not_join_the_mission(self):
        host = FakeHost(apples(2))
        supervisor = start(host)
        host.items.append(SceneItem("apple_9", "apple", "red", rank=9))
        host.remove("apple_0")
        supervisor.on_action_result("a1", "succeeded")
        host.remove("apple_1")
        supervisor.on_action_result("a2", "succeeded")
        self.assertEqual(host.dispatched_ids, ["apple_0", "apple_1"])
        self.assertEqual(supervisor.state.status, COMPLETED)


class R5FailuresDoNotKillTheMission(unittest.TestCase):

    def test_object_is_retried_up_to_the_limit_then_recorded_failed(self):
        host = FakeHost(apples(2))
        supervisor = start(host, retry_limit=2)
        supervisor.on_action_result("a1", "failed")     # try 1
        supervisor.on_action_result("a2", "failed")     # try 2
        supervisor.on_action_result("a3", "failed")     # over the limit
        self.assertIn("apple_0", supervisor.state.failed_ids)
        self.assertIn("apple_1", host.dispatched_ids)

    def test_the_summary_admits_the_failure(self):
        host = FakeHost(apples(2))
        supervisor = start(host, retry_limit=0)
        supervisor.on_action_result("a1", "failed")
        host.remove("apple_1")
        supervisor.on_action_result("a2", "succeeded")
        self.assertIn("1개 완료", host.said[-1])
        self.assertIn("1개 실패", host.said[-1])


class CancelAndPause(unittest.TestCase):

    def test_cancel_keeps_what_was_already_done(self):
        host = FakeHost(apples(3))
        supervisor = start(host)
        host.remove("apple_0")
        supervisor.on_action_result("a1", "succeeded")
        supervisor.cancel()
        self.assertEqual(supervisor.state.status, CANCELLED)
        self.assertEqual(supervisor.state.completed_ids, ["apple_0"])

    def test_pause_stops_dispatching_but_changes_nothing_else(self):
        host = FakeHost(apples(3))
        supervisor = start(host)
        supervisor.pause()
        before = list(host.dispatched_ids)
        host.remove("apple_0")
        supervisor.on_action_result("a1", "succeeded")
        self.assertEqual(host.dispatched_ids, before)
        self.assertEqual(supervisor.state.status, PAUSED)
        self.assertEqual(supervisor.state.completed_ids, ["apple_0"])

    def test_a_paused_mission_never_restarts_on_its_own(self):
        """The whole point of PAUSED: no amount of *other* events resumes it.
        Only a human's command does."""
        host = FakeHost(apples(3))
        supervisor = start(host)
        supervisor.pause()
        host.remove("apple_0")
        supervisor.on_action_result("a1", "succeeded")
        supervisor.apply_patch(destination="table")
        supervisor.on_action_result("", "succeeded")
        self.assertEqual(host.dispatched_ids, ["apple_0"])
        supervisor.resume()
        self.assertEqual(host.dispatched_ids, ["apple_0", "apple_1"])

    def test_cancelled_object_goes_back_to_the_front(self):
        host = FakeHost(apples(2))
        supervisor = start(host)
        supervisor.pause()
        supervisor.on_action_result("a1", "cancelled")
        self.assertEqual(supervisor.state.pending_ids[0], "apple_0")
        self.assertEqual(supervisor.state.completed_ids, [])


class Revision(unittest.TestCase):

    def test_destination_change_applies_to_what_is_left(self):
        host = FakeHost(apples(3))
        supervisor = start(host, destination="basket")
        host.remove("apple_0")
        supervisor.on_action_result("a1", "succeeded")
        supervisor.apply_patch(destination="table")
        host.remove("apple_1")
        supervisor.on_action_result("a2", "succeeded")
        places = [place for _id, place, _reason in host.dispatched]
        self.assertEqual(places, ["basket", "basket", "table"])

    def test_completed_is_never_rewritten(self):
        host = FakeHost(apples(3))
        supervisor = start(host)
        host.remove("apple_0")
        supervisor.on_action_result("a1", "succeeded")
        supervisor.apply_patch(destination="table", remove_object_ids=["apple_2"])
        self.assertEqual(supervisor.state.completed_ids, ["apple_0"])
        self.assertIn("apple_2", supervisor.state.skipped_ids)
        self.assertEqual(supervisor.state.revision, 1)

    def test_removing_the_rest_ends_the_mission_after_the_one_in_flight(self):
        """"나머지는 됐어" cannot un-dispatch the object already on its way --
        that is PLACE_REDIRECT territory. It clears what has not gone out yet,
        and the mission finishes when the in-flight one reports back."""
        host = FakeHost(apples(3))
        supervisor = start(host)
        host.remove("apple_0")
        supervisor.on_action_result("a1", "succeeded")   # apple_1 now in flight
        supervisor.apply_patch(remove_object_ids=["apple_1", "apple_2"])
        self.assertEqual(supervisor.state.status, RUNNING)
        self.assertIn("apple_2", supervisor.state.skipped_ids)
        host.remove("apple_1")
        supervisor.on_action_result("a2", "succeeded")
        self.assertEqual(supervisor.state.status, COMPLETED)
        self.assertEqual(supervisor.state.completed_ids, ["apple_0", "apple_1"])

    def test_redirect_scope_only_chases_the_current_object_when_asked(self):
        host = FakeHost(apples(2))
        supervisor = start(host)
        self.assertTrue(supervisor.redirect_scope(CURRENT_AND_REMAINING))
        self.assertFalse(supervisor.redirect_scope("REMAINING_ONLY"))


class Blocked(unittest.TestCase):

    def test_a_prohibition_bites_at_dispatch_time_not_only_at_planning(self):
        """The user can forbid a class *after* the mission started. Filtering
        once, up front, would send it anyway."""
        forbidden = set()
        host = FakeHost([SceneItem("apple_0", "apple", "red", rank=0),
                         SceneItem("cup_1", "cup", "white", rank=1)])
        supervisor = start(host, blocked=lambda c, _color: c in forbidden)
        forbidden.add("cup")
        host.remove("apple_0")
        supervisor.on_action_result("a1", "succeeded")
        self.assertEqual(host.dispatched_ids, ["apple_0"])
        self.assertIn("cup_1", supervisor.state.skipped_ids)


class StatePublishing(unittest.TestCase):

    def test_every_change_is_published_and_the_copy_is_detached(self):
        host = FakeHost(apples(2))
        supervisor = start(host)
        self.assertTrue(host.states)
        first = host.states[0]
        supervisor.state.completed_ids.append("apple_0")
        self.assertEqual(first.completed_ids, [])

    def test_status_reaches_a_terminal_value(self):
        host = FakeHost(apples(1))
        supervisor = start(host)
        host.remove("apple_0")
        supervisor.on_action_result("a1", "succeeded")
        self.assertEqual(supervisor.state.status, COMPLETED)
        self.assertNotEqual(supervisor.state.status, RUNNING)


if __name__ == "__main__":
    unittest.main()
