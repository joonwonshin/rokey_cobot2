"""The tool schemas are the API contract with the model; a typo breaks a turn."""

import unittest

from vla_system.agent.tools import (
    APPLY_SCOPES,
    MISSION_TOOLS,
    MOTION_TOOLS,
    TERMINAL_TOOLS,
    TOOL_NAMES,
    TOOLS,
)


class ToolSchemaTest(unittest.TestCase):
    def test_every_tool_the_plan_calls_for_exists(self):
        expected = {
            "pick_and_place",
            "set_place",
            "cancel_current_action",
            "reset_after_stop",
            "ask_clarification",
            "wait",
            # Mission-level, added 2026-08-12. They change what the supervisor
            # intends to do next; none of them moves the arm.
            "modify_mission",
            "cancel_mission",
            "pause_mission",
            "resume_mission",
            # The hold path, added 2026-08-12 (contract #13/#14).
            "pick_and_hold",
            "release_held",
        }
        self.assertEqual(set(TOOL_NAMES), expected)

    def test_there_is_exactly_one_way_to_name_a_destination(self):
        """set_place is that tool. A `place_held` synonym emitting the same
        command would be a coin flip the model has to win every turn -- the
        plan asked for one, and this is why there isn't one."""
        naming_a_place = {
            tool["name"] for tool in TOOLS
            if "place" in tool["parameters"]["properties"]
            and tool["name"] != "pick_and_place"
        }
        self.assertEqual(naming_a_place, {"set_place"})

    def test_hold_tools_do_not_take_a_destination(self):
        """The whole point of both: no destination. If either grows a `place`
        argument it has silently become pick_and_place again."""
        for name in ("pick_and_hold", "release_held"):
            tool = next(t for t in TOOLS if t["name"] == name)
            self.assertNotIn("place", tool["parameters"]["properties"], name)

    def test_names_are_unique(self):
        self.assertEqual(len(TOOL_NAMES), len(set(TOOL_NAMES)))

    def test_schemas_are_in_the_flat_responses_api_shape(self):
        for tool in TOOLS:
            self.assertEqual(tool["type"], "function")
            self.assertIn("name", tool)
            self.assertIn("parameters", tool)

    def test_strict_mode_requires_every_property(self):
        """With strict=True the API rejects an optional property outright."""
        for tool in TOOLS:
            if not tool.get("strict"):
                continue
            parameters = tool["parameters"]
            self.assertFalse(parameters["additionalProperties"], tool["name"])
            self.assertEqual(
                set(parameters["properties"]), set(parameters["required"]), tool["name"]
            )

    def test_every_tool_is_described_for_the_model(self):
        for tool in TOOLS:
            self.assertTrue(tool.get("description", "").strip(), tool["name"])

    def test_motion_and_terminal_tools_are_real_tools(self):
        for name in MOTION_TOOLS + TERMINAL_TOOLS + MISSION_TOOLS:
            self.assertIn(name, TOOL_NAMES)

    def test_mission_tools_do_not_move_the_arm(self):
        """The split matters at dispatch: MOTION_TOOLS are withheld when a stop
        landed mid-decision. Mission tools must not be in that set -- correcting
        a plan after a stop is exactly what should still be allowed."""
        self.assertFalse(set(MISSION_TOOLS) & set(MOTION_TOOLS))

    def test_modify_mission_scopes_are_the_two_the_supervisor_knows(self):
        tool = next(t for t in TOOLS if t["name"] == "modify_mission")
        properties = tool["parameters"]["properties"]
        self.assertEqual(set(properties["apply_scope"]["enum"]), set(APPLY_SCOPES))
        self.assertEqual(
            set(properties["new_destination"]["enum"]),
            {"basket", "table", "discard", None},
        )

    def test_pause_mission_is_not_the_stop_button(self):
        """정지 goes GUI -> robot with no LLM in the path. If the description
        ever invites the model to handle "멈춰", that guarantee is gone."""
        tool = next(t for t in TOOLS if t["name"] == "pause_mission")
        self.assertIn("쓰지 마라", tool["description"])

    def test_picking_tools_take_an_object_handle(self):
        for tool in TOOLS:
            if tool["name"] == "pick_and_place":
                self.assertIn("object_id", tool["parameters"]["properties"])

    def test_every_tool_carries_a_sentence_for_the_user(self):
        """Without this the robot can act, or stop, in total silence."""
        for tool in TOOLS:
            properties = tool["parameters"]["properties"]
            self.assertTrue(
                {"say", "question"} & set(properties),
                f"{tool['name']} has nothing to tell the user",
            )

    def test_pick_and_place_carries_a_destination(self):
        """place is nullable (contract #13, 2026-08-11): None means "hold and
        wait for a later set_place" instead of guessing basket -- the enum
        must allow it alongside the three real destinations."""
        tool = next(t for t in TOOLS if t["name"] == "pick_and_place")
        properties = tool["parameters"]["properties"]
        self.assertEqual(
            set(properties["place"]["enum"]), {"basket", "table", "discard", None}
        )
        self.assertIn("null", properties["place"]["type"])

    def test_set_place_carries_a_destination(self):
        """Unlike pick_and_place, set_place's place is never optional -- it
        exists only to answer the question WAIT_PLACE_TARGET asked."""
        tool = next(t for t in TOOLS if t["name"] == "set_place")
        properties = tool["parameters"]["properties"]
        self.assertEqual(
            set(properties["place"]["enum"]), {"basket", "table", "discard"}
        )
        self.assertNotIn("object_id", properties)

    def test_clarification_can_carry_candidates_for_the_gui_to_crop(self):
        tool = next(t for t in TOOLS if t["name"] == "ask_clarification")
        properties = tool["parameters"]["properties"]
        self.assertEqual(properties["object_ids"]["type"], "array")
        self.assertIn("question", properties)


if __name__ == "__main__":
    unittest.main()
