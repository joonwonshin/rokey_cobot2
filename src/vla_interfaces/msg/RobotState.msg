# Ground truth about the arm, injected into every LLM call.
#
# The conversation history is the LLM's memory, but memory drifts. What the
# gripper is actually holding is only ever read from here.

std_msgs/Header header

string status                # idle | moving | holding | waiting_place | stopping | error
                              # waiting_place: gripper closed, holding the object,
                              # parked in cobot2_ws's WAIT_PLACE_TARGET waiting for
                              # a set_place command (contract #13, 2026-08-11)
string holding_object_id     # empty when the gripper is open
string holding_class_name

string current_action_id     # action in flight, empty when idle
string current_action

string last_action_id
string last_action
string last_result           # succeeded | failed | cancelled | rejected
string details

bool motion_enabled
