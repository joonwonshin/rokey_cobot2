# One multi-object job, as both tiers and the GUI see it.
#
# This is a VLA-workspace-internal message. It does NOT cross the boundary to
# cobot2_ws -- that boundary is three JSON topics and nothing else (contract).
# Publishing this to the FSM would bind the two workspaces into one build.
#
# Mirrors vla_system.agent.mission.MissionState. Fields are ordered the way the
# GUI reads them: what is left, what is happening now, what already happened.

std_msgs/Header header

string mission_id
uint32 revision                 # bumped on every accepted correction
string original_instruction     # what the user actually said

string status                   # RUNNING | PAUSED | COMPLETED | CANCELLED | FAILED
string scope                    # SNAPSHOT (default) | DYNAMIC

string[] pending_ids            # not yet dispatched, in order
string current_object_id        # "" when nothing is in flight
string current_action_id        # correlates with RobotAction.action_id
string[] completed_ids
string[] failed_ids             # gave up after retry_limit tries
string[] skipped_ids            # vanished from the scene, or forbidden

string destination              # basket | table | discard | "" (= hold, no place)
bool continue_on_failure
uint8 retry_limit

string summary                  # "4개 완료, 1개 실패" -- the line the user hears
