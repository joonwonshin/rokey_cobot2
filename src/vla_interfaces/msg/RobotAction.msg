# One tool call the LLM decided on, addressed to the robot executor.
#
# Only the object handle travels here, never coordinates: the executor looks
# the live position up in its own latest SceneSnapshot at the moment it starts
# moving. A position the LLM saw one API round-trip ago is already stale.

std_msgs/Header header
string action_id
string name          # pick_and_place | set_place
string object_id     # scene object handle; empty for set_place (applies to
                      # whatever the arm is currently holding, not a scene object)
string place         # basket | table | discard; empty means "unspecified" --
                      # for pick_and_place this now means "hold and wait for a
                      # later set_place" (cobot2_ws parks in WAIT_PLACE_TARGET,
                      # vla-bridge-contract.md #13, 2026-08-11), NOT "default to
                      # basket" (that auto-fallback moved to cobot2_ws's own
                      # wait_place_timeout_sec, this side no longer guesses it).
                      # set_place always sets this (required, never empty).
                      # table/discard were UNVERIFIED on real hardware as of
                      # 2026-08-10 (placeholder joint values copied from home),
                      # taught 2026-08-11 -- vla_pick_bridge_node still gates
                      # them behind allow_unverified_place until a full
                      # integrated cycle is run (contract #5/#7).
string reason        # the LLM's one-line rationale, for the GUI and the log
