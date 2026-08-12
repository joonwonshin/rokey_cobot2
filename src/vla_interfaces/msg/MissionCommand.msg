# A correction to the mission in progress, from Tier 2 or from the GUI.
#
# Separate from RobotAction on purpose: RobotAction moves the arm, this one
# only changes what the supervisor intends to do next. Nothing here reaches the
# FSM directly -- the supervisor decides what, if anything, to dispatch.

std_msgs/Header header

# MODIFY   change destination and/or the object set (see apply_scope)
# CANCEL   stop for good; completed_ids is kept
# PAUSE    stop dispatching. Does not stop the arm -- that is /pick/pause.
# RESUME   carry on, replanned from the latest scene
string command

string mission_id               # "" = whichever mission is active

# MODIFY only ------------------------------------------------------------
# CURRENT_AND_REMAINING  also redirect the object already in flight
# REMAINING_ONLY         leave the one in flight alone (default, safer)
string apply_scope

string new_destination          # "" = leave the destination as it is
string[] remove_object_ids
string[] add_object_ids

string say                      # what to tell the user, if anything
