# ============================================================
# NavigationTarget
#
# Semantic target and its safe approach point.
# All coordinates use project ENU frame:
#   x = East
#   y = North
#   z = Up
# ============================================================

uint64 timestamp_us

bool valid
string label
float32 confidence

# Raw visible target surface point estimated by RGB-D
float32 target_x
float32 target_y
float32 target_z

# Safe flight point in front of the target
float32 approach_x
float32 approach_y
float32 approach_z

# ENU yaw that points toward the semantic target
float32 yaw

float32 standoff_distance
float32 target_horizontal_range
