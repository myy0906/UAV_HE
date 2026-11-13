# ============================================================
# SemanticTarget
#
# 3D semantic target estimate in project ENU frame.
#
# x = East
# y = North
# z = Up
# ============================================================

uint64 timestamp_us

bool valid
bool stable

string label
float32 confidence

float32 x
float32 y
float32 z

float32 range_m

uint32 sample_count
float32 scatter_xy
