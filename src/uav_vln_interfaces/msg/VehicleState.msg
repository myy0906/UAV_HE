# =============================================================================
# VehicleState
#
# UAV-VLN 项目统一飞行器状态。
# 所有位置、速度均使用 ROS ENU。
#
# x / vx : East
# y / vy : North
# z / vz : Up
# yaw    : ENU yaw [rad]
# =============================================================================

uint64 timestamp_us

bool connected
bool position_valid

bool armed
uint8 nav_state
bool failsafe
bool pre_flight_checks_pass

float32 x
float32 y
float32 z

float32 vx
float32 vy
float32 vz

float32 yaw
