cmake_minimum_required(VERSION 3.8)
project(uav_vln_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}

  "action/ExecuteFlightAction.action"
  "action/ExecuteMission.action"

  "msg/VehicleState.msg"
  "msg/TargetDetection.msg"
  "msg/SemanticTarget.msg"
  "msg/NavigationTarget.msg"
  "msg/MissionCommand.msg"

  "srv/ParseMission.srv"
  "srv/PlanPath.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
