cmake_minimum_required(VERSION 3.5)
project(bridge_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlanControl.msg"
  "msg/PlanControlState.msg"
  "msg/VehicleState.msg"
  "msg/EstimatedState.msg"
  "msg/RemoteState.msg"
  "msg/SonarData.msg"
  "msg/DesiredHeading.msg"
  "msg/DesiredHeadingRate.msg"
  "msg/DesiredPitch.msg"
  "msg/DesiredRoll.msg"
  "msg/DesiredSpeed.msg"
  "msg/DesiredZ.msg"
  "msg/PlanDB.msg"
  "msg/PlanSpecification.msg"
  "msg/PlanDBInformation.msg"
  "msg/PlanDBState.msg"
  "msg/PlanManeuver.msg"
  "msg/Maneuver.msg"
  "msg/PolygonVertex.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs nav_msgs
)


#ament_export_dependencies(rosidl_default_runtime)
ament_package()
