cmake_minimum_required(VERSION 3.5)
project(imc_ros2_bridge)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bridge_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



include_directories(
  include
  external/imc-cxx
  external/md5
  ${rclcpp_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIR} 
)


## Declare a C++ library
add_library( md5 external/md5/md5.cpp )
#add_library( ros_to_imc_heartbeat src/ros_to_imc/Heartbeat.cpp )
#add_library( imc_to_ros_heartbeat src/imc_to_ros/Heartbeat.cpp )
#add_library( ros_to_imc_gpsfix src/ros_to_imc/GpsFix.cpp )
#add_library( ros_to_imc_goto src/ros_to_imc/Goto.cpp )
#add_library( imc_to_ros_goto src/imc_to_ros/Goto.cpp )
add_library( imc_factory external/imc-cxx/IMC/Base/Factory.cpp )
#add_library( udp_link src/udp_link.cpp )
#add_library( imc_handle src/imc_handle.cpp )
#add_library( imc_to_ros_abort src/imc_to_ros/Abort.cpp )
#add_library( imc_to_ros_plandb src/imc_to_ros/PlanDB.cpp )
#add_library( ros_to_imc_plandb src/ros_to_imc/PlanDB.cpp )
#add_library( ros_to_imc_gpsnavdata src/ros_to_imc/GpsNavData.cpp )
#add_library( imc_to_ros_plancontrol src/imc_to_ros/PlanControl.cpp )
#add_library( ros_to_imc_plancontrolstate src/ros_to_imc/PlanControlState.cpp )
#add_library( ros_to_imc_estimatedstate src/ros_to_imc/EstimatedState.cpp )
#add_library( ros_to_imc_vehiclestate src/ros_to_imc/VehicleState.cpp )
#add_library( ros_to_imc_remotestate src/ros_to_imc/RemoteState.cpp )
#add_library( ros_to_imc_sonardata src/ros_to_imc/SonarData.cpp )
#add_library( ros_to_imc_DesiredHeading src/ros_to_imc/DesiredHeading.cpp )
#add_library( ros_to_imc_DesiredHeadingRate src/ros_to_imc/DesiredHeadingRate.cpp )
#add_library( ros_to_imc_DesiredPitch src/ros_to_imc/DesiredPitch.cpp )
#add_library( ros_to_imc_DesiredRoll src/ros_to_imc/DesiredRoll.cpp )
#add_library( ros_to_imc_DesiredSpeed src/ros_to_imc/DesiredSpeed.cpp )
#add_library( ros_to_imc_DesiredZ src/ros_to_imc/DesiredZ.cpp )


file(GLOB SOURCE_FILES
 src/udp_link.cpp
 src/imc_handle.cpp
 src/imc_to_ros/Goto.cpp
 src/imc_to_ros/Abort.cpp
 src/imc_to_ros/Heartbeat.cpp
 src/imc_to_ros/PlanDB.cpp
 src/imc_to_ros/PlanControl.cpp

 src/ros_to_imc/Heartbeat.cpp
 src/ros_to_imc/Goto.cpp
 src/ros_to_imc/RemoteState.cpp
 src/ros_to_imc/GpsFix.cpp
 src/ros_to_imc/GpsNavData.cpp
 src/ros_to_imc/EstimatedState.cpp

 #include/${PROJECT_NAME}/udp_link.hpp
 #include/${PROJECT_NAME}/imc_handle.hpp
)


add_executable(bridge_node src/bridge_node.cpp ${SOURCE_FILES})
ament_target_dependencies(bridge_node rclcpp
    "sensor_msgs" 
    "geometry_msgs"
    "bridge_msgs"
    "tf2"
    "tf2_geometry_msgs"
)

target_link_libraries(bridge_node md5 imc_factory)


#Install C++ targets
install(TARGETS
  #${PROJECT_NAME}_sensor
  bridge_node
  DESTINATION lib/${PROJECT_NAME}
)


#Install Launch file
#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

