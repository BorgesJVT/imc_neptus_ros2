
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <neptus_msgs/msg/plan_db.hpp>
#include <neptus_msgs/msg/plan_control.hpp>
#include <neptus_msgs/msg/remote_state.hpp>
#include <neptus_msgs/msg/estimated_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <iostream>
//#include "udp_link.hpp"
#include "imc_handle.hpp"

//IMC_TO_ROS
#include <imc_ros_bridge/imc_to_ros/Heartbeat.h>
#include <imc_ros_bridge/imc_to_ros/Abort.h>
#include <imc_ros_bridge/imc_to_ros/Goto.h>
#include <imc_ros_bridge/imc_to_ros/PlanDB.h>
#include <imc_ros_bridge/imc_to_ros/PlanControl.h>

//ROS_TO_IMC
#include <imc_ros_bridge/ros_to_imc/Heartbeat.h>
#include <imc_ros_bridge/ros_to_imc/Goto.h>
#include <imc_ros_bridge/ros_to_imc/RemoteState.h>
#include <imc_ros_bridge/ros_to_imc/GpsFix.h>
#include <imc_ros_bridge/ros_to_imc/GpsNavData.h>
#include <imc_ros_bridge/ros_to_imc/EstimatedState.h>



int main(int argc, char * argv[])
{
  int bridge_port = 6004;
  int imc_src = 6;
  int imc_id = 32;
  
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("transponder_node",
                                        rclcpp::NodeOptions()
                                            .allow_undeclared_parameters(true)
                                            .automatically_declare_parameters_from_overrides(true));
  
  std::string bridge_addr = node->get_parameter("bridge_address").as_string();
  std::string neptus_addr = node->get_parameter("neptus_address").as_string();
  std::string sys_name = node->get_parameter("sys_name").as_string();
  
  IMCHandle imc_handle(node, bridge_addr, bridge_port, neptus_addr, sys_name, imc_id, imc_src);

  ros_to_imc::BridgeServer<std_msgs::msg::Empty, IMC::Heartbeat> heartbeat_trans(node, imc_handle, "transponder_heartbeat");
  ros_to_imc::BridgeServer<neptus_msgs::msg::EstimatedState, IMC::EstimatedState> estimatedstate_trans(node, imc_handle, "transponder_estimated_state");
  ros_to_imc::BridgeServer<sensor_msgs::msg::NavSatFix, IMC::GpsFix> gpsfix_trans(node, imc_handle, "transponder_gps");
 

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
