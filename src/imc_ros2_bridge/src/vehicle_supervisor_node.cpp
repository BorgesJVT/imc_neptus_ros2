#include "rclcpp/rclcpp.hpp"
#include "neptus_msgs/msg/plan_control.hpp"
#include "neptus_msgs/msg/vehicle_state.hpp"
#include "neptus_msgs/msg/entity_monitoring_state.hpp"

#include <imc_ros_bridge/ros_to_imc/VehicleState.h>

class PlanControlNode : public rclcpp::Node
{
public:
    PlanControlNode() : Node("vehicle_supervisor_node")
    {
        setInitialState();
        
        plan_control_subscription_ = this->create_subscription<neptus_msgs::msg::PlanControl>(
            "plan_control", 10,
            std::bind(&PlanControlNode::planControlCallback, this, std::placeholders::_1));

        entity_monitoring_subscription_ = this->create_subscription<neptus_msgs::msg::EntityMonitoringState>(
            "entity_monitoring_state", 10,
            std::bind(&PlanControlNode::entityMonitoringStateCallback, this, std::placeholders::_1));

        vehicle_state_publisher_ = this->create_publisher<neptus_msgs::msg::VehicleState>("vehicle_state", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PlanControlNode::publishVehicleState, this));
        
        RCLCPP_INFO(get_logger(), "Vehicle Supervisor is running...");
    }

private:
    void setInitialState()
    {
        m_vs_.op_mode = neptus_msgs::msg::VehicleState::BOOT;
        m_vs_.maneuver_type = 0xFFFF;
        m_vs_.maneuver_stime = -1;
        m_vs_.maneuver_eta = 0xFFFF;
        m_vs_.error_ents.clear();
        m_vs_.error_count = 0;
        m_vs_.flags = 0;
        m_vs_.last_error.clear();
        m_vs_.last_error_time = -1;
        m_vs_.control_loops = 0;
    }

    void planControlCallback(const neptus_msgs::msg::PlanControl::SharedPtr msg)
    {
        // Empty for now
    }

    void entityMonitoringStateCallback(const neptus_msgs::msg::EntityMonitoringState::SharedPtr msg)
    {
        unsigned m_eboot = 0; // Entities booting
        m_vs_.error_count = 0;
        if (errorMode() || bootMode())
        {
          if (!m_vs_.error_count)
            changeMode(IMC::VehicleState::VS_SERVICE);
          else if (!m_eboot && bootMode())
            changeMode(IMC::VehicleState::VS_ERROR);

          return;
        }
    }

    void publishVehicleState()
    {
        vehicle_state_publisher_->publish(m_vs_);
    }

    void
    changeMode(IMC::VehicleState::OperationModeEnum s, IMC::Message* maneuver = 0)
    {
        m_vs_.op_mode = s;
    }

    inline bool
    errorMode(void) const
    {
        return modeIs(IMC::VehicleState::VS_ERROR);
    }

    inline bool
    bootMode(void) const
    {
        return modeIs(IMC::VehicleState::VS_BOOT);
    }

    inline bool
    modeIs(IMC::VehicleState::OperationModeEnum mode) const
    {
        return m_vs_.op_mode == mode;
    }

    rclcpp::Subscription<neptus_msgs::msg::PlanControl>::SharedPtr plan_control_subscription_;
    rclcpp::Subscription<neptus_msgs::msg::EntityMonitoringState>::SharedPtr entity_monitoring_subscription_;
    rclcpp::Publisher<neptus_msgs::msg::VehicleState>::SharedPtr vehicle_state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    neptus_msgs::msg::VehicleState m_vs_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanControlNode>());
    rclcpp::shutdown();
    return 0;
}
