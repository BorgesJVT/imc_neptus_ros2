#include "rclcpp/rclcpp.hpp"
#include "neptus_msgs/msg/entity_state.hpp"
#include "neptus_msgs/msg/entity_monitoring_state.hpp"

class MonitorsNode : public rclcpp::Node
{
public:
    MonitorsNode() : Node("monitors_node")
    {
        // entity_state_subscription_ = this->create_subscription<neptus_msgs::msg::EntityState>(
        //     "entity_state", 10,
        //     std::bind(&MonitorsNode::entityStateCallback, this, std::placeholders::_1));

        entity_monitoring_publisher_ = this->create_publisher<neptus_msgs::msg::EntityMonitoringState>(
            "entity_monitoring_state", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MonitorsNode::publishEntityMonitoringState, this));

        RCLCPP_INFO(get_logger(), "Monitor is running...");
    }

private:
    // void entityStateCallback(const neptus_msgs::msg::EntityState::SharedPtr msg)
    // {
    //     auto monitoring_msg = neptus_msgs::msg::EntityMonitoringState();
        
    //     // Placeholder logic to process entity state and update monitoring state
    //     monitoring_msg.mcount = 0; // Number of monitored entities
    //     monitoring_msg.mnames = "";
    //     monitoring_msg.ecount = 0; // Number of entities with errors
    //     monitoring_msg.enames = "";
    //     monitoring_msg.ccount = 0; // Number of entities with critical errors
    //     monitoring_msg.cnames = "";
    //     monitoring_msg.last_error = "";
    //     monitoring_msg.last_error_time = 0;
        
    //     entity_monitoring_publisher_->publish(monitoring_msg);
    // }

    void publishEntityMonitoringState()
    {
        auto monitoring_msg = neptus_msgs::msg::EntityMonitoringState();
        entity_monitoring_publisher_->publish(m_ems_);
    }

    // rclcpp::Subscription<neptus_msgs::msg::EntityState>::SharedPtr entity_state_subscription_;
    rclcpp::Publisher<neptus_msgs::msg::EntityMonitoringState>::SharedPtr entity_monitoring_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    neptus_msgs::msg::EntityMonitoringState m_ems_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonitorsNode>());
    rclcpp::shutdown();
    return 0;
}
