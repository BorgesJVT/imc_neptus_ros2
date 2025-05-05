#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "neptus_msgs/msg/estimated_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class OdometryToEstimatedStateNode : public rclcpp::Node
{
public:
    OdometryToEstimatedStateNode() : Node("odometry_to_estimated_state_node")
    {
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dynamics/odometry", 10,
            std::bind(&OdometryToEstimatedStateNode::odometryCallback, this, std::placeholders::_1));

        estimated_state_publisher_ = this->create_publisher<neptus_msgs::msg::EstimatedState>("estimated_state", 10);
        initial_lat_rad = -0.40174237833249615; // -0.39840630835274565;
        initial_lon_rad = -0.773666187225512; // -0.7535904104444776;
        latitude_initial_deg = initial_lat_rad * (180.0 / M_PI);
        longitude_initial_deg = initial_lon_rad * (180.0 / M_PI);
    }

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto estimated_state_msg = neptus_msgs::msg::EstimatedState();
        
        double Re = 6371000.0;
        double delta_lat = msg->pose.pose.position.x / Re;
        double delta_lon = msg->pose.pose.position.y / (Re * std::cos(initial_lat_rad));

        estimated_state_msg.lat = initial_lat_rad;// + delta_lat;
        estimated_state_msg.lon = initial_lon_rad;// + delta_lon;

        tf2::Quaternion quaternion(msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 rot;
        rot.setRotation(quaternion);
        double roll, pitch, yaw;
        rot.getRPY(roll, pitch, yaw);

        estimated_state_msg.height = msg->pose.pose.position.z;
        estimated_state_msg.x = msg->pose.pose.position.x;
        estimated_state_msg.y = msg->pose.pose.position.y;
        estimated_state_msg.z = msg->pose.pose.position.z;
        estimated_state_msg.phi = roll;
        estimated_state_msg.theta = pitch;
        estimated_state_msg.psi = yaw;
        // estimated_state_msg.u = msg->twist.twist.linear.x;
        // estimated_state_msg.v = msg->twist.twist.linear.y;
        // estimated_state_msg.w = msg->twist.twist.linear.z;
        // estimated_state_msg.vx = msg->twist.twist.linear.x;
        // estimated_state_msg.vy = msg->twist.twist.linear.y;
        // estimated_state_msg.vz = msg->twist.twist.linear.z;
        // estimated_state_msg.p = msg->twist.twist.angular.x;
        // estimated_state_msg.q = msg->twist.twist.angular.y;
        // estimated_state_msg.r = msg->twist.twist.angular.z;
        // estimated_state_msg.depth = msg->pose.pose.position.z;
        // estimated_state_msg.alt = msg->pose.pose.position.z;
        
        estimated_state_publisher_->publish(estimated_state_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::Publisher<neptus_msgs::msg::EstimatedState>::SharedPtr estimated_state_publisher_;
    double initial_lat_rad;
    double initial_lon_rad;
    double latitude_initial_deg;
    double longitude_initial_deg;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryToEstimatedStateNode>());
    rclcpp::shutdown();
    return 0;
}
