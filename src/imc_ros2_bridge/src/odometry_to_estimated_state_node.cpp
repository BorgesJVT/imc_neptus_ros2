#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "neptus_msgs/msg/estimated_state.hpp"

class OdometryToEstimatedStateNode : public rclcpp::Node
{
public:
    OdometryToEstimatedStateNode() : Node("odometry_to_estimated_state_node")
    {
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/lauv/dynamics/odometry", 10,
            std::bind(&OdometryToEstimatedStateNode::odometryCallback, this, std::placeholders::_1));

        estimated_state_publisher_ = this->create_publisher<neptus_msgs::msg::EstimatedState>("estimated_state", 10);
        initial_lat_rad = 0.71881385;
        initial_lon_rad = -0.15195186;
        latitude_initial_deg = initial_lat_rad * (180.0 / M_PI);
        longitude_initial_deg = initial_lon_rad * (180.0 / M_PI);
    }

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto estimated_state_msg = neptus_msgs::msg::EstimatedState();
        
        double Re = 6378137.0;
        // double delta_lat = msg->pose.pose.position.x / Re * (180.0 / M_PI);
        // double delta_lon = msg->pose.pose.position.y / (Re * cos(initial_lat_rad)) * (180.0 / M_PI);
        // double lat = delta_lat + latitude_initial_deg;
        // double lon = delta_lon + longitude_initial_deg;
        // estimated_state_msg.lat = lat; // Placeholder, requires GPS data transformation
        // estimated_state_msg.lon = lon; // Placeholder, requires GPS data transformation

        estimated_state_msg.lat = initial_lat_rad + (msg->pose.pose.position.x / Re);
        estimated_state_msg.lon = initial_lon_rad + (msg->pose.pose.position.y / Re);

        // estimated_state_msg.height = msg->pose.pose.position.z;
        // estimated_state_msg.x = msg->pose.pose.position.x;
        // estimated_state_msg.y = msg->pose.pose.position.y;
        // estimated_state_msg.z = msg->pose.pose.position.z;
        // estimated_state_msg.phi = msg->pose.pose.orientation.x;
        // estimated_state_msg.theta = msg->pose.pose.orientation.y;
        // estimated_state_msg.psi = msg->pose.pose.orientation.z;
        // estimated_state_msg.u = msg->twist.twist.linear.x;
        // estimated_state_msg.v = msg->twist.twist.linear.y;
        // estimated_state_msg.w = msg->twist.twist.linear.z;
        // estimated_state_msg.vx = msg->twist.twist.linear.x;
        // estimated_state_msg.vy = msg->twist.twist.linear.y;
        // estimated_state_msg.vz = msg->twist.twist.linear.z;
        // estimated_state_msg.p = msg->twist.twist.angular.x;
        // estimated_state_msg.q = msg->twist.twist.angular.y;
        // estimated_state_msg.r = msg->twist.twist.angular.z;
        // estimated_state_msg.depth = -msg->pose.pose.position.z;
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
