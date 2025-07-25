#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <memory>

class VelocityPublisher : public rclcpp::Node
{
public:
    VelocityPublisher() : Node("velocity_publisher")
    {
        // Create publisher for /hunter/velocity
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/hunter/velocity", 10);
        
        // Create subscriber for odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ackermann_like_controller/odom", 10,
            std::bind(&VelocityPublisher::odom_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Velocity Publisher Node Started");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract linear velocity in x direction (m/s)
        auto velocity_msg = std_msgs::msg::Float64();
        velocity_msg.data = msg->twist.twist.linear.x;
        
        // Publish velocity
        velocity_pub_->publish(velocity_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityPublisher>());
    rclcpp::shutdown();
    return 0;
}