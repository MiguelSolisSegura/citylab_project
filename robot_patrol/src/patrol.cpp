#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class Patrol : public rclcpp::Node {
public:
    // Public methods
    Patrol() : rclcpp::Node("patrol") {
        // Info messages
        RCLCPP_INFO(this->get_logger(), "Creating instance object of Patrol class");
        // Subscription
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10, 
            std::bind(&Patrol::laserCallback, this, std::placeholders::_1));
        // Publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&Patrol::publishVel, this));
        // Pi
        pi_ = std::atan2(0, -1);
    }
private:
    // Private methods
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Reading laser data");
        int size = static_cast<int>(msg->ranges.size());
        float max_value = static_cast<float>(msg->range_max);
        float far_object = 0;
        direction_ = 0;
        for (int i = size * 0.25; i < size * 0.75; i++) {
            if (max_value >= msg->ranges[i] && msg->ranges[i] > far_object) {
                far_object = msg->ranges[i];
                direction_ = -((2 * static_cast<float>(i - i * 0.25) / size) * pi_ - (pi_ / 2));
            }
        }
        RCLCPP_INFO(this->get_logger(), "The furthest object is at: %.2f m", far_object);
        RCLCPP_INFO(this->get_logger(), "The direction is : %.2f radians", direction_);
        RCLCPP_INFO(this->get_logger(), "The direction is : %.2f degrees", direction_ * 180 / pi_);
    }
    void publishVel() {
        RCLCPP_INFO(this->get_logger(), "Publishing angular velocity of : %.2f rad/s", direction_ / 2);
        geometry_msgs::msg::Twist vel_message;
        vel_message.linear.x = 0.1;
        vel_message.angular.z = direction_ / 2;
        publisher_->publish(vel_message);
    }

    // Private attributes
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float direction_;
    float pi_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}