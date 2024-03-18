#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

class Patrol : public rclcpp::Node {
public:
    // Public methods
    Patrol() : rclcpp::Node("patrol") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10, 
            std::bind(&Patrol::laserCallback, this, std::placeholders::_1));
    }
private:
    // Private methods
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int size = static_cast<int>(msg->ranges.size());
        float max_value = static_cast<float>(msg->range_max);
        float far_object = 0;
        for (int i = 0; i < size / 2; i++) {
            if (max_value >= msg->ranges[i] && msg->ranges[i] > far_object) {
                far_object = msg->ranges[i];
                direction_ = static_cast<float>(i)/size * 360;
            }
        }
        RCLCPP_INFO(this->get_logger(), "The furthest object is at: %.2f m", far_object);
        RCLCPP_INFO(this->get_logger(), "The direction is : %.2f degres", direction_);
    }

    // Private attributes
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    float direction_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}