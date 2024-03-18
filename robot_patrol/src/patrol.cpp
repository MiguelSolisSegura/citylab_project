#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

class Patrol : public rclcpp::Node {
public:
    // Public methods
    Patrol() : rclcpp::Node("patrol") {
        _subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10, 
            std::bind(&Patrol::laserCallback, this, std::placeholders::_1));
    }
private:
    // Private methods
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        int size = static_cast<int>(msg->ranges.size());
        RCLCPP_INFO(this->get_logger(), "The laser scans array has a size of: %i", size);
    }
    // Private attributes
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _subscription;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}