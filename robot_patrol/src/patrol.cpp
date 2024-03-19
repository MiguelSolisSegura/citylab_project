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
            std::chrono::milliseconds(100),
            std::bind(&Patrol::publishVel, this));
        // Pi
        pi_ = std::atan2(0, -1);
    }
private:
    // Private methods
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Reading laser data");
        float laser_idx = 0;
        int range_size = int(msg->ranges.size());
        float laser_max = float(msg->range_max);
        float farthest_object = 0;
        closest_object_ = laser_max;
        // SIMULATED ROBOT 
        // Scan ranges front to left 
        // for (int i = 0; i < int(range_size * 0.25); i++) {
        //     if (laser_max > msg->ranges[i] && msg->ranges[i] > farthest_object) {
        //         farthest_object = msg->ranges[i];
        //         laser_idx = i;
        //     }
        //     if (i < int(range_size * 0.125) && msg->ranges[i] < closest_object_) {
        //         closest_object_ = msg->ranges[i];
        //         avoidance_direction_ = -1;
        //     }
        // }
        // // Scan ranges right to front
        // for (int i = int(range_size * 0.75); i < range_size; i++) {
        //     if (laser_max > msg->ranges[i] && msg->ranges[i] > farthest_object) {
        //         farthest_object = msg->ranges[i];
        //         laser_idx = -(range_size - i);
        //     }
        //     if (i > int(range_size * 0.875) && msg->ranges[i] < closest_object_) {
        //         closest_object_ = msg->ranges[i];
        //         avoidance_direction_ = 1;
        //     }
        // }
        // direction_ = laser_idx * 2 * pi_ / range_size; 
        
        // REAL ROBOT
        for (int i = int(range_size * 0.25); i < int(range_size * 0.75); i++) {
            if (laser_max > msg->ranges[i] && msg->ranges[i] > farthest_object) {
                 farthest_object = msg->ranges[i];
                 laser_idx = i;
            }
            if (msg->ranges[i] < closest_object_) {
                 closest_object_ = msg->ranges[i];
                 if (i < range_size * 0.5) {
                    avoidance_direction_ = 1;
                 }
                 else {
                    avoidance_direction_ = -1;
                 }
            }
        }
        laser_idx -= range_size * 0.5;
        direction_ = laser_idx * pi_ / (range_size * 0.5); 

        RCLCPP_INFO(this->get_logger(), "The farthest object is at: %.2f m", farthest_object);
        RCLCPP_INFO(this->get_logger(), "The closest object is at: %.2f m", closest_object_);
        RCLCPP_INFO(this->get_logger(), "The direction is : %.2f radians", direction_);
        RCLCPP_INFO(this->get_logger(), "The direction is : %.2f degrees", direction_ * 180 / pi_);
        
    }
    void publishVel() {
        geometry_msgs::msg::Twist vel_message;
        
        if (closest_object_ > 0.25) {
            vel_message.linear.x = 0.1;
            vel_message.angular.z = direction_ / 2;
        }
        else {
            vel_message.linear.x = 0.0;
            vel_message.angular.z = avoidance_direction_ * 0.25;
        }

        RCLCPP_INFO(this->get_logger(), "Publishing linear velocity of : %.2f m/s", vel_message.linear.x);
        RCLCPP_INFO(this->get_logger(), "Publishing angular velocity of : %.2f rad/s", vel_message.angular.z);
        publisher_->publish(vel_message);
    }

    // Private attributes
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float direction_;
    float closest_object_;
    int avoidance_direction_;
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