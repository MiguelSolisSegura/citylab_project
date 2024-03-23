#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <functional>
#include <memory>
#include <cstdlib>
#include <stdexcept>
#include <string>

using GetDirection = robot_patrol::srv::GetDirection;

class DirectionService : public rclcpp::Node {
public:
    // Methods
    DirectionService(float right_index_multiplier) : rclcpp::Node::Node("direction_service") {
        using namespace std::placeholders;
        service_ = this->create_service<GetDirection>(
            "/direction_service", 
            std::bind(&DirectionService::handle_service, this, _1, _2));

        this->right_index_multiplier_ = right_index_multiplier;

        RCLCPP_INFO(this->get_logger(), "Server started.");
    }
private:
    // Attributes
    rclcpp::Service<GetDirection>::SharedPtr service_;
    float right_index_multiplier_;

    // Methods
    void handle_service(const std::shared_ptr<GetDirection::Request> request, std::shared_ptr<GetDirection::Response> response) {
        int array_size = request->laser_data.ranges.size();
        int right_idx = array_size * right_index_multiplier_;
        float total_dist_sec_right = 0;
        float total_dist_sec_front = 0;
        float total_dist_sec_left = 0;
        int idx = right_idx;
        int half_array = array_size / 2;
        for (int i = 0; i < half_array; i++) {
            if (i < half_array / 3) {
                total_dist_sec_right += request->laser_data.ranges[idx];
            }
            else if (i < 2 * half_array / 3) {
                total_dist_sec_front += request->laser_data.ranges[idx];
            }
            else {
                total_dist_sec_left += request->laser_data.ranges[idx];
            }
            idx++;
            if (idx >= array_size) {idx = 0;}
        }
        if (total_dist_sec_right > total_dist_sec_front && total_dist_sec_right > total_dist_sec_left) {
            response->direction = "right";
        }
        else {
            response->direction = total_dist_sec_front > total_dist_sec_left ? "forward" : "left";
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    float right_index_multiplier = 0.75;
    auto node = std::make_shared<DirectionService>(right_index_multiplier);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}