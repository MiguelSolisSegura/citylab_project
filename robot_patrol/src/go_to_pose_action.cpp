#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_action/types.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include <functional>
#include <memory>

using Pose2D = geometry_msgs::msg::Pose2D;
using GoToPose = robot_patrol::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;
using namespace std::placeholders;

class GoToPoseServer : public rclcpp::Node {
public:
    GoToPoseServer() : rclcpp::Node::Node("go_to_pose_action") {
        this->action_server_ = rclcpp_action::create_server<GoToPose>(
            this,
            "/go_to_pose",
            std::bind(&GoToPoseServer::handle_goal, this, _1, _2),
            std::bind(&GoToPoseServer::handle_cancel, this, _1),
            std::bind(&GoToPoseServer::handle_accepted, this, _1)
        );
    RCLCPP_INFO(this->get_logger(), "Launched /go_to_pose action server.");
    }
private:
    // Attributes
    Pose2D desired_pos_;
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
    // Methods
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoToPose::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), 
        "Received goal with x: %.2f y: %.2f theta: %.2f", goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Excecuting goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<GoToPose::Feedback>();
        auto &current_distance = feedback->current_pos;
        current_distance.x = goal->goal_pos.x;
        current_distance.y = goal->goal_pos.y;
        current_distance.theta = goal->goal_pos.theta;
        goal_handle->publish_feedback(feedback);
        auto result = std::make_shared<GoToPose::Result>();
        result->status = true;
        goal_handle->succeed(result);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPoseServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}