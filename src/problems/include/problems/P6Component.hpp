#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <problems/action/waypoint.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

namespace composition{

class P6Component : public rclcpp::Node {
    public:
        explicit P6Component(const rclcpp::NodeOptions &options);
        using GoalHandleActionServer = rclcpp_action::ServerGoalHandle<problems::action::Waypoint>;
    private:
        rclcpp_action::Server<problems::action::Waypoint>::SharedPtr action_server_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const problems::action::Waypoint::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleActionServer> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleActionServer> goal_handle);
        void execute(const std::shared_ptr<GoalHandleActionServer> goal_handle);
        float x_pos;
        float y_pos;
        float theta_pos;
};

}