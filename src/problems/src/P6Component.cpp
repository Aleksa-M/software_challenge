#include <cmath>
#include <memory>
#include "../include/problems/P6Component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace composition {

P6Component::P6Component(const rclcpp::NodeOptions& options) : Node("moving_turtle_action_server", options) {

    // publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/moving_turtle/cmd_vel", rclcpp::QoS(10));

    // topic statistics
    auto subscriber_callback_ = [this](const turtlesim::msg::Pose::SharedPtr msg) -> void {
        this->P6Component::x_pos = msg->x;
        this->P6Component::y_pos = msg->y;
        this->P6Component::theta_pos = msg->theta;
    };

    // subscriber
    subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/moving_turtle/pose", rclcpp::QoS(10), subscriber_callback_);

    // action server
    action_server_ = rclcpp_action::create_server<problems::action::Waypoint>(
        this, "moving_turtle_action_server",
        std::bind(&P6Component::handle_goal, this, _1, _2),
        std::bind(&P6Component::handle_cancel, this, _1),
        std::bind(&P6Component::handle_accepted, this, _1)
    );

}

rclcpp_action::GoalResponse P6Component::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const problems::action::Waypoint::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Goal Received");
    RCLCPP_INFO(this->get_logger(), "linear X:%f Y:%f", goal->x, goal->y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse P6Component::handle_cancel(const std::shared_ptr<GoalHandleActionServer> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Recieved Request to cancel goal!");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void P6Component::handle_accepted(const std::shared_ptr<GoalHandleActionServer> goal_handle) {
    std::thread{std::bind(&P6Component::execute, this, _1), goal_handle}.detach();
}

void P6Component::execute(const std::shared_ptr<GoalHandleActionServer> goal_handle) {

    // init
    rclcpp::Time start_time = this->now();
    rclcpp::Rate cycle_rate{1};
    const auto goal = goal_handle->get_goal();
    std::unique_ptr<problems::action::Waypoint::Result> result = std::make_unique<problems::action::Waypoint::Result>();

    // calculate angle
    float target_ang = atan((goal->y - P6Component::y_pos) / (goal->x - P6Component::x_pos));
    if (target_ang < 0) target_ang += 2*M_PI;

    // error margins
    float rot_error = 0.01;
    float dist_error = 0.01;

    // rotation
    while (rclcpp::ok() && (abs(P6Component::theta_pos - target_ang) >= rot_error)) {
        // check to see if goal is cancelled
        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Goal Canceled");
            // get the time it has taken thus far and update result
            rclcpp::Time curr_time = this->now();
            rclcpp::Duration time = curr_time - start_time;
            long int duration{time.nanoseconds()};
            result->duration = duration;
            goal_handle->canceled(std::move(result));
            return;
        }

        // publish to cmd_vel but only rotate
        auto rot_msg = std::make_unique<geometry_msgs::msg::Twist>();
        rot_msg->linear.x  = 0;
        rot_msg->linear.y  = 0;
        rot_msg->linear.z  = 0;
        rot_msg->angular.x = 0;
        rot_msg->angular.y = 0;
        rot_msg->angular.z = 1;
        this->publisher_->publish(std::move(rot_msg));

        // feedback
        auto feedback = std::make_unique<problems::action::Waypoint::Feedback>();
        feedback->x_pos = this->P6Component::x_pos;
        feedback->y_pos = this->P6Component::y_pos;
        feedback->theta_pos = this->P6Component::theta_pos;
        goal_handle->publish_feedback(std::move(feedback));
    }

    // check to see if goal is cancelled
    if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");

        // get the time it has taken thus far and update result
        rclcpp::Time curr_time = this->now();
        rclcpp::Duration time = curr_time - start_time;
        long int duration{time.nanoseconds()};
        result->duration = duration;

        goal_handle->canceled(std::move(result));
        return;
    }

    // straight line motion
    while (rclcpp::ok() && (sqrt(pow(P6Component::x_pos - goal->x, 2) + pow(P6Component::y_pos < goal->y, 2)) >= dist_error)) {
        // check to see if goal is cancelled
        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(this->get_logger(), "Goal Canceled");

            // get the time it has taken thus far and update result
            rclcpp::Time curr_time = this->now();
            rclcpp::Duration time = curr_time - start_time;
            long int duration{time.nanoseconds()};
            result->duration = duration;

            goal_handle->canceled(std::move(result));
            return;
        }

        // publish to cmd_vel but only in forward direction
        auto fwd_msg = std::make_unique<geometry_msgs::msg::Twist>();
        fwd_msg->linear.x  = (P6Component::x_pos - goal->x < 0) ? 1 : -1;
        fwd_msg->linear.y  = 0;
        fwd_msg->linear.z  = 0;
        fwd_msg->angular.x = 0;
        fwd_msg->angular.y = 0;
        fwd_msg->angular.z = 0;
        this->publisher_->publish(std::move(fwd_msg));

        auto feedback = std::make_unique<problems::action::Waypoint::Feedback>();
        feedback->x_pos = this->P6Component::x_pos;
        feedback->y_pos = this->P6Component::y_pos;
        feedback->theta_pos = this->P6Component::theta_pos;
        goal_handle->publish_feedback(std::move(feedback));
    }

    if (rclcpp::ok()) {
        rclcpp::Time end = this->now();
        rclcpp::Duration duration = end - start_time;
        long int res_time{duration.nanoseconds()};
        result->duration = res_time;
        goal_handle->succeed(std::move(result));
        RCLCPP_INFO(this->get_logger(), "Finish Executing Goal");
    }
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(composition::P6Component)