#pragma once

#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <problems/msg/dist.hpp>
#include <turtlesim/msg/pose.hpp>

namespace composition {
class P5Component : public rclcpp::Node {
    public:
        P5Component(const rclcpp::NodeOptions& options);
    private:
        rclcpp::Publisher<problems::msg::Dist>::SharedPtr publisher_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr stationary_subscriber_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr moving_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        int stationary_turtle_x;
        int stationary_turtle_y;
        int moving_turtle_x;
        int moving_turtle_y;
        rclcpp::CallbackGroup::SharedPtr callback_group;
};

}