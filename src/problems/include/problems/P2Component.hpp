#pragma once

#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace composition {
class P2Component : public rclcpp::Node {
    public:
        P2Component(const rclcpp::NodeOptions& options);
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        void rotate();
};

}