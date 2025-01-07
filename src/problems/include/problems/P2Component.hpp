#pragma once

#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace composition {
class P1Component : public rclcpp::Node {
    public:
        P2Component(const rclcpp::NodeOptions& options);
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_
};

}