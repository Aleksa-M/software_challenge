#pragma once

#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/kill.hpp>

namespace composition {
class P1Component : public rclcpp::Node {
    public:
        P1Component(const rclcpp::NodeOptions& options);
    private:
        rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client_;
        rclcpp::TimerBase::SharedPtr timer_;
        void kill();
};

}