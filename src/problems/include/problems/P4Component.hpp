#pragma once

#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

namespace composition {
class P4Component : public rclcpp::Node {
    public:
        P4Component(const rclcpp::NodeOptions& options);
    private:
        rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client_;
        void teleport();
};

}

