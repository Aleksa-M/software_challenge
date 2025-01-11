#pragma once

#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>

namespace composition {
class P3Component : public rclcpp::Node {
    public:
        P3Component(const rclcpp::NodeOptions& options);
    private:
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
        void spawn();
};

}