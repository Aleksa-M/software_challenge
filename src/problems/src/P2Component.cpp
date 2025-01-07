#include "../include/problems/P2Component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace composition {

P2Component::P2Component(const rclcpp::NodeOptions& options) : Node("rotate_publisher") {

    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel");

    P2Component::rotate();

}

void P2Component::rotate() {

    std::cout<<"Starting rotate callback\n";
    
    auto message = geometry_msgs::msg::Twist({1, 0, 0}, {0, 0, 1});
    publisher_->publish(message);

}

}

RCLCPP_COMPONENTS_REGISTER_NODE(composition::P2Component)