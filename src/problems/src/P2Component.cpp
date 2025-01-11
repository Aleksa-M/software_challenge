#include "../include/problems/P2Component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace composition {

P2Component::P2Component(const rclcpp::NodeOptions& options) : Node("rotate_publisher", options) {
    // publisher
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", rclcpp::QoS(10));

    // timer
    timer_ = create_wall_timer(std::chrono::duration<double, std::chrono::seconds::period>(0.01), std::bind(&P2Component::rotate, this));

    std::cout<<"sigma boy 2"<<std::endl;

}

void P2Component::rotate() {

    std::cout<<"Starting rotate callback\n";
    
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 1;
    message.linear.y = 0;
    message.linear.z = 0;
    message.angular.x = 0;
    message.angular.y = 0;
    message.angular.z = 1;

    publisher_->publish(message);

}

}

RCLCPP_COMPONENTS_REGISTER_NODE(composition::P2Component)