#include "../include/problems/P4Component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace composition {

P4Component::P4Component(const rclcpp::NodeOptions& options) : Node("reset_client", options) {
    // client
    client_ = create_client<turtlesim::srv::TeleportAbsolute>("/moving_turtle/teleport_absolute");

    P4Component::teleport();

    std::cout<<"sigma boy 4"<<std::endl;

}

void P4Component::teleport() {

    std::cout<<"Starting reset callback\n";
    
    auto req = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    req->x = 25.0;
    req->y = 10.0;

    auto callback = [this](rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture response) -> void {
        std::cout<<"reseting moving_turtle"<<std::endl;
        auto output = response.get();
        std::cout<<"reset moving_turtle"<<std::endl;
    };

    auto result = client_->async_send_request(req, callback);

}

}

RCLCPP_COMPONENTS_REGISTER_NODE(composition::P4Component)