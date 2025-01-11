#include "../include/problems/P3Component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace composition {

P3Component::P3Component(const rclcpp::NodeOptions& options) : Node("spawn_client", options) {
    // client
    client_ = create_client<turtlesim::srv::Spawn>("/spawn");

    P3Component::spawn();

    std::cout<<"sigma boy 3"<<std::endl;

}

void P3Component::spawn() {

    std::cout<<"Starting spawn callback\n";
    
    auto stationary_req = std::make_shared<turtlesim::srv::Spawn::Request>();
    stationary_req->name = "stationary_turtle";
    stationary_req->x = 5.0;
    stationary_req->y = 5.0;

    auto stationary_callback = [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture response) -> void {
        std::cout<<"spawning stationary_turtle"<<std::endl;
        auto output = response.get();
        std::cout<<"spawned stationary_turtle"<<std::endl;
    };

    auto stationary_result = client_->async_send_request(stationary_req, stationary_callback);

    auto moving_req = std::make_shared<turtlesim::srv::Spawn::Request>();
    moving_req->name = "moving_turtle";
    moving_req->x = 25.0;
    moving_req->y = 10.0;

    auto moving_callback = [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture response) -> void {
        std::cout<<"spawning moving_turtle"<<std::endl;
        auto output = response.get();
        std::cout<<"spawned moving_turtle"<<std::endl;
    };

    auto moving_result = client_->async_send_request(moving_req, moving_callback);

}

}

RCLCPP_COMPONENTS_REGISTER_NODE(composition::P3Component)