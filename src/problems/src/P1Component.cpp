#include "../include/problems/P1Component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace composition {

P1Component::P1Component(const rclcpp::NodeOptions& options) : Node("kill_client", options) {
    // client
    client_ = create_client<turtlesim::srv::Kill>("/kill");

    // callback
    //timer_ = create_wall_timer(std::chrono::duration<int, std::chrono::seconds::period>(2), std::bind(&P1Component::kill, this));

    P1Component::kill();

    std::cout<<"sigma boy"<<std::endl;


}

void P1Component::kill() {

    std::cout<<"starting kill callback\n";

    auto topics = this->get_topic_names_and_types();

    for (const auto& topic : topics) {
        // each turtle has 3 topics, so only look for the same one to not duplicate
        if (topic.first.find("turtle") != std::string::npos && topic.first.find("pose") != std::string::npos) {
            std::cout<<"killing turtle\n";
            // getting name from topic
            size_t start_pos = topic.first.find("/turtle") + 1;
            size_t end_pos = topic.first.find("/pose");
            std::string turtle_name = topic.first.substr(start_pos, end_pos - start_pos);

            // create kill request
            auto req = std::make_shared<turtlesim::srv::Kill::Request>();
            req->name = turtle_name;

            
            // callback
            auto callback = [this, turtle_name](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture response) -> void {
                std::cout<<"killed: "<<turtle_name<<"\n";
                auto output = response.get();
                std::cout<<"response: "<<output<<"\n";
                //rclcpp::shutdown();
            };
            
            auto result = client_->async_send_request(req, callback);
        }
    }

    std::cout<<"here"<<std::endl;

}

}

RCLCPP_COMPONENTS_REGISTER_NODE(composition::P1Component)