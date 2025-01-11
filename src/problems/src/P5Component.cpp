#include "../include/problems/P5Component.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace composition {

P5Component::P5Component(const rclcpp::NodeOptions& options) : Node("dist_publisher", options) {

    // callbacks
    auto stationary_turtle_callback = [this](const turtlesim::msg::Pose::SharedPtr response) -> void {
        this->stationary_turtle_x = response->x;
        this->stationary_turtle_y = response->y;
    };

    auto moving_turtle_callback = [this](const turtlesim::msg::Pose::SharedPtr response) -> void {
        this->moving_turtle_x = response->x;
        this->moving_turtle_y = response->y;
    };

    auto publisher_callback = [this](void) -> void {
        auto msg = problems::msg::Dist();
        msg.delta_x = abs(this->stationary_turtle_x - this->moving_turtle_x);
        msg.delta_y = abs(this->stationary_turtle_y - this->moving_turtle_y);
        msg.dist = sqrt((this->stationary_turtle_x - this->moving_turtle_x)*(this->stationary_turtle_x - this->moving_turtle_x) + (this->stationary_turtle_y - this->moving_turtle_y)*(this->stationary_turtle_y - this->moving_turtle_y));
        
        publisher_->publish(msg);
    };

    // callback group
    callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto callback_option = rclcpp::SubscriptionOptions();
    callback_option.callback_group = callback_group;

    // topic statistics
    callback_option.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    std::string s_name("/statistics/");
    std::string node_name(this->get_name());
    std::string stat_name = s_name + node_name;
    callback_option.topic_stats_options.publish_topic = stat_name.c_str();

    // for publishing dist msg
    publisher_ = create_publisher<problems::msg::Dist>("/turtle_dist", rclcpp::QoS(10));

    // for getting coordinates
    stationary_subscriber_ = create_subscription<turtlesim::msg::Pose>("/stationary_turtle/pose", rclcpp::QoS(10), stationary_turtle_callback, callback_option);
    moving_subscriber_ = create_subscription<turtlesim::msg::Pose>("/moving_turtle/pose", rclcpp::QoS(10),  moving_turtle_callback, callback_option);

    timer_ = this->create_wall_timer(std::chrono::duration<int, std::chrono::seconds::period>(1), publisher_callback, callback_group);

    std::cout<<"sigma boy 5"<<std::endl;

}

}

RCLCPP_COMPONENTS_REGISTER_NODE(composition::P5Component)