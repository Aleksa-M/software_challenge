from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name = 'kill_container',
            namespace = 'kill',
            package = 'rclcpp_components',
            executable = 'component_container',
            composable_node_descriptions = [
                ComposableNode(
                    package ='problems',
                    plugin = 'composition::P1Component',
                    name = 'P1Component'),
            ],
            output = 'screen',
        )
    ])