from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    subscriber_node = Node(package="test_package", executable="subscriber")

    ld.add_action(subscriber_node)

    return ld
