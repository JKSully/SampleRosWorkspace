from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    publisher_node = Node(package="test_package", executable="publisher")

    ld.add_action(publisher_node)

    return ld
