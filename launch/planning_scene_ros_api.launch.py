import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    planning_scene_ros_api_tutorial = Node(
        name="planning_scene_ros_api",
        package="mtc_tutorial",
        executable="planning_scene_ros_api",
        output="screen",
    )

    return LaunchDescription([planning_scene_ros_api_tutorial])