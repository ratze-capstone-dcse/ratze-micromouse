#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam',
            executable='slam.py',
            name='slam_node',
            output='screen',
            parameters=[
                {'map_resolution': 0.05},
                {'map_width': 400},
                {'map_height': 400},
            ]
        )
    ])
