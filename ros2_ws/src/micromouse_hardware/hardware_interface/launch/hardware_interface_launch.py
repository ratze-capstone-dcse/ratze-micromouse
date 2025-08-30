from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import include_launch_description
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    hardware_interface = Node(
        package='hardware_interface',
        executable='hardware_interface_node',
        parameters=[{'use_sim_time': True}]
    )
    
    nodes = [hardware_interface]

    return LaunchDescription(nodes)