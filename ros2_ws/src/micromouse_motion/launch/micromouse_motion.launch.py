from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import include_launch_description
import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    micromouse_motion = Node(
        package='micromouse_motion',
        executable='micromouse_motion',
        name= 'micromouse_motion',
        output='screen',
    )
    
    nodes = [micromouse_motion]

    return LaunchDescription(nodes)