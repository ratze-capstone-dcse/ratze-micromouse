from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import include_launch_description, DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('micromouse_slam'),
        'config'
    )
    
    slam_config = os.path.join(config_dir, 'slam_params.yaml')
    
    slam_dir = get_package_share_directory('slam_toolbox')
    
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_config,
        description='Path to the SLAM parameters file'
    )
    
    slam_launch = include_launch_description(
        PythonLaunchDescriptionSource(os.path.join(
            slam_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': 'false',
                          'slam_params_file': slam_config}.items()
    )
    
    # hardware_interface = Node(
    #     package='micromouse_hardware_interface',
    #     executable='hardware_interface_node',
    #     name='hardware_interface',
    #     output='screen', )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
       
    )
    
    return LaunchDescription([
        # hardware_interface,
        slam_params_arg,
        slam_launch,
        rviz_node
    ])