#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    wheelchair_core_dir = get_package_share_directory('wheelchair_core')
    fast_lio_dir = get_package_share_directory('fast_lio')
    
    # Fast-LIO node
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[os.path.join(fast_lio_dir, 'config', 'mid360.yaml')],
        output='screen'
    )
    
    # Include wheelchair core launch
    wheelchair_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(wheelchair_core_dir, 'launch', 'wheelchair_core.launch.py')
        ])
    )
    
    # Include sensor launch  
    wheelchair_sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(wheelchair_core_dir, 'launch', 'wheelchair_sensors.launch.py')
        ])
    )
    
    return LaunchDescription([
        wheelchair_core_launch,
        wheelchair_sensors_launch,
        fast_lio_node
    ])