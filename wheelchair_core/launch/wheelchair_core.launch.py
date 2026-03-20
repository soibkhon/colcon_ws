#!/usr/bin/env python3
"""
wheelchair_core.launch.py - Main launch file for wheelchair core control
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_wheelchair_core = get_package_share_directory('wheelchair_core')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_file = LaunchConfiguration('config_file')
    
    # Paths
    default_config_file = os.path.join(pkg_wheelchair_core, 'config', 'wheelchair_params.yaml')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Full path to the wheelchair configuration file'
    )
    
    # Wheelchair core node
    wheelchair_core_node = Node(
        package='wheelchair_core',
        executable='wheelchair_core_node',
        name='wheelchair_core',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odom', '/odom'),
            ('cmd_vel', '/cmd_vel'),
            ('joint_states', '/joint_states'),
        ]
    )
    
    # Joint state publisher for other joints (non-wheel)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'source_list': ['/joint_states'],
            'rate': 50
        }]
    )
    
    # Robot state publisher for TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': LaunchConfiguration('robot_description', default=''),
            'publish_frequency': 50.0
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_config_file_cmd,
        wheelchair_core_node,
        joint_state_publisher_node,
        
    ])


