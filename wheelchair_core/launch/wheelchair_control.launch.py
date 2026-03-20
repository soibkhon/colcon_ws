#!/usr/bin/env python3
"""
wheelchair_control.launch.py - Launch file with diff_drive_controller
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_wheelchair_core = get_package_share_directory('wheelchair_core')
    pkg_wheelchair_description = FindPackageShare('wheelchair_description')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Paths
    wheelchair_params = os.path.join(pkg_wheelchair_core, 'config', 'wheelchair_params.yaml')
    controller_params = os.path.join(pkg_wheelchair_core, 'config', 'diff_drive_controller.yaml')
    xacro_file = PathJoinSubstitution([pkg_wheelchair_description, 'urdf', 'wheelchair.xacro'])
    
    # Process the xacro file
    robot_description = Command(['xacro ', xacro_file])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }]
    )
    
    # Wheelchair core node
    wheelchair_core_node = Node(
        package='wheelchair_core',
        executable='wheelchair_core_node',
        name='wheelchair_core',
        output='screen',
        parameters=[wheelchair_params, {'use_sim_time': use_sim_time}],
    )
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )
    
    # Spawn controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Delay controller spawning to ensure controller manager is ready
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner]
    )
    
    delayed_diff_drive_controller_spawner = TimerAction(
        period=3.0,
        actions=[diff_drive_controller_spawner]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        robot_state_publisher,
        wheelchair_core_node,
        controller_manager,
        delayed_joint_state_broadcaster_spawner,
        delayed_diff_drive_controller_spawner,
    ])