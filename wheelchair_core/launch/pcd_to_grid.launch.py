#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to your PCD file
    pcd_file_path = '/wheelchair_ws/src/FAST_LIO/PCD/scan.pcd'
    
    # Output map file (without extension)
    output_map_path = '/wheelchair_ws/maps/wheelchair_map'
    
    # Create maps directory if it doesn't exist
    os.makedirs('/wheelchair_ws/maps', exist_ok=True)
    
    # PCD to grid conversion node
    pcd_to_grid_node = Node(
        package='pointcloud_to_grid',
        executable='pointcloud_to_grid_node',
        name='pcd_to_grid_converter',
        parameters=[{
            'pcd_file_path': pcd_file_path,
            'output_file_path': output_map_path,
            'resolution': 0.05,  # 5cm resolution
            'min_height': 0.1,   # Filter ground
            'max_height': 2.0,   # Filter ceiling
            'occupancy_threshold': 0.65,
            'free_threshold': 0.196,
            'map_frame': 'map'
        }],
        output='screen'
    )
    
    return LaunchDescription([
        pcd_to_grid_node
    ])