#!/usr/bin/env python3
"""
wheelchair_fastlio.launch.py - FAST-LIO SLAM for wheelchair
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_wheelchair_core = get_package_share_directory('wheelchair_core')
    pkg_fast_lio = get_package_share_directory('fast_lio')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Configuration files
    fastlio_config = os.path.join(pkg_fast_lio, 'config', 'mid360.yaml')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    
    # 1. Wheelchair core (wheel odometry)
    wheelchair_core = IncludeLaunchDescription(
        os.path.join(pkg_wheelchair_core, 'launch', 'wheelchair_core.launch.py'),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 2. Sensors (your existing Livox setup)
    sensors = IncludeLaunchDescription(
        os.path.join(pkg_wheelchair_core, 'launch', 'wheelchair_sensors.launch.py'),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 3. FAST-LIO for 3D SLAM
    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        output='screen',
        parameters=[fastlio_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/cloud_registered', '/cloud_registered_fastlio'),
            ('/Odometry', '/fastlio_odom'),
            ('/path', '/fastlio_path')
        ]
    )
    
    # 4. Transform from FAST-LIO to standard frames
    fastlio_to_odom_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fastlio_to_odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'map', '--child-frame-id', 'camera_init'
        ]
    )
    
    # 5. Sensor fusion (combine wheel odometry + FAST-LIO)
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_fastlio_fusion',
        parameters=[{
            'frequency': 30.0,
            'sensor_timeout': 0.1,
            'two_d_mode': False,  # 3D mode for FAST-LIO
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'map',
            
            # Wheel odometry
            'odom0': '/odom',
            'odom0_config': [True,  True,  False,
                            False, False, True,
                            True,  True,  False,
                            False, False, True,
                            False, False, False],

            # FAST-LIO odometry
            'odom1': '/fastlio_odom',
            'odom1_config': [True,  True,  True,
                            False, False, True,
                            False, False, False,
                            False, False, False,
                            False, False, False],
        }]
    )
    
    # 6. Convert FAST-LIO map to occupancy grid for navigation
    pointcloud_to_grid = Node(
        package='pointcloud_to_grid',  # You may need to install this
        executable='pointcloud_to_grid_node',
        name='pointcloud_to_grid',
        parameters=[{
            'cloud_topic': '/cloud_registered_fastlio',
            'grid_topic': '/map',
            'frame_id': 'map',
            'resolution': 0.05,
            'min_height': 0.1,
            'max_height': 1.5,
            'map_size_x': 100.0,
            'map_size_y': 100.0
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        wheelchair_core,
        sensors,
        fastlio_node,
        fastlio_to_odom_transform,
        robot_localization,
        pointcloud_to_grid,
    ])