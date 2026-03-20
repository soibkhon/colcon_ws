#!/usr/bin/env python3
"""
wheelchair_autonomous.launch.py - PC-side autonomous navigation stack
Sensors, SLAM, Nav2 - Motor controller runs separately on Jetson
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_wheelchair_core = get_package_share_directory('wheelchair_core')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_fast_lio = get_package_share_directory('fast_lio')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map_file')
    nav_params_file = LaunchConfiguration('nav_params_file')
    slam_mode = LaunchConfiguration('slam_mode', default='true')
    enable_collision_viz = LaunchConfiguration('enable_collision_viz', default='false')
    
    # Default paths
    default_map_file = os.path.join(pkg_wheelchair_core, 'maps', 'wheelchair_map.yaml')
    default_nav_params = os.path.join(pkg_wheelchair_core, 'config', 'nav2_params.yaml')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_file,
        description='Full path to map file for navigation mode'
    )
    
    declare_nav_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=default_nav_params,
        description='Nav2 parameters file'
    )
    
    declare_slam_mode_cmd = DeclareLaunchArgument(
        'slam_mode',
        default_value='true',
        description='Whether to run in SLAM mode (true) or navigation mode (false)'
    )
    
    declare_collision_viz_cmd = DeclareLaunchArgument(
        'enable_collision_viz',
        default_value='false',
        description='Enable collision avoidance visualization in RViz'
    )

    # 1. Sensor System (Livox LiDAR)
    sensors_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wheelchair_core'),
                'launch',
                'wheelchair_sensors.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # 2. FAST-LIO for SLAM (mapping mode)
    fast_lio_group = GroupAction(
        condition=IfCondition(slam_mode),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('fast_lio'),
                        'launch',
                        'mapping.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'config_file': 'mid360.yaml',
                    'rviz': 'false',  # We'll launch our own RViz
                }.items()
            )
        ]
    )
    
    # 3. Octomap Server for 2D map generation
    octomap_node = Node(
        package='octomap_server2',
        executable='octomap_server',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': 0.05,
            'frame_id': 'map',
            'base_frame_id': 'base_link',
            'height_map': True,
            'colored_map': False,
            'filter_ground': True,
            'ground_filter/distance': 0.2,
            'ground_filter/angle': 0.3,
            'ground_filter/plane_distance': 0.07,
            'compress_map': True,
            'sensor_model/max_range': 20.0,
            'sensor_model/hit': 0.7,
            'sensor_model/miss': 0.4,
            'sensor_model/min': 0.12,
            'sensor_model/max': 0.97,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('projected_map', '/map_octomap'),
        ]
    )
    
    # 4. Map Server (navigation mode only)
    map_server_node = Node(
        condition=UnlessCondition(slam_mode),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('map', '/map')
        ]
    )
    
    # 5. Nav2 Navigation Stack
    nav2_bringup_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav_params_file,
            'use_lifecycle_mgr': 'true',
            'map_subscribe_transient_local': 'true',
        }.items()
    )
    
    # 6. Pointcloud to LaserScan converter (for Nav2 compatibility)
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.141592,
            'angle_max': 3.141592,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cloud_in', '/livox/lidar_192_168_1_172'),
            ('scan', '/scan')
        ]
    )
    
    # 7. Optional: Collision Visualization Node (runs on PC for RViz)
    collision_viz_node = Node(
        condition=IfCondition(enable_collision_viz),
        package='wheelchair_core',
        executable='collision_visualizer',
        name='collision_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'safety_distance': 0.6,
            'warning_distance': 1.2,
            'visualization_frequency': 10.0,
        }],
        remappings=[
            ('scan', '/scan'),
            ('robot_pose', '/odom'),
        ]
    )
    
    # 8. RViz with navigation config
    rviz_config_file = os.path.join(pkg_wheelchair_core, 'config', 'nav_wheelchair.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 9. Lifecycle manager for Nav2
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'planner_server', 'controller_server', 
                          'recoveries_server', 'bt_navigator']
        }]
    )

    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time_cmd,
        declare_map_file_cmd,
        declare_nav_params_file_cmd,
        declare_slam_mode_cmd,
        declare_collision_viz_cmd,
        
        # PC-side systems only
        sensors_bringup,
        #fast_lio_group,
        octomap_node,
        map_server_node,
        nav2_bringup_group,
        #pointcloud_to_laserscan_node,
        collision_viz_node,
        rviz_node,
        lifecycle_manager,
    ])