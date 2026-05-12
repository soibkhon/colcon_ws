#!/usr/bin/env python3
"""
wheelchair_local_nav_mppi.launch.py

Same sensor/TF pipeline as wheelchair_local_nav.launch.py but uses the
MPPI (Model Predictive Path Integral) controller instead of DWB.

MPPI is an MPC variant: it samples thousands of random control trajectories,
forward-simulates them, and picks the optimal one via weighted averaging.
This gives smoother motion and better obstacle avoidance around the wheelchair
footprint compared to DWB.

Nav2 params: nav2_params_mppi_local.yaml
  - motion_model: DiffDrive
  - batch_size: 2000 (CPU-safe)
  - horizon: 2.8 s (56 steps × 0.05 s)
  - critics: Constraint, Obstacles, Goal, GoalAngle,
             PathAlign, PathFollow, PathAngle, PreferForward
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_wheelchair_core = get_package_share_directory('wheelchair_core')
    pkg_wheelchair_description = get_package_share_directory('wheelchair_description')

    config_dir = os.path.join(pkg_wheelchair_core, 'config')
    user_config_path = os.path.join(config_dir, 'MID360_config.json')
    urdf_file = os.path.join(pkg_wheelchair_description, 'urdf', 'wheelchair.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav_params_file = LaunchConfiguration('nav_params_file')
    start_livox_driver = LaunchConfiguration('start_livox_driver')
    rviz = LaunchConfiguration('rviz')

    # ── Livox driver ──────────────────────────────────────────────────────────
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        condition=IfCondition(start_livox_driver),
        parameters=[{
            'xfer_format': 0,
            'multi_topic': 1,
            'data_src': 0,
            'publish_freq': 20.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': user_config_path,
        }]
    )

    # ── Robot description ─────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]), value_type=str),
            'use_sim_time': use_sim_time,
        }]
    )

    # ── TF ────────────────────────────────────────────────────────────────────
    tf_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_livox_frame',
        arguments=[
            '--x', '0.54', '--y', '0.0', '--z', '0.28',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    tf_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_footprint',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_footprint',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── Ground filter: RANSAC plane removal → /cloud_right + /cloud_left ─────
    ground_filter = Node(
        package='wheelchair_core',
        executable='ground_filter_node',
        name='ground_filter_node',
        output='screen',
        parameters=[{
            'distance_threshold': 0.12,     # points within 50cm of ground plane are removed
            'max_iterations':     50,
            'min_inlier_ratio':   0.15,   # plane must explain ≥15% of points to be ground
            'normal_z_min':       0.75,   # plane normal must be ≥75% vertical
            'use_sim_time':       use_sim_time,
        }]
    )

    # ── Merged cloud → laser scan (visualisation) ─────────────────────────────
    cloud_merger_for_scan = Node(
        package='wheelchair_core',
        executable='dual_laser_merger_node',
        name='cloud_merger_for_scan',
        output='screen',
        parameters=[{
            'output_topic': '/cloud_merged',
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('input_cloud1', '/livox/lidar_192_168_1_105'),
            ('input_cloud2', '/livox/lidar_192_168_1_172'),
        ]
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01,
            'min_height': -0.1,
            'max_height': 1.5,
            'angle_min': -1.8,
            'angle_max':  1.8,
            'scan_time': 0.05,
            'range_min': 0.3,
            'range_max': 9.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('cloud_in', '/cloud_merged'),
            ('scan',     '/scan'),
        ]
    )

    # ── Nav2 (local costmap only — no map subscription) ──────────────────────
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav_params_file,
            'use_lifecycle_mgr': 'true',
            'map_subscribe_transient_local': 'false',
            'autostart': 'true',
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', os.path.join(config_dir, 'display_point_cloud_ROS2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',       default_value='false'),
        DeclareLaunchArgument('start_livox_driver', default_value='true'),
        DeclareLaunchArgument('rviz',               default_value='true'),
        DeclareLaunchArgument('nav_params_file',
            default_value=os.path.join(config_dir, 'nav2_params_mppi_local.yaml')),

        livox_driver,
        robot_state_publisher,
        tf_livox,
        tf_base_footprint,
        ground_filter,
        cloud_merger_for_scan,
        pointcloud_to_laserscan,
        nav2_bringup,
        rviz_node,
    ])
