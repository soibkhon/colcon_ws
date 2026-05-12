#!/usr/bin/env python3
"""
wheelchair_local_nav.launch.py

Dual-LiDAR obstacle avoidance without a pre-built map.
No AMCL, no map_server — Nav2 uses the odom frame as global frame.
Requires wheelchair_core (wheel odometry → /odom) running separately.

Physical layout:
  [172 left]   [105 right]          both on same plane, front of wheelchair,
      |               |             on top of left/right front wheels.
  left wheel    right wheel

  MID360_config.json extrinsic (y=±217mm, pitch=5.2°) places both clouds
  in the shared livox_frame centred between the two sensors.

Topics:
  /livox/lidar_192_168_1_105  →  /cloud_right  (Nav2 local obstacle layer)
  /livox/lidar_192_168_1_172  →  /cloud_left   (Nav2 local obstacle layer)
  both merged                 →  /cloud_merged → /scan (visualisation)

Nav2 params: nav2_params_local_only.yaml
  Both costmaps roll with the robot in the odom frame.
  No global path planning to map goals — use for reactive obstacle avoidance.
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

    # ── Relay: stable topic names for Nav2 ───────────────────────────────────
    relay_right = Node(
        package='topic_tools',
        executable='relay',
        name='relay_cloud_right',
        parameters=[{
            'input_topic':  '/livox/lidar_192_168_1_105',
            'output_topic': '/cloud_right',
            'use_sim_time': use_sim_time,
        }]
    )

    relay_left = Node(
        package='topic_tools',
        executable='relay',
        name='relay_cloud_left',
        parameters=[{
            'input_topic':  '/livox/lidar_192_168_1_172',
            'output_topic': '/cloud_left',
            'use_sim_time': use_sim_time,
        }]
    )

    # ── Merged cloud → laser scan (visualisation / future SLAM use) ──────────
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
            default_value=os.path.join(config_dir, 'nav2_params_local_only.yaml')),

        livox_driver,
        robot_state_publisher,
        tf_livox,
        tf_base_footprint,
        relay_right,
        relay_left,
        cloud_merger_for_scan,
        pointcloud_to_laserscan,
        nav2_bringup,
        rviz_node,
    ])
