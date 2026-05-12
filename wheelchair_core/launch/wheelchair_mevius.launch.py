#!/usr/bin/env python3
"""
wheelchair_mevius.launch.py

Dual-LiDAR + FAST-LIO pipeline for odometry comparison.

Physical layout:
  Both MID360s on the same plane at the front, on top of left/right front wheels.
  JSON extrinsic (y=±217mm, pitch=5.78°) puts both clouds in shared livox_frame.

Odometry topics for comparison:
  /odom           — wheel encoder odometry (from wheelchair_core)
  /fastlio_odom   — LiDAR-inertial odometry from FAST-LIO

Pipeline:
  /livox/lidar_192_168_1_105 (right) → FAST-LIO → /cloud_registered_body → /cloud_right_filtered → Nav2
  /livox/lidar_192_168_1_172 (left)  ──────────────────────────────────── → /cloud_left_filtered  → Nav2
  /livox/imu_192_168_1_105           → Madgwick → /imu/data
  both clouds merged                 → /cloud_merged → pointcloud_to_laserscan → /scan → AMCL

TODO: in fast_lio/config/mid360.yaml set:
        lid_topic: "livox/lidar_192_168_1_105"
        imu_topic: "livox/imu_192_168_1_105"
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
    pkg_fast_lio = get_package_share_directory('fast_lio')

    config_dir = os.path.join(pkg_wheelchair_core, 'config')
    user_config_path = os.path.join(config_dir, 'MID360_config.json')
    fastlio_config = os.path.join(pkg_fast_lio, 'config', 'mid360.yaml')
    urdf_file = os.path.join(pkg_wheelchair_description, 'urdf', 'wheelchair_with_arm.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    nav_params_file = LaunchConfiguration('nav_params_file')
    rviz = LaunchConfiguration('rviz')

    # ── Livox driver ──────────────────────────────────────────────────────────
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
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
    tf_livox_frame = Node(
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

    # Anchors FAST-LIO's starting frame to odom
    tf_odom_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_camera_init',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom',
            '--child-frame-id', 'camera_init',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── IMU filter ────────────────────────────────────────────────────────────
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('imu/data_raw', '/livox/imu_192_168_1_105'),
            ('imu/data',     '/imu/data'),
        ]
    )

    # ── FAST-LIO ──────────────────────────────────────────────────────────────
    # /Odometry → /fastlio_odom  so it can be compared against wheel /odom
    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        output='screen',
        parameters=[fastlio_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/Odometry', '/fastlio_odom'),
        ]
    )

    # ── Relay clouds to Nav2 topic names ─────────────────────────────────────
    relay_cloud_right = Node(
        package='topic_tools',
        executable='relay',
        name='relay_cloud_right',
        parameters=[{
            'input_topic':  '/cloud_registered_body',
            'output_topic': '/cloud_right_filtered',
            'use_sim_time': use_sim_time,
        }]
    )

    relay_cloud_left = Node(
        package='topic_tools',
        executable='relay',
        name='relay_cloud_left',
        parameters=[{
            'input_topic':  '/livox/lidar_192_168_1_172',
            'output_topic': '/cloud_left_filtered',
            'use_sim_time': use_sim_time,
        }]
    )

    # ── Merged cloud → laser scan for AMCL ───────────────────────────────────
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
            'angle_min': -3.14159,
            'angle_max':  3.14159,
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

    # ── Map server + AMCL ─────────────────────────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': use_sim_time,
        }],
        remappings=[('map', '/map')]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav_params_file],
        remappings=[('scan', '/scan')]
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }]
    )

    # ── Nav2 ─────────────────────────────────────────────────────────────────
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
            'map_subscribe_transient_local': 'true',
            'autostart': 'true',
        }.items()
    )

    # ── Wheelchair core (wheel odometry → /odom) ──────────────────────────────
    wheelchair_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wheelchair_core, 'launch', 'wheelchair_core.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
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
        DeclareLaunchArgument('use_sim_time',    default_value='false'),
        DeclareLaunchArgument('rviz',            default_value='true'),
        DeclareLaunchArgument('map_file',
            default_value=os.path.join(pkg_wheelchair_core, 'maps', 'clear_lab.yaml')),
        DeclareLaunchArgument('nav_params_file',
            default_value=os.path.join(config_dir, 'nav2_params_dual_cloud.yaml')),

        livox_driver,
        robot_state_publisher,
        tf_livox_frame,
        tf_base_footprint,
        tf_odom_camera_init,
        imu_filter,
        fastlio_node,
        relay_cloud_right,
        relay_cloud_left,
        cloud_merger_for_scan,
        pointcloud_to_laserscan,
        map_server,
        amcl,
        lifecycle_manager_localization,
        wheelchair_core,
        nav2_bringup,
        rviz_node,
    ])
