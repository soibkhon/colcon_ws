#!/usr/bin/env python3
"""
wheelchair_dual_cloud_nav.launch.py

Both MID360s feed Nav2 as separate PointCloud2 observation sources.
AMCL handles localization against a pre-built map using a laser scan
derived from the right LiDAR. No FAST-LIO, no IMU processing.

Physical layout:
  [172 left]   [105 right]          both on same plane, front of wheelchair,
      |               |             on top of left/right front wheels.
  left wheel    right wheel

  MID360_config.json extrinsic (y=±217mm, pitch=5.78°) puts both clouds
  into the shared livox_frame centred between the two sensors.

Topics produced:
  /livox/lidar_192_168_1_105  ─┐
                                ├→ dual_laser_merger → /cloud_merged → /scan  (AMCL)
  /livox/lidar_192_168_1_172  ─┘
  /livox/lidar_192_168_1_105  →  /cloud_right    (Nav2 obstacle layer)
  /livox/lidar_192_168_1_172  →  /cloud_left     (Nav2 obstacle layer)

Nav2 params: nav2_params_dual_cloud.yaml
  obstacle_layer reads cloud_right + cloud_left as PointCloud2 sources.
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
    map_file = LaunchConfiguration('map_file')
    nav_params_file = LaunchConfiguration('nav_params_file')
    start_livox_driver = LaunchConfiguration('start_livox_driver')
    rviz = LaunchConfiguration('rviz')

    # ── Livox driver ──────────────────────────────────────────────────────────
    # multi_topic=1: each LiDAR gets its own topic by IP.
    # The JSON extrinsic (y=±217mm, pitch=5.78°) already places both clouds
    # in the shared livox_frame — no post-processing needed.
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
    # One frame for both LiDARs — JSON extrinsic centres the clouds here.
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

    # ── Relay: publish right cloud on /cloud_right ────────────────────────────
    # Simple topic rename so nav2 params can reference a stable name regardless
    # of the IP-based topic the driver produces.
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

    # ── Laser scan for AMCL ───────────────────────────────────────────────────
    # Merge both clouds into one before converting to a laser scan so AMCL
    # gets a denser 360° scan from both sensors combined.
    # Both clouds are already in livox_frame (JSON extrinsic handles positioning)
    # so the merger just concatenates raw bytes — no transform needed.
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

    # ── Map server ────────────────────────────────────────────────────────────
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

    # ── AMCL ─────────────────────────────────────────────────────────────────
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
    # obstacle_layer in nav2_params_dual_cloud.yaml uses cloud_right + cloud_left
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
        DeclareLaunchArgument('map_file',
            default_value=os.path.join(pkg_wheelchair_core, 'maps', 'clear_lab.yaml')),
        DeclareLaunchArgument('nav_params_file',
            default_value=os.path.join(config_dir, 'nav2_params_dual_cloud.yaml')),

        livox_driver,
        robot_state_publisher,
        tf_livox,
        tf_base_footprint,
        relay_right,
        relay_left,
        cloud_merger_for_scan,
        pointcloud_to_laserscan,
        map_server,
        amcl,
        lifecycle_manager_localization,
        nav2_bringup,
        rviz_node,
    ])
