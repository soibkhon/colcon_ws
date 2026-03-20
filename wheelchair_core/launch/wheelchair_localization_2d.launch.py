import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    bd_code = LaunchConfiguration('bd_code')
    cloud_topic = LaunchConfiguration('cloud_topic')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    maps_dir = os.path.join(get_package_share_directory('wheelchair_core'), 'maps')
    declare_map_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(maps_dir, 'clear_lab.yaml'),
        description='2D occupancy grid YAML used for localization'
    )

    declare_bd_cmd = DeclareLaunchArgument(
        'bd_code',
        default_value='livox0000000001',
        description='Broadcast code of MID360 used for localization'
    )

    declare_cloud_topic_cmd = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/livox/lidar_192_168_1_172',
        description='Pointcloud topic from the single MID360'
    )

    single_lidar_launch = PathJoinSubstitution([
        get_package_share_directory('wheelchair_core'),
        'launch',
        'wheelchair_single_lidar.launch.py'
    ])

    single_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_lidar_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'bd_code': bd_code,
            'frame_id': 'livox_frame',
            'publish_freq': '10.0'
        }.items()
    )

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.05,
            'min_height': -0.3,
            'max_height': 1.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 15.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('cloud_in', cloud_topic),
            ('scan', '/scan')
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([
                    get_package_share_directory('wheelchair_description'),
                    'urdf',
                    'wheelchair.xacro'
                ])
            ]),
            'use_sim_time': use_sim_time
        }]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'base_link',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'map_file_name': map_file,
            'mode': 'localization',
            'transform_publish_period': 0.02,
            'map_update_interval': 2.0,
            'resolution': 0.05,
            'max_laser_range': 10.0,
            'minimum_laser_range': 0.2,
            'minimum_time_interval': 0.1,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,
            'scan_buffer_size': 5,
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_travel_distance': 0.05,
            'minimum_travel_heading': 0.05,
            'do_loop_closing': False
        }]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_cmd,
        declare_bd_cmd,
        declare_cloud_topic_cmd,
        single_lidar,
        pointcloud_to_scan,
        robot_state_publisher,
        slam_toolbox_node,
    ])
