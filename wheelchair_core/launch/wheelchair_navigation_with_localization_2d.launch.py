import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    cloud_topic = LaunchConfiguration('cloud_topic')
    bd_code = LaunchConfiguration('bd_code')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    default_map = os.path.join(
        get_package_share_directory('wheelchair_core'),
        'maps',
        'clear_lab.yaml'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=default_map,
        description='2D map YAML used for localization'
    )

    declare_cloud_topic_cmd = DeclareLaunchArgument(
        'cloud_topic',
        default_value='livox/lidar',
        description='Pointcloud topic containing merged MID360 data'
    )

    declare_bd_cmd = DeclareLaunchArgument(
        'bd_code',
        default_value='livox0000000001',
        description='Broadcast code of MID360 to activate during localization'
    )

    localization_manager_node = Node(
        package='wheelchair_core',
        executable='localization_manager_2d',
        name='localization_manager_2d',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'map_yaml': map_file},
            {'cloud_topic': cloud_topic},
            {'mid360_bd_code': bd_code},
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_cmd,
        declare_cloud_topic_cmd,
        declare_bd_cmd,
        localization_manager_node,
    ])
