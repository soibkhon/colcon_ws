import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_pcd_path = LaunchConfiguration('map_pcd_path')
    single_lidar_bd_code = LaunchConfiguration('single_lidar_bd_code')
    single_lidar_topic = LaunchConfiguration('single_lidar_topic')
    single_imu_topic = LaunchConfiguration('single_imu_topic')
    single_lidar_bd_code = LaunchConfiguration('single_lidar_bd_code')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    default_pcd = os.path.join(
        os.path.expanduser('~'),
        'colcon_ws',
        'src',
        'FAST_LIO',
        'PCD',
        'clear.pcd'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map_pcd_path',
        default_value=default_pcd,
        description='Absolute path to the FAST-LIO-generated PCD map'
    )

    declare_bd_cmd = DeclareLaunchArgument(
        'single_lidar_bd_code',
        default_value='livox0000000001',
        description='Broadcast code for the MID360 used during localization'
    )

    declare_lidar_topic_cmd = DeclareLaunchArgument(
        'single_lidar_topic',
        default_value='livox/lidar_192_168_1_172',
        description='Pointcloud topic to use during FAST-LIO localization'
    )

    declare_imu_topic_cmd = DeclareLaunchArgument(
        'single_imu_topic',
        default_value='livox/imu_192_168_1_172',
        description='IMU topic to use during FAST-LIO localization'
    )

    localization_manager_node = Node(
        package='wheelchair_core',
        executable='localization_manager',
        name='localization_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'map_pcd_path': map_pcd_path},
            {'single_lidar_bd_code': single_lidar_bd_code},
            {'single_lidar_topic': single_lidar_topic},
            {'single_imu_topic': single_imu_topic},
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_cmd,
        declare_bd_cmd,
        declare_lidar_topic_cmd,
        declare_imu_topic_cmd,
        localization_manager_node,
    ])
