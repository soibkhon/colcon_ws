import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    bd_code = LaunchConfiguration('bd_code')
    frame_id = LaunchConfiguration('frame_id')
    publish_freq = LaunchConfiguration('publish_freq')
    config_path = LaunchConfiguration('config_path')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_bd_code_cmd = DeclareLaunchArgument(
        'bd_code',
        default_value='livox0000000001',
        description='Broadcast code of the MID360 to activate'
    )

    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id',
        default_value='livox_frame',
        description='Frame ID used for the LiDAR pointcloud'
    )

    declare_publish_freq_cmd = DeclareLaunchArgument(
        'publish_freq',
        default_value='10.0',
        description='Publishing frequency for the Livox driver'
    )

    config_dir = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..',
        'config'
    )

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(config_dir, 'MID360_config.json'),
        description='Configuration file for the MID360 driver'
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_single_mid360',
        output='screen',
        parameters=[
            {'xfer_format': 1},
            {'multi_topic': 1},
            {'data_src': 0},
            {'publish_freq': publish_freq},
            {'output_data_type': 0},
            {'frame_id': frame_id},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': config_path},
            {'cmdline_input_bd_code': bd_code},
            {'use_sim_time': use_sim_time},
        ]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_bd_code_cmd,
        declare_frame_id_cmd,
        declare_publish_freq_cmd,
        declare_config_path_cmd,
        livox_driver,
    ])
