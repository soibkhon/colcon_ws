import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_rear = LaunchConfiguration('enable_rear')
    front_bd_code = LaunchConfiguration('front_bd_code')
    rear_bd_code = LaunchConfiguration('rear_bd_code')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_enable_rear_cmd = DeclareLaunchArgument(
        'enable_rear',
        default_value='true',
        description='Enable the rear MID360 before merging'
    )

    declare_front_bd_cmd = DeclareLaunchArgument(
        'front_bd_code',
        default_value='livox0000000001',
        description='Broadcast code of the front MID360'
    )

    declare_rear_bd_cmd = DeclareLaunchArgument(
        'rear_bd_code',
        default_value='livox0000000002',
        description='Broadcast code of the rear MID360'
    )

    config_dir = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..',
        'config'
    )
    user_config_path = os.path.join(config_dir, 'MID360_config.json')

    front_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        namespace='front',
        name='livox_front',
        output='screen',
        parameters=[
            {'xfer_format': 0},
            {'multi_topic': 1},
            {'data_src': 0},
            {'publish_freq': 20.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_front_frame'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': user_config_path},
            {'cmdline_input_bd_code': front_bd_code},
            {'use_sim_time': use_sim_time},
        ]
    )

    rear_driver = Node(
        condition=IfCondition(enable_rear),
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        namespace='rear',
        name='livox_rear',
        output='screen',
        parameters=[
            {'xfer_format': 0},
            {'multi_topic': 1},
            {'data_src': 0},
            {'publish_freq': 20.0},
            {'output_data_type': 0},
            {'frame_id': 'livox_rear_frame'},
            {'lvx_file_path': '/home/livox/livox_test.lvx'},
            {'user_config_path': user_config_path},
            {'cmdline_input_bd_code': rear_bd_code},
            {'use_sim_time': use_sim_time},
        ]
    )

    dual_merger = Node(
        package='wheelchair_core',
        executable='dual_laser_merger_node',
        name='dual_laser_merger',
        output='screen',
        parameters=[{'output_topic': '/livox/lidar'}],
        remappings=[
            ('input_cloud1', '/front/livox/lidar'),
            ('input_cloud2', '/rear/livox/lidar'),
        ]
    )

    front_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_front_lidar',
        arguments=[
            '--x', '0.54', '--y', '0.0', '--z', '0.28',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_front_frame'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rear_tf = Node(
        condition=IfCondition(enable_rear),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_rear_lidar',
        arguments=[
            '--x', '-0.54', '--y', '0.0', '--z', '0.28',
            '--roll', '0', '--pitch', '0', '--yaw', '3.14159',
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_rear_frame'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_fused_lidar',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'livox_front_frame',
            '--child-frame-id', 'livox_frame'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_enable_rear_cmd,
        declare_front_bd_cmd,
        declare_rear_bd_cmd,
        front_driver,
        rear_driver,
        dual_merger,
        front_tf,
        rear_tf,
        alias_tf,
    ])
