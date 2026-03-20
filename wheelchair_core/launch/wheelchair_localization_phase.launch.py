import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('pcd_map_path')
    bd_code = LaunchConfiguration('bd_code')
    lidar_topic = LaunchConfiguration('lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'pcd_map_path',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'colcon_ws',
            'src',
            'FAST_LIO',
            'PCD',
            'clear.pcd'
        ),
        description='PCD map created by FAST-LIO for localization'
    )

    declare_bd_cmd = DeclareLaunchArgument(
        'bd_code',
        default_value='livox0000000001',
        description='Broadcast code for the single MID360 used during localization'
    )

    declare_lidar_topic_cmd = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/livox/lidar_192_168_1_172',
        description='Pointcloud topic to use for FAST-LIO localization'
    )

    declare_imu_topic_cmd = DeclareLaunchArgument(
        'imu_topic',
        default_value='/livox/imu_192_168_1_172',
        description='IMU topic to use for FAST-LIO localization'
    )

    pkg_wheelchair_core = get_package_share_directory('wheelchair_core')
    single_lidar_launch = PathJoinSubstitution([
        pkg_wheelchair_core,
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

    pkg_wheelchair_description = get_package_share_directory('wheelchair_description')
    urdf_file = os.path.join(pkg_wheelchair_description, 'urdf', 'wheelchair.xacro')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time,
        }]
    )

    static_transform_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_livox',
        arguments=['0.54', '0', '0.28', '0', '0', '0', 'base_link', 'livox_frame'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    fastlio_config = os.path.join(
        get_package_share_directory('fast_lio'),
        'config',
        'mid360.yaml'
    )

    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_localization',
        output='screen',
        parameters=[fastlio_config, {
            'use_sim_time': use_sim_time,
            'map_file_path': map_file,
            'feature_extract_enable': False,
            'common.lid_topic': lidar_topic,
            'common.imu_topic': imu_topic,
            'mapping.localization_mode': True
        }],
        remappings=[
            ('/cloud_registered', '/cloud_registered_fastlio'),
            ('/Odometry', '/fastlio_odom'),
        ]
    )

    map_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_cmd,
        declare_bd_cmd,
        declare_lidar_topic_cmd,
        declare_imu_topic_cmd,
        single_lidar,
        robot_state_publisher,
        static_transform_livox,
        map_to_camera_tf,
        fastlio_node,
    ])
