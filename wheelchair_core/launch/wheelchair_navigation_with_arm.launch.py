import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

################### user configure parameters for ros2 start ###################
xfer_format = 0      # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic = 0      # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src = 0         # 0-lidar, others-Invalid data src
publish_freq = 20.0  # frequency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type = 0
frame_id = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():
    pkg_share = FindPackageShare('wheelchair_description').find('wheelchair_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'wheelchair_with_arm_nav.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'wheelchair_display.rviz')

    pkg_wheelchair_core = get_package_share_directory('wheelchair_core')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map_file')
    nav_params_file = LaunchConfiguration('nav_params_file')
    slam_mode = LaunchConfiguration('slam_mode', default='true')
    start_livox_driver = LaunchConfiguration('start_livox_driver', default='true')
    enable_collision_viz = LaunchConfiguration('enable_collision_viz', default='false')

    # Default paths
    default_map_file = os.path.join(pkg_wheelchair_core, 'maps', 'clear_lab.yaml')
    default_nav_params = os.path.join(pkg_wheelchair_core, 'config', 'nav2_params.yaml')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )

    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_file,
        description='Full path to map file for navigation mode'
    )

    declare_nav_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=default_nav_params,
        description='Nav2 parameters file'
    )

    declare_start_livox_driver_cmd = DeclareLaunchArgument(
        'start_livox_driver',
        default_value='true',
        description='Start the embedded Livox driver when true'
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        condition=IfCondition(start_livox_driver)
    )

    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }],
    )

    static_transform_publisher2_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher2',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # CRITICAL: Add transform from base_link to livox_frame
    static_transform_livox_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_livox',
        arguments=[
            '--x', '0.54', '--y', '0', '--z', '0.28',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_transform_base_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_base',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_footprint'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'livox_frame',
            'transform_tolerance': 0.01,
            'min_height': -0.1,
            'max_height': 1.5,
            'angle_min': -2.35619,
            'angle_max': 2.35619,
            'scan_time': 0.33,
            'range_min': 0.1,
            'range_max': 9.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        remappings=[
            ('cloud_in', 'livox/lidar'),
            ('scan', '/scan')
        ]
    )

    map_server_node = Node(
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

    nav2_bringup_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav_params_file,
            'use_lifecycle_mgr': 'true',
            'map_subscribe_transient_local': 'true',
        }.items()
    )

    rviz_config_file = os.path.join(pkg_wheelchair_core, 'config', 'nav_wheelchair.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'planner_server',
                'controller_server',
                'recoveries_server',
                'bt_navigator'
            ]
        }]
    )

    return LaunchDescription([
        declare_start_livox_driver_cmd,
        declare_use_sim_time_cmd,
        declare_map_file_cmd,
        declare_nav_params_file_cmd,
        livox_driver,
        pointcloud_to_laserscan_node,
        robot_state_publisher_node,
        static_transform_publisher2_cmd,
        static_transform_livox_cmd,
        static_transform_base_to_base,
        map_server_node,
        nav2_bringup_group,
        rviz_node,
        lifecycle_manager,
    ])
