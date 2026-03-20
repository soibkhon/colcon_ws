import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

################### user configure parameters ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 1    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar
publish_freq  = 10.0 # publishing frequency
output_type   = 0
lvx_file_path = '/home/livox/livox_test.lvx'
frame_id      = 'livox_frame'
# unique bd codes for your two livox devices
cmdline_bd_code_left  = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### end user configure parameters ###################

def generate_launch_description():
    pkg_share = FindPackageShare('wheelchair_description').find('wheelchair_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'wheelchair.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'wheelchair_display.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }],
    )

    # Livox driver node for left lidar
    livox_left_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_publisher',
        output='screen',
        parameters=[{
            "xfer_format": xfer_format,
            "multi_topic": multi_topic,
            "data_src": data_src,
            "publish_freq": publish_freq,
            "output_data_type": output_type,
            "frame_id": frame_id,
            "lvx_file_path": lvx_file_path,
            "user_config_path": user_config_path,
            "cmdline_input_bd_code": cmdline_bd_code_left
        }]
    )

    static_transform_publisher2_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher2',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
    )

    # CRITICAL: Add transform from base_link to livox_frame
    static_transform_livox_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_livox',
        arguments=[
            '--x', '0.2', '--y', '0', '--z', '0.3',  # Adjust these values to match your LiDAR position
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
    )


    pointcloud_to_laserscan_node = Node(
          package='pointcloud_to_laserscan',
          executable='pointcloud_to_laserscan_node',
          name='pointcloud_to_laserscan',
          parameters=[{
               'target_frame': 'livox_frame',
               'transform_tolerance': 0.1,  # Increased tolerance
               'min_height': -0.5,          # Lower minimum height
               'max_height': 2.0,           # Higher maximum height
               'angle_min': -3.14159,       # Full 360 degrees
               'angle_max': 3.14159,        # Full 360 degrees
               'angle_increment': 0.0175,   # 1 degree increments
               'scan_time': 0.1,
               'range_min': 0.1,
               'range_max': 10.0,
               'use_inf': True,
               'inf_epsilon': 1.0,
               'use_sim_time': use_sim_time
           }],
           remappings=[
               ('cloud_in', '/livox/lidar_192_168_1_172'),   # Note the leading slash
               ('scan', '/scan')
           ]
       )

    

    # RViz for visualization
    livox_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        livox_left_driver,
        static_transform_publisher2_cmd,
        static_transform_livox_cmd,  # ADD THIS LINE - This is the missing transform!
        pointcloud_to_laserscan_node,
        livox_rviz
    ])