import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the launch directory
    pkg_dir = get_package_share_directory('wheelchair_core')  # Change this to your package name
    
    # Create the launch configuration variables
    map_file_name = LaunchConfiguration('map_file_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file_name',
        default_value='/home/wheelchair_pc/saved_map/clear_lab.yaml',
        description='Full path to map file to load')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Create the slam_toolbox localization node
    start_localization_slam_toolbox_node = Node(
        parameters=[
            {
                'slam_toolbox': {
                    'odom_frame': 'odom',
                    'map_frame': 'map', 
                    'base_frame': 'base_footprint',
                    'scan_topic': '/scan',
                    'mode': 'localization',
                    'map_file_name': map_file_name,
                    'use_sim_time': use_sim_time,
                    
                    # LiDAR specific parameters
                    'max_laser_range': 6.0,  # Match your LiDAR specs
                    'minimum_laser_range': 0.2,  # Match your LiDAR specs
                    'resolution': 0.05,
                    'throttle_scans': 1,
                    'transform_publish_period': 0.02,
                    'map_update_interval': 2.0,
                    'minimum_time_interval': 0.1,  # Reduced to handle faster data
                    'transform_timeout': 0.2,
                    'tf_buffer_duration': 30.0,
                    'stack_size_to_use': 40000000,
                    'enable_interactive_mode': True,
                    
                    # Performance tuning
                    'scan_buffer_size': 5,  # Reduced buffer size
                    'scan_buffer_maximum_scan_distance': 10.0,
                    'use_scan_matching': True,
                    'use_scan_barycenter': True,
                    'minimum_travel_distance': 0.3,
                    'minimum_travel_heading': 0.3,
                    
                    # Loop closure (disabled for localization performance)
                    'do_loop_closing': False,
                    
                    # Correlation parameters
                    'correlation_search_space_dimension': 0.3,
                    'correlation_search_space_resolution': 0.01,
                    'correlation_search_space_smear_deviation': 0.1,
                    
                    # Scan matcher parameters
                    'distance_variance_penalty': 0.3,
                    'angle_variance_penalty': 0.8,
                    'fine_search_angle_offset': 0.00349,
                    'coarse_search_angle_offset': 0.349,
                    'coarse_angle_resolution': 0.0349,
                    'minimum_angle_penalty': 0.9,
                    'minimum_distance_penalty': 0.5,
                    'use_response_expansion': True
                }
            }
        ],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to the launch description
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_localization_slam_toolbox_node)

    return ld