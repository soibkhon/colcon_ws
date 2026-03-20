from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'wheelchair_core'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.json')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 autonomous wheelchair control with navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheelchair_core_node = wheelchair_core.wheelchair_core_node:main',
            'joystick_controller = wheelchair_core.joystick_controller_node:main',
            'point_cloud_merger = wheelchair_core.point_cloud_merger_node:main',
            'safety_monitor = wheelchair_core.safety_monitor_node:main',
            'position_controller = wheelchair_core.position_controller_node:main',
            'odometry_calibration = wheelchair_core.odometry_calibration_node:main',
            'dual_laser_merger_node = wheelchair_core.dual_laser_merger_node:main',
            'collision_visualizer = wheelchair_core.collision_visualizer_node:main',
            'localization_manager = wheelchair_core.localization_manager:main',
            'localization_manager_2d = wheelchair_core.localization_manager_2d:main',
        ],
    },
)