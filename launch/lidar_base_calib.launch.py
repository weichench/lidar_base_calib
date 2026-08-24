import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    default_config = os.path.join(
        get_package_share_directory('lidar_base_calib'),
        'config', 'lidar_base_calib.yaml')

    node = Node(
        package='lidar_base_calib',
        executable='lidar_base_calib_node',
        name='lidar_base_calib',
        output='screen',
        arguments=[config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        node,
    ])
