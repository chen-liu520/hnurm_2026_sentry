import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_dir = get_package_share_directory('pointcloud_filter')
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(camera_dir, 'params', 'default.yaml'),
            description='default params file'
        ),
        Node(
            package='pointcloud_filter',
            executable='pointcloud_filter_node',
            output='screen',
            parameters=[params_file]
        )
    ])
