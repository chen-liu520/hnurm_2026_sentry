import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression,PathJoinSubstitution,ThisLaunchFileDir
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  return LaunchDescription([

    # 点云地面分割
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(
          get_package_share_directory('linefit_ground_segmentation_ros'),
          'launch', 'segmentation.launch.py'
        )
      )
    ),

    # 点云转激光雷达
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(
          get_package_share_directory('pointcloud_to_laserscan'),
          'launch', 'pointcloud_to_laserscan_launch.py'
        )
      )
    ),

    # 点云滤波
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(
          get_package_share_directory('pointcloud_filter'),
          'launch', 'hnurm_pointcloud_filter.launch.py'
        )
      )
    ),

  ])
