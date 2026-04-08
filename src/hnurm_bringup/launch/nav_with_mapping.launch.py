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
  rviz_config_file = '/home/rm/slam/ws/rviz.rviz'
  return LaunchDescription([
    # Node(
    #       package='hnurm_sendcmd',
    #       executable='hnurm_sendcmd',  
    #       name='hnurm_sendcmd',
    #       output='screen', 
    #     ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(
          get_package_share_directory('hnurm_navigation'),
          'launch',
          'bringup_no_amcl_launch.py'
        )
      )
    ),

   
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(
          get_package_share_directory('linefit_ground_segmentation_ros'),
          'launch',
          'segmentation.launch.py'
        )
      )
    ),

    #  Node(  #去车体
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(
          get_package_share_directory('pointcloud_to_laserscan'),
          'launch',
          'pointcloud_to_laserscan_launch.py'
        )
      )
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(
          get_package_share_directory('hnurm_navigation'),
          'launch',
          'online_async_launch.py'
        )
      )
    ),

    # 声明rviz_config_file参数
    DeclareLaunchArgument(
        'rviz_config_file',
        default_value=rviz_config_file,
        description='Full path to the RVIZ config file to use'
    ),

    # # 启动rviz2节点
    # Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', LaunchConfiguration('rviz_config_file')],
    # ),
  ])
