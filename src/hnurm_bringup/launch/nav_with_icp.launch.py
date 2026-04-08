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
    # IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('hnurm_navigation'),'launch','bringup_launch.py'))
    # ),
    # Node(
    #       package='hnurm_sendcmd',
    #       executable='hnurm_sendcmd',  
    #       name='hnurm_sendcmd',
    #       output='screen', 
    #     ),

    # 静态变换
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(
          get_package_share_directory('hnurm_bringup'),
          'launch',
          'static_transform.launch.py'
        )
      )
    ),

    # # 启动串口节点
    # IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(
    #     os.path.join(
    #       get_package_share_directory('hnurm_uart'),
    #       'launch',
    #       'hnurm_uart.launch.py'
    #     )
    #   )
    # ),

    # # 激光雷达驱动
    # IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(
    #     os.path.join(
    #       get_package_share_directory('livox_ros_driver2'),
    #       'launch',
    #       'msg_MID360_launch.py'
    #     )
    #   )
    # ),

    # # IMU互补滤波
    # IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(
    #     os.path.join(
    #       get_package_share_directory('imu_complementary_filter'),
    #       'launch',
    #       'complementary_filter.launch.py'
    #     )
    #   )
    # ),

    # # 点云分割
    # IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(
    #     os.path.join(
    #       get_package_share_directory('linefit_ground_segmentation_ros'),
    #       'launch',
    #       'segmentation.launch.py'
    #     )
    #   )
    # ),

    # # 点云建图-fast_lio
    # IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(
    #     os.path.join(
    #       get_package_share_directory('fast_lio'),
    #       'launch',
    #       'mapping.launch.py'
    #     )
    #   )
    # ),

    # # 点云建图-point_lio
    # IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(
    #     os.path.join(
    #       get_package_share_directory('point_lio'),
    #       'launch',
    #       'mapping_mid360.launch.py'
    #     )
    #   )
    # ),

    # 点云降采样
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(
          get_package_share_directory('pointcloud_downsampling'),
          'launch',
          'pointcloud_downsampling.launch.py'
        )
      )
    ),

    # # ICP配准
    # IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(
    #     os.path.join(
    #       get_package_share_directory('icp_registration'),
    #       'launch',
    #       'icp.launch.py'
    #     )
    #   )
    # ),

    Node(
      package='robot_localization',
      executable='ekf_node',
      name='ekf_filter_node_odom',
      output='screen',
      parameters=[os.path.join(get_package_share_directory('hnurm_bringup'), 'params', 'rl.yaml')],    
      #remappings=[('/odometry/filtered', '/local_odom')]
    ), 
    Node(  #去车体
          package='hnurm_pointcloud',
          executable='hnurm_pointcloud_node',
          name='hnurm_pointcloud_node',
          output='screen',
        ),



    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('pointcloud_to_laserscan'),'launch','pointcloud_to_laserscan_launch.py'))
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

#保存地图
#ros2 run nav2_map_server map_saver_cli -f /home/rm/slam/ws/src/hnu-vision-ros/hnurm_navigation/map/RMUC
#ros2 service call /map_save std_srvs/srv/Trigger "{}"
