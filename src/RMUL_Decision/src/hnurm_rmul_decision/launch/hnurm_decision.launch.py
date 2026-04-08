import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable


def generate_launch_description():
    decision_dir = get_package_share_directory('hnurm_ul_decision')
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
         # 设置环境变量：启用彩色日志输出
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        # 设置环境变量：使用 stdout 输出（确保颜色正常显示）
        # SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(decision_dir, 'param', 'params.yaml'),
            description='default decision yaml'
        ),
        Node(
            package='hnurm_ul_decision',
            executable='decision_node',
            output='screen',
            parameters=[params_file]
        )
        # DeclareLaunchArgument(
        #     'areas_params_file',
        #     default_value=os.path.join(decision_dir, 'params', 'areas.yaml'),
        #     description='special areas'
        # ),
        
    ])