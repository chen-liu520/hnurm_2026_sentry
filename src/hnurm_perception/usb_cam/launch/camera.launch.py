import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

usb_cam_dir = get_package_share_directory('usb_cam')
detect_dir = get_package_share_directory('hnurm_detect')
detect_model_path = os.path.join(detect_dir, 'model')
model_path = os.path.join(detect_model_path, "mlp.onnx")
label_path = os.path.join(detect_model_path, "label.txt")

additional_param = {
    'model_path': model_path,
    'label_path': label_path,
}

def generate_launch_description():
    usb_params_file = LaunchConfiguration('usb_params_file')
    detect_params_file = LaunchConfiguration('detect_params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'usb_params_file',
            default_value=os.path.join(usb_cam_dir, 'config', 'params_1.yaml'),
            description='usb camera params file'
        ),
        DeclareLaunchArgument(
            'detect_params_file',
            default_value=os.path.join(detect_dir, 'params', 'usb_camera.yaml'),
            description='detect params file'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            namespace='back_camera',
            parameters=[usb_params_file]
        ),
        Node(
            package='hnurm_detect',
            executable='hnurm_detect_node',
            output='screen',
            namespace='back_camera',
            parameters=[detect_params_file, additional_param]
        ),
    ])

