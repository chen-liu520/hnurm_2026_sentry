#!/bin/bash


# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"

echo "[1/5] 启动串口..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py; exec bash"

sleep 5

echo "[2/2] 启动 FAST_LIVO2 里程计..."
gnome-terminal -- bash -c "source $WORKSPACE_DIR/install/setup.bash && ros2 launch fast_livo mapping_mid360.launch.py; exec bash"
