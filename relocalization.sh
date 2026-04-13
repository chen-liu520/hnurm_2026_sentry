#!/bin/bash
cmd_ip=(
    "echo ' '|sudo -S ip link set enp87s0 down"
    "echo ' '|sudo -S ip link set enp87s0 up"
    
)
for cmd in "${cmd_ip[@]}"
do
    echo Current CMD : "$cmd"
    gnome-terminal -- bash -c "$cmd;"
    sleep 5.0
done
sleep 5.0

cmds=( 

    #"ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics" 

    #自瞄（包含串口）
    "ros2 launch hnurm_bringup bringup.launch.py"

    #fast_livo2
    "ros2 launch livox_ros_driver2 msg_MID360_launch.py"

    'ros2 launch fast_livo odometry_mode.launch.py'
    
    
    # TF 转换
    "ros2 launch hnurm_bringup tf_transformer.launch.py"

    # 点云数据流
    "ros2 launch hnurm_bringup test.launch.py"






    #relocaliztion
    "ros2 launch registration registration_rviz.launch.py"
    # "rviz2"
    
    #"ros2 run gimbal_rotation_server referee"

    #"ros2 launch decision_gesture_test params_visualizer.launch.py"

    #"ros2 run gimbal_rotation_server gesture_printer"

    "ros2 run hnurm_referee_sim referee_sim_node"
    
)
for cmd in "${cmds[@]}"
do
    echo Current CMD : "$cmd"
    gnome-terminal -- bash -c "cd $(pwd);source ~/ros-humble-ros1-bridge/install/local_setup.bash;source /home/rm/decision_test/install/setup.bash;$cmd;exec bash;"
    sleep 4.0
done
sleep 3.0


cmds_nav=( 
    "ros2 launch hnurm_navigation bringup_launch.py"
)
for idx in "${cmds_nav[@]}"
do
    echo Current CMD : "$idx"
    gnome-terminal -- bash -c "cd $(pwd);source /home/rm/decision_test/install/setup.bash;$idx;exec bash;"
    sleep 4.0
done

# 启动 decision_gesture_test
echo "[gesture.sh]: Starting decision_gesture_test..."
gnome-terminal -- bash -c "cd $(pwd);source /home/rm/decision_test/install/setup.bash;ros2 launch hnurm_ul_decision hnurm_decision.launch.py;exec bash;"