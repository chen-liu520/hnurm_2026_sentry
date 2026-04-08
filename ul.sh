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
sleep 1.0

cmds=( 

    "ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics"
    
    # "echo ' '|sudo -S chmod 777 /dev/ttyUSB0"
    # "ros2 launch ldlidar_stl_ros2 ld06.launch.py"
    #后视相机
    #"ros2 launch usb_cam camera.launch.py"  
    #vision
    #"ros2 launch hnurm_camera hnurm_camera.launch.py"
    #"ros2 launch hnurm_detect hnurm_detect.launch.py"
    #"ros2 launch hnurm_armor hnurm_armor.launch.py"
    #uart serial
    #"ros2 launch hnurm_uart hnurm_uart.launch.py"
    # 自瞄（包含串口）
    "ros2 launch hnurm_bringup bringup.launch.py"
    #navigation
    "echo ' '|sudo -S docker run --rm --net host --env DISPLAY=\$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix --volume /dev:/dev --privileged livo2:latest bash -c 'source ~/.bashrc && roslaunch fast_livo maping_mid360.launch rviz:=0 img_en:=0'"
    #'ros2 launch livox_ros_driver2 msg_MID360_launch.py'

    #'ros2 launch fast_livo mapping_mid360.launch.py'
    
    

    "ros2 launch hnurm_bringup tf_transformer.launch.py"

    "ros2 launch hnurm_bringup test.launch.py"
    #relocaliztion
    # "ros2 launch registration registration.launch.py"
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
    sleep 1.5
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
