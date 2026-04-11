#!/bin/bash
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
    "ros2 launch hnurm_uart hnurm_uart.launch.py"
    #navigation
    "echo ' '|sudo -S docker run --rm --net host --env DISPLAY=\$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix --volume /dev:/dev --privileged livo2:latest bash -c 'source ~/.bashrc && roslaunch fast_livo maping_mid360.launch rviz:=0 img_en:=0'"
    

    "ros2 launch hnurm_bringup tf_transformer.launch.py"

    "ros2 launch hnurm_bringup test.launch.py"
    #relocaliztion
    # "ros2 launch registration registration.launch.py"
    # "rviz2"
    

    #"ros2 run gimbal_rotation_server gesture_printer"
    
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

#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

while [ 1 ]; do
    tcount=$(ps -ef | grep decision_node | grep -v grep | wc -l)

    if [ "${tcount}" -ge 1 ]; then
        echo "[daemon]: Decision is running"
        sleep 1
    else
        gnome-terminal -- bash -c "\
        cd ${SCRIPT_DIR} && \
        source /home/rm/decision_test/install/setup.bash && \
        ros2 run hnurm_small_decision decision_node --ros-args --params-file ${SCRIPT_DIR}/src/hnurm_small_decision/param/params.yaml; \
        exec bash"
        echo "[daemon]: re-running"
        sleep 2
    fi
done
