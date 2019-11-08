#!/bin/bash 

    xhost +local:docker  && \
    docker run -it --rm \
        --name ros_melodic_desktop_full_task_1 \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --env ROS_MASTER_URI=http://localhost:11311 \
        --volume="$ROS_QR_SIM/qr_reader_ros_ws/src:/root/catkin_ws/src/" \
        --device="/dev/video0:/dev/video0" \
        --publish="30000:30000" \
        ros-melodic-desktop-full \
        bash 
