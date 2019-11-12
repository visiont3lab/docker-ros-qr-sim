 #!/bin/bash

    xhost +local:docker && \
    docker run -it --rm \
        --name ros_melodic_desktop_full_task_2 \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --env ROS_MASTER_URI=http://localhost:11311 \
        --publish="30000:30000" \
        --runtime=nvidia \
        --device=/dev/snd \
        -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
        -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native:Z \
        --group-add $(getent group audio | cut -d: -f3) \
        -v $ROS_QR_SIM/qr_reader_ros_ws/scripts/config.yaml:/root/.ignition/fuel/config.yaml  \
        -v $ROS_QR_SIM/qr_reader_ros_ws/src:/root/catkin_ws/src/ \
        visiont3lab/ros-melodic-desktop-full:x86 \
        bash 
