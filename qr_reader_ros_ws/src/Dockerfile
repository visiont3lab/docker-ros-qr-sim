FROM osrf/ros:melodic-desktop-full
            # nvidia-container-runtime
            ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
            ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

            # docker build -t ros-melodic-desktop-full .

            RUN \
            apt-get update && \
            apt-get -y install python-pip libzbar-dev vim apt-utils psmisc screen && \
            pip install pyzbar  && \
            rm -rf /var/lib/apt/lists/* 
