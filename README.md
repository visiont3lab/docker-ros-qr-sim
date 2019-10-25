
# QR Detector

## Introduction

 In this report we focused our attention on how to detect and extract QR data. In doing this we focused our attention on the [zbar](http://zbar.sourceforge.net/) library.  In particular we have tested the qr code detector in python using the pyzbar package and on c++ using the zbar library available on the remote (apt install libzbar-dev).

 We are interested in real time application and for this reason we have build a simple docker [ROS](https://www.ros.org/) setup. The latter contains the usb_cam node (to get raw image from a camera) and the QR detector node in charge of detecting the QR and display the result. We have build a result image called "image_result" that is shown automatically using image_view.


## Setup 

### Run python qr detector
It works with both pytho2 and python3. 

```
cd qr_reader_python
pip install pyzbar opencv-python numpy 
python qr_reader_opencv.py
or
python qr_reader_pyzbar.py
```
We have notice that the pyzbar implementation is able to detect the qr at larger distance than the opencv one. 

### Run cpp qr detector
C++ implementation based on [this example](https://www.learnopencv.com/opencv-qr-code-scanner-c-and-python/)
```
cd qr_reader_cpp
mdkir -p build
cd build
cmake ..
make
./run_qr_reader
```

### Run docker ros container (usb_cam + qr_detector)
Setup ROS workspace using docker.  More information are available at [Docker with ROS](https://tuw-cpsg.github.io/tutorials/docker-ros/)
For this example we will use ros melodic that is usually connected to Ubuntu 18.04 LTS. We will also use python2.7 beacause ROS support only this one.
In running the docker container we will mount the volume "qr_reader_ros" that will be our R0S workspace. 

Set X Server host permissions.  More information available [here](http://wiki.ros.org/docker/Tutorials/GUI)

```
xhost +local:docker 
```

Run docker ros docker container with sudo to give device permission [device-docker](https://medium.com/@zwinny/docker-using-webcam-9fafb26cf1e6).

```
sudo docker run \
    -it \
    --name ros_qr_demo  \
    -v $HOME/visiont3lab-github/qr_reader/qr_reader_ros_ws/src:/root/catkin_ws/src/ \
    --env DISPLAY=$DISPLAY \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --network host \
    --env ROS_MASTER_URI=http://localhost:11311 \
    --device=/dev/video0:/dev/video0 \
    ros:melodic-perception-bionic /bin/bash
```

After having started the container we need to install the [usb_cam](http://wiki.ros.org/usb_cam) package that will allow to communicate with a camera.
To do this inside the container we need to do:

```
apt update
apt install -y ros-melodic-usb-cam python-pip libzbar-dev vim
pip install pyzbar

```

Now we only need to compile and run. We assume that you a camera connected to your pc.

1. Compile workspace and start roscore

    ```
    source /opt/ros/melodic/devel/setup.bash
    cd /root/catkin_ws/
    catkin_make
    roscore
    ```

2. Run USB Cam package 
    
    Open a new terminal inside the docker container and run the usb_cam node
    ```
    docker exec -it ros_qr_demo /bin/bash
    source /opt/ros/melodic/devel/setup.bash
    roslaunch usb_cam usb_cam-test.launch
    ```
    The usb_cam-test.launch file is located at "/opt/ros/melodic/share/usb_cam/launch/usb_cam-test.launch" . You can modify it as you like.

3. Run QR Detector package (Open a new terminal (new instance of docker container))
    
    Open a new terminal inside the docker container and run the qr detector 
    Before running this it is required that usb_cam node is started. Wait until you see /usb_cam/image_raw is displayed
    ```
    docker exec -it ros_qr_demo /bin/bash
    source /root/catkin_ws/devel/setup.bash
    roslaunch qr_detecor qr_detector.launch
    ```