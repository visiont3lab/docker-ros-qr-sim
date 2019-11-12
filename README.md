# QR Detector and pose estimation with respect to a camera

## Introduction

In this repository we are going to focus our attention on:

1. Case 1) QR detection and decoding using both python and c++. In doing this we focused our attention on the [zbar](http://zbar.sourceforge.net/) library.  In particular we have tested the qr code detector in python using the pyzbar package and on c++ using the zbar library available on the remote (apt install libzbar-dev).
2. Case 2) Moving QR pose (position and orientation) estimation with respect to a fixed camera  in both real and simulated world. To do this we have developed a small ROS workspace. We have chosen to run ROS  inside a docker container. In particular, we have developed two different scenarios:
    1.  Task 1 (Real Environment): Given an USB camera we are going to estimate the QR code position with respect to the USB camera.
    2.  Task 2 (Simulated Environment): We have simulated a small warehouse where there are some moving QR code and two fixed camera. The objective is to estimate the position of the QR code with respect to the camera. 

We also provide a detailed tutorial on how to setup docker with ROS,GAZEBO, RVIZ and RQT.  [MORE INFORMATION HERE](qr_reader_ros_ws/src/instructions.md)


## Requirements

* [Install docker](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-18-04)
* [Install nvidia-docker2](https://github.com/NVIDIA/nvidia-docker)

## Setup

```
cd $HOME && \
sudo apt install xfce4-terminal && \
git clone https://github.com/visiont3lab/docker-ros-qr-sim.git && \
echo "export ROS_QR_SIM=$HOME/docker-ros-qr-sim" >> $HOME/.bashrc && source $HOME/.bashrc
```

## Case 1) Testing QR detection and decoding

### Run python qr detector
It works with both python2 and python3. 

```
cd $ROS_QR_SIM/qr_reader_python && \
pip install pyzbar opencv-python numpy && \ 
python qr_reader_opencv.py
or
python qr_reader_pyzbar.py
```
We have notice that the pyzbar implementation is able to detect the qr at larger distance than the opencv one. 

### Run cpp qr detector
C++ implementation based on [this example](https://www.learnopencv.com/opencv-qr-code-scanner-c-and-python/)
It is required to have opencv installed on your pc. More information available [here](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)

```
cd $ROS_QR_SIM/qr_reader_cpp && \
mdkir -p build && \
cd build && \
cmake .. && \
make && \
./run_qr_reader
```

## Case 2) Implementing  QR code pose estimation with respect to a camera in both real and simulated world

Here we summarize the main step required to run both task1 and task2. However we have written a step by step tutorial on how to use docker together with gazebo,rviz,rqt and apply the constructed container to our example. 

###  Task 1 (Real Environment)

Launch task1

```
cd $ROS_QR_SIM/qr_reader_ros_ws/scripts &&  \
./launch_full_task1.sh
```

It will take around 20 sec to start cause we are also compiling the added ros packages inside the docker container.

Go to http://0.0.0.0:30000 to see the results (click on /output))

###  Task 2 (Simulated Environment)

Extra Requirements: For this example it is required to have:

* a pc with a linux os system installed (ex Ubuntu 18.04 LTS), 
* nvidia recent drivers installed on your machine. We will use nvidia driver to run gazebo inside the docker container.
* if you are interested in support also audio inside gazebo simulation we also need ALSA drivers

Launch task2

```
cd $ROS_QR_SIM/qr_reader_ros_ws/scripts &&  \
./launch_full_task2.sh
```

Go to http://0.0.0.0:30000 to see the results (click on /output_high and /output_low)
