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

## Let's use docker ROS container!

Requirements: for this example it is required to have:

* a pc with a linux os system installed (ex Ubuntu 18.04 LTS), 
* nvidia recent drivers installed on your machine
* if you are interested in support also audio inside gazebo simulation we also need ALSA drivers

We will use the power of docker to create a docker ros melodic container able to:

    1. Perform a qr detection given an input image coming from the webcam
    2. Simulate a small warehouse in which there a some QR code moving with a speed of 1.7m/s and fixed cameras at different height. The object is to detect and decode the moving QR codes. For this example we will use the ros gazebo simulation tool.

To achieve both the goals we will use the following ros packages contained inside the folder qr_reader_ros_ws:

* qr_detector_pkg : this is the core of our application. We have build this package and it is object it is to detect qr code and estimate their position with respect to a fixed camera.

* web_video_server and async_web_server_cpp: we have used these two packages to display on the browser the cameras' views.  Cameras' views are available at http://localhost:3000 (we have customized the port). Furtheremore it is also possible to display the camera content as video (vp9.vp8.h264). This package use ffmped to convert a series of images into a video. More information on this package are avbailable [here](http://wiki.ros.org/web_video_server).

* gazebo_models_pkg (only for simulation): This folder contains everything we need to run our custom gazebo simulation. In particular, inside the foldder worlds there is the qr_world.world file and inside models we will find instead all the models that we will use for the simulation. It is important to point out that some models require plugin that has to be build separately. Therefore before to build our workspace we need to build the build the plugins as following:
  
    ```    
    cd gazebo_models_pkg
    mkdir -p build && cd build
    cmake .. && make 
    # To modify the plugin you can modify the content of the src folder
    ```

* usb_cam (only for real time): This ros package will be used to retrieve usb camera images. We will use this package in the task number 1, where given a usb connected camera we want to detect the qr code and estimate it is posizione with respect to the camera. More information available [here](http://wiki.ros.org/usb_cam)

### Tutorial on how to setup a ROS docker container able to use graphics ( gazebo, rviz, rqt)
We will use the package [ros-melodic-desktop-full](http://wiki.ros.org/melodic/Installation/Ubuntu) that is the one associated to Ubuntu Bionic 18.04 LTS.
We assume that you have a computer with nvidia driver installed on it (preferebly a linux operating system). We are going to build a docker container able to run, using X11 server,  graphic tools 
such as gazebo, rviz, rtq or simply display real time windows using the opencv gui. We are going to deal with gazebo that required nvidia driver to work and for this reason, for the sake of this example, we will
focus our attention on how to setup a docker ROS framework able to also use nvidia graphics drivers.

We will use this guide as a [reference](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#Using_nvidia-docker)
It is important to point out that to be able to run gazebo, rqt and rviz inside a docker container it is required to enable hardware accelaration and the associated graphic drivers.
To do this, first of all we need to install nvidia-driver2, there is also the possibility to use nvidia-driver1 but we are going to use the most recent version.

1. Install [nvidia-driver2](https://github.com/NVIDIA/nvidia-docker). This will allow the docker container to use nvidia-driver. It is required the the host machine has a nvidia graphic card and some driver installed on it.
We do this on the host machine.

    ```
    # Add the package repositories
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
    curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

    sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
    sudo systemctl restart docker
    ```

3. Build the ROS docker image.
   
    * Create a Dockerfile and add nvidia requirementems for graphics

        ```
        echo \
            'FROM osrf/ros:melodic-desktop-full
            # nvidia-container-runtime
            ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
            ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

            # docker build -t ros-melodic-desktop-full .

            RUN \
            apt-get update && \
            apt-get -y install python-pip libzbar-dev vim apt-utils psmisc && \
            pip install pyzbar  && \
            rm -rf /var/lib/apt/lists/* ' \
            > Dockerfile
        ```

    * Build the docker image
        This image will contain all the packages associated to the ros-melodic-desktop-full package.
        ```
        docker build -t ros-melodic-desktop-full .
        ```
4. Create a docker container using the previous build image and start gazebo, rqt and rviz inside it.
   
    * Create a launch file and give xhost permission to use X11 server.

        ```
        echo \
            '#!/bin/bash 
            xhost +local:docker  && \
            docker run -it --rm \
                --name ros_melodic_desktop_full \
                --env="DISPLAY=$DISPLAY" \
                --env="QT_X11_NO_MITSHM=1" \
                --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                --env ROS_MASTER_URI=http://localhost:11311 \
                --runtime=nvidia \
                --device=/dev/snd \
                -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native \
                -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native:Z \
                --group-add $(getent group audio | cut -d: -f3) \
                -v $HOME/visiont3lab-github/qr_reader/qr_reader_ros_ws/src/config.yaml:/root/.ignition/fuel/config.yaml  \
                ros-melodic-desktop-full \
                bash' \
        > start_docker_test.sh
        ```

    * Make the launch file executable
  
        ```
        chmod a+x  start_docker_test.sh
        ```

    * Launch the file
  
        ```
        ./start_docker_test.sh 
        ```

        Note that we have add to the --rm to the docker run command line. This means that when you kill your container it will disappear.

5. Test if the installation is successful. Run  rviz, gazebo and rqt_plot

    Open 4 terminals and type the following commands
    ```
    # Run roscore
    docker exec -it ros_melodic_desktop_full /bin/bash -c '\
    cd source /opt/ros/melodic/setup.bash && roscore'

    # Run gazebo simulation world
    docker exec -it ros_melodic_desktop_full /bin/bash -c  \
    'source /opt/ros/melodic/setup.bash && roslaunch gazebo_ros empty_world.launch'

    # Run rviz
    docker exec -it ros_melodic_desktop_full /bin/bash -c \
    'source /opt/ros/melodic/setup.bash && rosrun rviz rviz'
    
    # Run rqt_plot
    docker exec -it ros_melodic_desktop_full /bin/bash -c \
    'source /opt/ros/melodic/setup.bash && rosrun rqt_plot rqt_plot'

    # Extra: To Kill gazebo 
    docker exec -it ros_melodic_desktop_full /bin/bash -c  \
    'pkill -9 gazebo; pkill -9 gzserver; killall -9 gzclient;'
    ```

### Task 1: Estimate moving QR codes position and orientation (pose) with respect to a moving or fixed usb camera 
To solve this task we will need to modify the previous create launch_test.sh file to also allow the docker container to use devices. For the sake of this task we will reate a launch_task1.sh file. In this example it is not required to use nvidia-drivers. For this reason we will remove --runtime=nvidia. This will allow to run this example on machine we nvidia driver ar not available. If you have nvidia-driver installed feel free to keep the command. Please also ensure that the camera is device0 otherwise change the bottom script with the right device number. For the sake of this example only one camera will be used.

```
echo \
    '#!/bin/bash 

    xhost +local:docker  && \
    docker run -it --rm \
        --name ros_melodic_desktop_full_task_1 \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --env ROS_MASTER_URI=http://localhost:11311 \
        --volume="$HOME/visiont3lab-github/qr_reader/qr_reader_ros_ws/src:/root/catkin_ws/src/" \
        --device="/dev/video0:/dev/video0" \
        --publish="30000:30000" \
        ros-melodic-desktop-full \
        bash ' \
> start_docker_task1.sh
```

We have added this volume (-v $HOME/visiont3lab-github/qr_reader/qr_reader_ros_ws/src:/root/catkin_ws/src/ ) to the docker container. It represents our ros workspace that we will use to run our ros packages.

We are ready to start the container as following:

```
chmod a+x launch_task1.sh &&  \
./start_docker_task1.sh
```

After having started the container we need to build and run our ros packages as following:

1. Compile workspace and start roscore. (On the open terminal created by the launch_task1.sh)

    ```
    source /opt/ros/melodic/setup.bash && \
    cd /root/catkin_ws/ && \
    catkin_make && \
    roscore
    ```

2. Run USB Cam package 
    
    Open a new terminal inside the docker container and run the usb_cam node

    ```
    docker exec -it ros_melodic_desktop_full_task_1 /bin/bash -c ' \
    source /root/catkin_ws/devel/setup.bash && \
    roslaunch usb_cam usb_cam-test.launch'
    ```

    The usb_cam-test.launch file is located at "/opt/ros/melodic/share/usb_cam/launch/usb_cam-test.launch" . You can modify it as you like.

3. Run QR Detector package (Open a new terminal (new instance of docker container))
    
    Open a new terminal inside the docker container and run the qr detector 
    Before running this it is required that usb_cam node is started. Wait until you see /usb_cam/image_raw is displayed

    ```
    docker exec -it ros_melodic_desktop_full_task_1 /bin/bash -c ' \
    source /root/catkin_ws/devel/setup.bash && \
    roslaunch qr_detector_pkg qr_detector_cam.launch'
    ```

4. Visualize camera image topic in the browser
   
    ```
    docker exec -it ros_melodic_desktop_full_task_1 /bin/bash -c ' \
    source /root/catkin_ws/devel/setup.bash && \
    roslaunch web_video_server web_video_server.launch'
    ```
    
    Visit http://0.0.0.0:30000 (from the host machine) to have access to the topic image viewer. Note that to be able to visualize the web page running inside the docker container we had to remap host port 30000 with docker container port 30000 (port forwarding, --publish="30000:30000").
    The web_vide_server package will provide:
    * an image_viewer showing all the image topics available at http://0.0.0.0:30000 . By click on them a simple web page displaying the sequence of image will appear.
    * if you are interested in only the streaming, this is available at http://0.0.0.0:30000/stream?topic=/usb_cam/image_raw where /usb_cam/image_raw is the topic name.
    * There is also the possibility to get a video url by typing http://0.0.0.0:30000/stream?topic=/usb_cam/image_raw&type=vp8 . It is also possible to set the type=vp9 or h264, however we found that the vp8 type is the one working better.
    * The image topic name displaying the QR code detection and estimation of it is position and orientation with respect to the camera is the "/output topic". Check both http://0.0.0.0:30000/stream?topic=/output and http://0.0.0.0:30000/stream?topic=/output&type=vp8



### Task 2: Estimate moving QR codes position and orientation (pose) with respect to a fixed camera inside a simulated magazine.

To do this we will use the previous build docker image (ros-melodic-desktop-full). We will modify slightly the launch_test.sh file to adapt to our scenario. In this case we are going to use some created ros packages available in our /root/catkin_ws workspace. For this reason we will need to mount a volume first (containing our pacakges). The file file will became the following:
In this case the launch_test.sh file will became:

```
echo \
    ' #!/bin/bash

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
        -v $HOME/visiont3lab-github/qr_reader/qr_reader_ros_ws/src:/root/catkin_ws/src/ \
        -v $HOME/visiont3lab-github/qr_reader/qr_reader_ros_ws/src/config.yaml:/root/.ignition/fuel/config.yaml  \
        ros-melodic-desktop-full \
        bash ' \
> start_docker_task2.sh
```
For the sake of this example we need to use nvidia-driver (--runtime=nvidia) and we do not need to mount any external device. 

We are ready to start the container as following:

```
chmod a+x launch_task2.sh &&  \
./start_docker_task2.sh
```

Once the container it is started we can run our created ros packages.

```
# Run roscore
docker exec -it ros_melodic_desktop_full_task_2 /bin/bash -c 'cd /root/catkin_ws && source /opt/ros/melodic/setup.bash && catkin_make && source /root/catkin_ws/devel/setup.bash && roscore'

# Run gazebo simulation world
docker exec -it ros_melodic_desktop_full_task_2 /bin/bash -c 'source /root/catkin_ws/devel/setup.bash && roslaunch /root/catkin_ws/src/gazebo_models_pkg/my_world.launch --wait world_name:=qr_world;'

# Run qr cameras detectors
docker exec -it ros_melodic_desktop_full_task_2 /bin/bash -c 'source /root/catkin_ws/devel/setup.bash && roslaunch qr_detector_pkg qr_detector_cams.launch'

# Run web video server for visualization (http://localhost:3000)
docker exec -it ros_melodic_desktop_full_task_2 /bin/bash -c 'source /root/catkin_ws/devel/setup.bash && roslaunch web_video_server web_video_server.launch'

# Extra Run rviz
docker exec -it ros_melodic_desktop_full_task_2 /bin/bash -c \
'source /opt/ros/melodic/setup.bash && rosrun rviz rviz'

# Extra Run rqt_plot
docker exec -it ros_melodic_desktop_full_task_2 /bin/bash -c \
'source /opt/ros/melodic/setup.bash && rosrun rqt_plot rqt_plot'

# To Kill gazebo 
docker exec -it ros_melodic_desktop_full_task_2 /bin/bash -c  'pkill -9 gazebo; pkill -9 gzserver; killall -9 gzclient;'
```


### Extra Notes:

* Host Commands ( My pc where ros-melodic-desktop full is installed)

    ```
    cd $HOME/visiont3lab-github/qr_reader/qr_reader_ros_ws/ && catkin_make
    souce $HOME/visiont3lab-github/qr_reader/qr_reader_ros_ws/devel/setup.bash
    roscore
    roslaunch $HOME/visiont3lab-github/qr_reader/qr_reader_ros_ws/src/gazebo_models_pkg/my_world_host.launch --wait world_name:=qr_world; 
    roslaunch web_video_server web_video_server.launch
    roslaunch qr_detector_pkg qr_detector_cams.launch
    pkill -9 gazebo; pkill -9 gzserver; killall -9 gzclient;
    ```

* Issue with ffmpeg4 : Install [ffmpeg4](https://linuxize.com/post/how-to-install-ffmpeg-on-ubuntu-18-04/) cause web_video_server otherwise will not be able to create compressed video (vp8,vp9,h264).

    ```
    # sudo apt purge ffmpeg
    sudo add-apt-repository ppa:jonathonf/ffmpeg-4
    sudo apt install ffmpeg
    ```

* Gazebo camera parameters meaning [Reference](http://gazebosim.org/tutorials?tut=logical_camera_sensor&cat=sensors)

    * near: Distance in meters from the pose of the sensor to the closest point on the near clip plane.
    * far: Distance in meters from the pose of the sensor to the closest point on the far clip plane.
    * horizontal_fov: The horizontal field of view of the camera in radians.
    * aspect_ratio: The ratio of the width and height of the camera. The aspect ratio combined with the horizontal field of view defines the vertical field of view of the camera.


* Extra Permission

    ```
    xhost +local:docker and xhost -local:docker 
    xhost +local:root   and xhost -local:root 
    ```

* Simulation gazebo-docker issues: 

  1. [Err] [REST.cc:205] Error in REST request
  libcurl: (51) SSL: no alternative certificate subject name matches target host name 'api.ignitionfuel.org' . Solved by modifying the  ~/.ignition/fuel/config.yaml file. Indeed we have mounted a new file inside the docker container.  [Reference](https://bitbucket.org/osrf/gazebo/issues/2607/error-restcc-205-during-startup-gazebo).

  2. ALSA lib confmisc.c:767:(parse_card) cannot find card '0' .Probelm with audio driver  inside the docker container. We have solved mounting them inside the docker container.  [Reference](https://gist.github.com/fsmunoz/d83dbcf6c60b651491e0815b12158d1c)

## Useful reference
[qr code](https://github.com/mikaelarguedas/gazebo_models) <br>
[Generate Qr meshing code](https://github.com/mikaelarguedas/gazebo_models) <br>
[web_video_server](https://msadowski.github.io/ros-web-tutorial-pt3-web_video_server/) <br>
[Ros gazebo docker](http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#Using_nvidia-docker) <br>
[Camera gazebo ros parameters](http://gazebosim.org/tutorials?tut=logical_camera_sensor&cat=sensors) <br>
[install ffmpeg4](https://linuxize.com/post/how-to-install-ffmpeg-on-ubuntu-18-04/) <br>
[Docker with ROS](https://tuw-cpsg.github.io/tutorials/docker-ros/) <br>
[X-Server host permission](http://wiki.ros.org/docker/Tutorials/GUI) <br>
[Docker Alias X server, Audio, dbus scripts](https://gist.github.com/fsmunoz/d83dbcf6c60b651491e0815b12158d1c)