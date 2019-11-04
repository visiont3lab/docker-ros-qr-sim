# Calibrate camera 

[ROS Camera Calibration tool](http://wiki.ros.org/camera_calibration)


```
# Open a terminal and do
./start_docker_task1.sh

source /opt/ros/melodic/setup.bash
cd /root/catkin_ws/
catkin_make

source devel/setup.bash
roslaunch usb_cam usb_cam-test.launch

# On second terminal do:
docker exec -it ros_melodic_desktop_task1 /bin/bash
source /opt/ros/melodic/devel/setup.bash
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.03 image:=/usb_cam/image_raw camera:=/usb_cam

#calibration data  are inside /tmp folder
```