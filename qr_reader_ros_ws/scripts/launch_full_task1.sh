#!/bin/bash

xfce4-terminal \
--tab --title "START_CONTAINER" --command "bash -c \"
./start_docker_task1.sh;
exec bash\""  \
--tab --title "ROSCORE" --command "bash -c \"
sleep 10 && docker exec -it ros_melodic_desktop_full_task_1 /bin/bash -c 'cd /root/catkin_ws && source /opt/ros/melodic/setup.bash && catkin_make && source /root/catkin_ws/devel/setup.bash && roscore';
exec bash\""  \
--tab --title "USB_CAM"	--command "bash -c \"
sleep 20 && docker exec -it ros_melodic_desktop_full_task_1 /bin/bash -c 'source /root/catkin_ws/devel/setup.bash && roslaunch usb_cam usb_cam-test.launch --wait'
exec bash\""  \
--tab --title "QR_DETECTOR"	--command "bash -c \"
sleep 20 && docker exec -it ros_melodic_desktop_full_task_1 /bin/bash -c 'source /root/catkin_ws/devel/setup.bash && roslaunch qr_detector_pkg qr_detector_cam.launch --wait';
exec bash\""  \
--tab --title "WEB_VIDEO_SERVER" --command "bash -c \"
sleep 20 && docker exec -it ros_melodic_desktop_full_task_1 /bin/bash -c 'source /root/catkin_ws/devel/setup.bash && roslaunch web_video_server web_video_server.launch --wait';
exec bash\"" & 