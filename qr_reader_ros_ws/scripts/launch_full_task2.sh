#!/bin/bash

xfce4-terminal \
--tab --title "START_CONTAINER" --command "bash -c \"
./start_docker_task2.sh;
exec bash\""  \
--tab --title "ROSCORE" --command "bash -c \"
sleep 4 && docker exec -it ros_melodic_desktop_full_task_2 /bin/bash -c 'cd /root/catkin_ws && source /opt/ros/melodic/setup.bash && catkin_make && source /root/catkin_ws/devel/setup.bash && roscore';
exec bash\""  \
--tab --title "GAZEBO PLUGIN BUILD" --command "bash -c \"
sleep 15 && docker exec -it ros_melodic_desktop_full_task_2 /bin/bash -c 'source /root/catkin_ws/devel/setup.bash && cd /root/catkin_ws/src/gazebo_models_pkg/ && mkdir -p build && cd build && cmake .. && make';
exec bash\""  \
--tab --title "GAZEBO SIMULATION"	--command "bash -c \"
sleep 35 && docker exec -it ros_melodic_desktop_full_task_2 /bin/bash -c 'source /root/catkin_ws/devel/setup.bash && roslaunch /root/catkin_ws/src/gazebo_models_pkg/my_world.launch --wait world_name:=qr_world';
exec bash\""  \
--tab --title "QR_DETECTOR"	--command "bash -c \"
sleep 35 && docker exec -it ros_melodic_desktop_full_task_2 /bin/bash -c 'source /root/catkin_ws/devel/setup.bash && roslaunch qr_detector_pkg qr_detector_cams.launch --wait';
exec bash\""  \
--tab --title "WEB_VIDEO_SERVER" --command "bash -c \"
sleep 35 && docker exec -it ros_melodic_desktop_full_task_2 /bin/bash -c 'source /root/catkin_ws/devel/setup.bash && roslaunch web_video_server web_video_server.launch --wait';
exec bash\"" & 
