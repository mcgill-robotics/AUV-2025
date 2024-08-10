#!/bin/bash

CMD="source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/local_setup.bash && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i optional_opencv_calibration_file:=/SN33688213.conf"
echo $CMD

docker exec -it jetson-zed-1 bash -c "$CMD"