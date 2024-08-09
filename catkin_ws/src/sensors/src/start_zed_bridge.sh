#!/bin/bash

CMD="source /opt/ros/humble/setup.bash && source /set_env.bash && source /root/ros2_ws/install/local_setup.bash && ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i"
echo $CMD

docker exec -dt jetson-zed-1 bash -c "$CMD"


CMD="source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash && source /set_env.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics"
echo $CMD

docker exec -it jetson-bridge-1 bash -c "$CMD"