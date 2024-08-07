cd ../Docker/jetson/
OVERRIDE_COMMAND="source /opt/ros/noetic/setup.bash && source /AUV-2024/catkin_ws/devel/setup.bash && roslaunch planner quali.launch" docker compose up