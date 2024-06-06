#!/usr/bin/env bash

####################################
PLANNER_LAUNCH_FILE="mission.launch"
LOG_FILE_BRINGUP="~/tmp/bringup.log"
LOG_FILE_PLANNER="~/tmp/planner.log"
####################################

cd ~/AUV-2024/catkin_ws && catkin clean -y && catkin build && source devel/setup.bash

tmux new-session -d -s ros_session_bringup "roslaunch bringup bringup.launch &> $LOG_FILE_BRINGUP"
sleep 5 # Wait a few seconds to ensure the first node has started
tmux new-session -d -s ros_session_planner "roslaunch planner $PLANNER_LAUNCH_FILE &> $LOG_FILE_PLANNER"

echo "Mission started."
echo "Logs can be found in $LOG_FILE_BRINGUP and $LOG_FILE_PLANNER"