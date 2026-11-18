#!/bin/bash
# Runs INSIDE ros_nav2.sif (called by diag_turn.sh).
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
WS=$BASE/vlm_nav/catkin_ws
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash
export GAZEBO_MODEL_PATH=$BASE/world_models
export GAZEBO_MODEL_DATABASE_URI=""
export ROS_PORT_SIM=${ROS_PORT_SIM:-11611}
export ROS_MASTER_URI=http://localhost:$ROS_PORT_SIM
export GAZEBO_MASTER_URI=http://localhost:$((ROS_PORT_SIM + 1000))
export WORLD=${WORLD:-office_s1}
export WORLD_FILE=$BASE/vlm_nav/worlds/$WORLD.world
export PYTHONPATH=$BASE/vlm_nav/ours:$PYTHONPATH
cd $BASE/vlm_nav/ours
xvfb-run -a -s "-screen 0 1280x1024x24" python3 -u diag_turn.py
