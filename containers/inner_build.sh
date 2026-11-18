#!/bin/bash
# Runs INSIDE ros_nav.sif (called by build_ws.sh): builds the catkin workspace in $VLMNAV_BASE/vlm_nav/catkin_ws
source /opt/ros/noetic/setup.bash
cd ${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}/vlm_nav/catkin_ws
rm -rf build devel
catkin_make -DCMAKE_BUILD_TYPE=Release
echo BUILD_EXIT $?
