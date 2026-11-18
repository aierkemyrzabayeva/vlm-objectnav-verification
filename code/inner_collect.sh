#!/bin/bash
# Runs INSIDE ros_nav2.sif (called by collect_verif.sh).
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
WS=$BASE/vlm_nav/catkin_ws
WORLD=${WORLD:-office_s1}
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash
export GAZEBO_MODEL_PATH=$BASE/world_models
export GAZEBO_MODEL_DATABASE_URI=""
export ROS_MASTER_URI=http://localhost:11499
export GAZEBO_MASTER_URI=http://localhost:12499
export WORLD_FILE=$BASE/vlm_nav/worlds/$WORLD.world

roscore -p 11499 > /tmp/roscore.log 2>&1 &
sleep 6
xvfb-run -a -s "-screen 0 1280x1024x24" roslaunch -p 11499 $BASE/vlm_nav/ours/multi_robot_scenario.launch world_name:=$WORLD_FILE > /tmp/launch.log 2>&1 &
for i in $(seq 1 60); do
  if rostopic list 2>/dev/null | grep -q "/realsense_camera/color/image_raw"; then echo "camera topic after $((i*5))s"; break; fi
  sleep 5
done
sleep 25
python3 -u $BASE/vlm_nav/ours/collect_verif.py --world $WORLD --out $BASE/verif_$WORLD --per_obj ${PER_OBJ:-3}
RC=$?
mkdir -p $BASE/logs; cp /tmp/launch.log $BASE/logs/collect_launch_$WORLD.log
pkill -f gzserver; pkill -f roslaunch; pkill -f roscore; pkill -f Xvfb
exit $RC
