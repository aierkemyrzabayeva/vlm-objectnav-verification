#!/bin/bash
# Runs INSIDE the ros_nav2.sif container (called by smoke3.sh).
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
WS=$BASE/vlm_nav/catkin_ws
WORLD=${WORLD:-office_s1}
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash
export GAZEBO_MODEL_PATH=$BASE/world_models
export GAZEBO_MODEL_DATABASE_URI=""
export ROS_MASTER_URI=http://localhost:11399
export GAZEBO_MASTER_URI=http://localhost:12399

# the authors' launch files resolve world files relative to the package launch/ directory
cp -f $BASE/vlm_nav/worlds/$WORLD.world $WS/src/multi_robot_scenario/launch/$WORLD.world
echo "world: $WORLD"

roscore -p 11399 > /tmp/roscore.log 2>&1 &
sleep 6
xvfb-run -a -s "-screen 0 1280x1024x24" roslaunch -p 11399 $BASE/vlm_nav/ours/multi_robot_scenario.launch world_name:=$WORLD.world > /tmp/launch.log 2>&1 &

for i in $(seq 1 60); do
  if rostopic list 2>/dev/null | grep -q "/realsense_camera/color/image_raw"; then echo "camera topic appeared after $((i*5))s"; break; fi
  sleep 5
done
sleep 20

echo "=== MODELS IN WORLD ==="
rostopic echo -n 1 /gazebo/model_states/name 2>/dev/null | tr -d '[]' | tr ',' '\n' | head -n 60
echo "=== RTF ==="
timeout 25 rostopic echo -n 3 /gazebo/performance_metrics | grep -i "real_time_factor"
echo "=== COLOR/DEPTH/POINTS RATE (sim time) ==="
timeout 12 rostopic hz /realsense_camera/color/image_raw 2>&1 | grep "average rate" | tail -n 1
timeout 12 rostopic hz /realsense_camera/depth/image_raw 2>&1 | grep "average rate" | tail -n 1
timeout 20 rostopic hz /realsense_camera/depth/color/points 2>&1 | grep "average rate" | tail -n 1
echo "=== FRAMES ==="
python3 $BASE/vlm_nav/ours/grab_frames.py $BASE/frames_$WORLD
echo "=== MISSING MODELS / ERRORS ==="
grep -i -E "unable to find|not found|error|died|exception" /tmp/launch.log | sort | uniq -c | head -n 30
pkill -f gzserver; pkill -f roslaunch; pkill -f roscore; pkill -f Xvfb
sleep 3
