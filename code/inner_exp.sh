#!/bin/bash
# Runs INSIDE ros_nav2.sif (called by exp.sh). The env starts roscore + Gazebo itself.
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
WS=$BASE/vlm_nav/catkin_ws
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash
export GAZEBO_MODEL_PATH=$BASE/world_models
export GAZEBO_MODEL_DATABASE_URI=""
export ROS_PORT_SIM=${ROS_PORT_SIM:-11511}
export ROS_MASTER_URI=http://localhost:$ROS_PORT_SIM
export GAZEBO_MASTER_URI=http://localhost:$((ROS_PORT_SIM + 1000))
export WORLD=${WORLD:-office_s1}
export WORLD_FILE=$BASE/vlm_nav/worlds/$WORLD.world
export PYTHONPATH=$BASE/vlm_nav/ours:$PYTHONPATH
cd $BASE/vlm_nav/ours
echo "world=$WORLD episodes=${EPISODES:-3} prompts=${PROMPTS:-A} rule=${RULE:-and} n_views=${N_VIEWS:-1} view_rule=${VIEW_RULE:-any} sweep=${SWEEP:-0} search_dist=${SEARCH_DIST:-2.5}"
xvfb-run -a -s "-screen 0 1280x1024x24" python3 -u run_experiment.py \
    --world $WORLD --scenarios ${SCEN:-S1 S2} --conditions ${CONDS:-B0 B1 B1r B2 FULL} --episodes ${EPISODES:-3} \
    --prompts ${PROMPTS:-A} --rule ${RULE:-and} --n_views ${N_VIEWS:-1} --view_rule ${VIEW_RULE:-any} --sweep ${SWEEP:-0} --search_verify_dist ${SEARCH_DIST:-2.5} \
    --vlm_url http://127.0.0.1:${VLM_PORT:-8123} \
    --policy ${POLICY:-$BASE/vlm_nav/TD3/td3_gazebo_custom_policy.zip} \
    --frames_dir $BASE/results/frames_${WORLD}${TAG:-} \
    --out $BASE/results/exp_${WORLD}${TAG:-}.csv
