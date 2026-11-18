#!/bin/bash
#SBATCH --job-name=exp
#SBATCH --partition=ic-express
#SBATCH --account=YOUR_ACCOUNT
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=48G
#SBATCH --time=04:00:00
#SBATCH --output=%x_%j.log
# Variables you can pass with: sbatch --export=ALL,WORLD=office_s1,EPISODES=5,PROMPTS=A,RULE=and,EXTRA_VIEWS=0,TAG=_run1 exp.sh
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
export HF_HOME=$BASE/hf_cache
export MOONDREAM_REVISION=5d6c926f44e26b07957b0dd315bbedcb4c17a5fe
mkdir -p $BASE/logs $BASE/results
cd $BASE/vlm_nav/ours

# ports derived from the job id, so several jobs can run on one node at the same time
export VLM_PORT=$((8100 + SLURM_JOB_ID % 800))
export ROS_PORT_SIM=$((11700 + SLURM_JOB_ID % 200))

# 1) Moondream server on the GPU (background)
apptainer exec --nv --bind $BASE $BASE/vlm2.sif python -u vlm_verify_server.py --port $VLM_PORT \
    > $BASE/logs/vlm_server_$SLURM_JOB_ID.log 2>&1 &
SRV=$!
for i in $(seq 1 120); do
  if curl -s http://127.0.0.1:$VLM_PORT/health | grep -q '"ok":true'; then echo "VLM server ready after $((i*5))s"; break; fi
  sleep 5
done

# 2) Gazebo + controller + experiment in the ROS container (same node, talks to the server on localhost)
apptainer exec --bind $BASE $BASE/ros_nav2.sif bash $BASE/vlm_nav/ours/inner_exp.sh
RC=$?
kill $SRV 2>/dev/null
echo "EXIT: $RC"
echo DONE
