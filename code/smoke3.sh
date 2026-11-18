#!/bin/bash
#SBATCH --job-name=smoke3
#SBATCH --partition=IllinoisComputes
#SBATCH --account=YOUR_ACCOUNT
#SBATCH --cpus-per-task=8
#SBATCH --mem=24G
#SBATCH --time=00:30:00
#SBATCH --output=%x_%j.log
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
export WORLD=${WORLD:-office_s1}
apptainer exec --bind $BASE $BASE/ros_nav2.sif bash $BASE/vlm_nav/ours/inner_smoke3.sh
echo DONE
