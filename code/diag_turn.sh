#!/bin/bash
#SBATCH --job-name=diag_turn
#SBATCH --partition=IllinoisComputes
#SBATCH --account=YOUR_ACCOUNT
#SBATCH --cpus-per-task=8
#SBATCH --mem=24G
#SBATCH --time=00:20:00
#SBATCH --output=%x_%j.log
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
apptainer exec --bind $BASE $BASE/ros_nav2.sif bash $BASE/vlm_nav/ours/inner_diag.sh
echo DONE
