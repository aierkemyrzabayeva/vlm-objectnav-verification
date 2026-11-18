#!/bin/bash
#SBATCH --job-name=vlm_probe
#SBATCH --partition=IllinoisComputes
#SBATCH --account=YOUR_ACCOUNT
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=00:40:00
#SBATCH --output=%x_%j.log

BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
export HF_HOME=$BASE/hf_cache
mkdir -p "$HF_HOME"
IMG=${IMG:-$(ls $BASE/vlm_nav/images/*.png $BASE/vlm_nav/images/*.jpg $BASE/vlm_nav/images/*.jpeg 2>/dev/null | head -n 1)}
echo "image: $IMG"
cd $BASE/vlm_nav/ours
apptainer exec --bind $BASE $BASE/vlm.sif python vlm_probe.py "$IMG" person table chair elephant
echo DONE
