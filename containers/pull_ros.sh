#!/bin/bash
#SBATCH --job-name=pull_ros
#SBATCH --partition=YOUR_PARTITION
#SBATCH --account=YOUR_ACCOUNT
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=01:00:00
#SBATCH --output=%x_%j.log
# Base image of the ROS container: the public ROS Noetic desktop-full image of OSRF.
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
export APPTAINER_CACHEDIR=$BASE/.apptainer_cache
export APPTAINER_TMPDIR=$BASE/.apptainer_tmp
mkdir -p $APPTAINER_CACHEDIR $APPTAINER_TMPDIR
cd $BASE
apptainer pull ros_noetic.sif docker://osrf/ros:noetic-desktop-full
echo "DONE"
