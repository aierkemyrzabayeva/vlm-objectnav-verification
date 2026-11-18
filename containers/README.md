# Containers

The experiments ran in two Apptainer containers: `ros_nav2.sif` (ROS Noetic, Gazebo 11, Stable-Baselines3, the catkin workspace tools) and `vlm2.sif`
(PyTorch, Transformers, FastAPI). Add their definition files here (`ros_nav2.def`, `vlm2.def`) so that others can build them, together with the `catkin_ws` build instructions
(which camera packages at which commits: see `requirements/README.md`) and note that the launch file `multi_robot_scenario.launch` that the jobs read from `$VLMNAV_BASE/vlm_nav/ours/` is NOT included: it is the original system's launch file with two edits (world file from
`$WORLD_FILE`, rviz node removed). Create it with `code/make_launch_ours.py`.


## Build chain

```
ros_noetic.sif --(ros_nav.def)--> ros_nav.sif --(ros_nav2.def)--> ros_nav2.sif      controller / Gazebo container
docker pytorch/pytorch:2.4.1-cuda12.1-cudnn9-runtime --(vlm.def)--> vlm.sif --(vlm2.def)--> vlm2.sif      VLM container
```

1. `ros_noetic.sif`: the public OSRF image `docker://osrf/ros:noetic-desktop-full`; `pull_ros.sh` pulls it.
2. Copy the `.def` files and `inner_build.sh` to `$VLMNAV_BASE`, set `YOUR_PARTITION` and `YOUR_ACCOUNT` in the scripts, then run `build_ros_nav.sh`, `build_ros_nav2.sh`,
   `build_vlm.sh`, `build_vlm2.sh` (each with `VLMNAV_BASE` exported). The ROS layers need `--fakeroot --ignore-fakeroot-command`, as in the scripts.
3. The sources of the catkin workspace: `setup_catkin_ws.sh` clones the original repository to `$VLMNAV_BASE/vlm_nav` at commit `467a6ad441fc877eca0429a6a590f865590f0e59` (it contains the packages `multi_robot_scenario`,
   `my_robot_navigation`, `velodyne_simulator`), clones the two camera packages at the pinned commits and applies our edit of the colour camera (`patches/realsense2_description_d435_color.patch`). Build the workspace with
   `build_ws.sh` (it uses `ros_nav.sif`, as in the experiments). `fetch_world_models.sh` fetches the Gazebo object models of the generator into `$VLMNAV_BASE/world_models`.
4. The final versions inside the images are in `../requirements/` (measured with the commands in `../REPRODUCE.md`).

The images were built on the Illinois Campus Cluster. `ros_nav.def` installs the `openai` package only because the original system imports it; no API key is used or needed.
