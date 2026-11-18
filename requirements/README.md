# Versions used

* **Analysis, tests, world generator**: Python 3 standard library only (the analysis scripts, `gen_world.py`, `test_logic.py`); `test_controller.py` and the controller side need NumPy.
* **ROS / controller container** (`controller_ros.txt`): Ubuntu 20.04, ROS Noetic, Gazebo 11.15.1, Python 3.8.10.
* **VLM container** (`vlm_server.txt`): PyTorch 2.5.1 (cu121), Transformers 4.46.3. Moondream2 additionally needs the packages of its remote code at the pinned revision
  (install them as the model card of that revision says).
* **Camera**: `realsense2_description` (issaiass fork) commit `d54b5e6503d3e33611df8c7ab152d442bb2b2841`; `realsense_gazebo_plugin` (issaiass) commit
  `a64ea263754bd1e923f26a3290ee4e86e6aa291a`. The colour stream is set to the resolution and field of view of the depth stream (1280x720, 85.2 degrees) in
  `urdf/_d435.gazebo.xacro`, so a pixel of a colour box indexes the registered depth image.
* **VLM**: Moondream2 revision `5d6c926f44e26b07957b0dd315bbedcb4c17a5fe` (set as `MOONDREAM_REVISION` in the jobs).

## Catkin workspace (`$VLMNAV_BASE/vlm_nav/catkin_ws/src`)

| Package | Source | Commit |
|---|---|---|
| `multi_robot_scenario`, `my_robot_navigation`, `velodyne_simulator` | the original system's repository (`Yernar-SDU/Yernar-SDU-VLM_project-VLM_DRL-based-autonomous-robot-ObjectNav`); not redistributed | `467a6ad441fc877eca0429a6a590f865590f0e59` |
| `realsense2_description` | `https://github.com/issaiass/realsense2_description` | `d54b5e6503d3e33611df8c7ab152d442bb2b2841` |
| `realsense_gazebo_plugin` | `https://github.com/issaiass/realsense_gazebo_plugin` | `a64ea263754bd1e923f26a3290ee4e86e6aa291a` |

Our edit of the camera package (colour stream with the resolution and field of view of the depth stream, 1280x720, 85.2 degrees, in `urdf/_d435.gazebo.xacro`) is described in the top-level `README.md`.
