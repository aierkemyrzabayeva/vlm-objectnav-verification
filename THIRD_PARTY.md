# Third-party components (not redistributed here)

Check the license of every component at its source before you redistribute anything derived from it.

| Component | Used for | Where to get it |
|---|---|---|
| Original VLM+TD3 system (`Yernar-SDU/Yernar-SDU-VLM_project-VLM_DRL-based-autonomous-robot-ObjectNav`) | environment `TD3/real_env_dd.py` (patched by `make_env_ours.py`) and the TD3 policy files | clone from GitHub; its license was not clear when this package was prepared. Its repository has contained an API key: do not use it |
| Gazebo model collection (osrf/gazebo_models) | the 14 object models of `gen_world.py` | public Gazebo model database, licensed under Creative Commons Attribution 3.0 Unported (`LICENSE` of the repository); the version used was not recorded, so `containers/world_models.sha256` lists the checksums of the files that were used |
| `realsense2_description`, `realsense_gazebo_plugin` (issaiass forks) | simulated RGB-D camera; commits in `requirements/` | GitHub |
| Moondream2, revision `5d6c926f44e26b07957b0dd315bbedcb4c17a5fe` | main VLM | Hugging Face (`vikhyatk/moondream2`) |
| Qwen2-VL-2B-Instruct, SmolVLM-Instruct | VLMs of the cross-model study | Hugging Face (`Qwen/Qwen2-VL-2B-Instruct`, `HuggingFaceTB/SmolVLM-Instruct`) |
| ROS Noetic, Gazebo 11, Stable-Baselines3, PyTorch, Transformers | runtime | see `requirements/` |

`make_env_ours.py` and `make_launch_ours.py` contain only short match patterns and replacement strings for two files of the original system (`TD3/real_env_dd.py` and `TD3/assets/multi_robot_scenario.launch`); the original files themselves are not part of this repository.
