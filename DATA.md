# Data formats

The result files are distributed as assets of the release `v1.0.0` of this repository: `results_and_manifests.tar.gz` (the series of the paper, `worlds/` manifests, `CONTENTS.md`, `SHA256SUMS.txt`)
and `verification_frames.tar.gz` (the frames of the verification study, `verif_frames/`). The checksums of the two archives are in `README.md`. `tools/collect_results.sh` builds the archives on the cluster.
The frames of the navigation runs are not archived; debug and smoke-test files are left out.

## World manifest `<name>.json` (written by `gen_world.py`)
`name`, `seed`, `room_size` (m), `start` (robot start), `params` (generator arguments), `instances` (`id`, `model`, `class`, `x`, `y`, `yaw`, `radius`),
`duplicate_models`, `relocations` (`id`, `model`, `class`, `from`, `to`: planned move of one object). Positions are in metres in the world frame; the room is a square
centred at the origin. The Gazebo file `<name>.world` is generated from the same seed.

## Navigation results `exp_<world><TAG>.csv` (written by `run_experiment.py`)
One row per mission (episode x condition).

| Column | Meaning |
|---|---|
| `world`, `scenario`, `episode`, `cond` | world name; `S1` duplicates or `S2` relocation; episode index; `B0`, `B1`, `B1r`, `B2` or `FULL` |
| `cls`, `target` | queried class; id of the target instance in the manifest |
| `sx`, `sy` | start position |
| `success` | strict: the controller reached its final goal and the robot ends within `radius + 1.0 m` of a true instance |
| `near` | the robot ends within `radius + 1.0 m` of a true instance |
| `decision_ok` | the declared goal lies within `radius + 0.6 m` of a true instance (independent of the controller) |
| `reached` | the controller reported the goal as reached |
| `reason` | `verified`, `found_by_search`, `NOT_FOUND`, `no_candidate` or `no_verification` (baselines) |
| `n_attempts`, `outcomes` | number of candidate attempts; their outcomes joined by `>` (`verified`, `not_found`, `unreachable`, `unverified`) |
| `vlm_calls`, `vlm_s` | requests to the VLM (every question and every request for a box after a "yes"); seconds spent in the VLM |
| `path`, `sim_s`, `wall_s` | path length (m), simulated time (s), wall-clock time (s) |
| `legs` | JSON list of controller legs: `target`, `steps`, `reason` (`goal`, `stuck`, `collision`, `max_steps`) |
| `recoveries`, `scene_dev` | recovery manoeuvres; deviation of objects from their expected positions after the reset (check) |
| `vlm_log` | JSON list of looks: `k` (index; also the number of the frame file), `tag` (`face` look at a candidate, `sweep`, `peripheral`, `scan`), `cls`, `yes`, `frame` (file name), `robot` = [x, y, yaw], `pos` = localised position or null |
| `prompts`, `rule`, `n_views`, `view_rule`, `sweep`, `search_dist`, `sigma` | configuration of the run (`sweep` in radians, `search_dist` and `sigma` in metres) |

The **primary success rate** (SR) of the paper is computed by `analyze_exp.py`: `near` and the mission declared a goal (`reason` is not `NOT_FOUND`/`no_candidate`).
The frames named in `vlm_log` belong to the frame folders of the runs; only the frames used in the figures are in `paper_figures/frames/`.

## Verification study
* `verif_<world>/`: PNG frames at approach poses (object present / object moved out of the room) and `index.json` (camera intrinsics, exact object poses at capture time). Over the five worlds there are 118 poses
  and 236 frames. The frames contain renderings of models of the Gazebo model database (https://github.com/osrf/gazebo_models, Creative Commons Attribution 3.0 Unported, https://creativecommons.org/licenses/by/3.0/).
* `verif_office_s<k>.csv` (prompts A-D on the full frame), `verif_office_s<k>_crop.csv` (centre-crop prompts, suffix `c`), `verif_office_s<k>_choice.csv` (choice prompts E/Ec) and
  `xverif_<model>_office_s<k>.csv` (`qwen2vl`, `smolvlm`; written by `eval_moondream.py`), `k` = 1..5: `frame`, `condition`, `target`, `query_cls`, `kind` (target / absent class), `label`
  (`pos`, `neg`, `amb`: computed from the stored poses and the field of view), `prompt` (A-E, suffix `c` = centre crop), `answer`, `yes`, `latency`.

## Controller test `ctrl_<world>.csv` (written by `test_controller.py`)
One row per episode of the oracle-goal test of a released TD3 policy (goal at the true position of an object).
