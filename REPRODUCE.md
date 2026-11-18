# Reproducing the results

Set `export VLMNAV_BASE=<project directory>` before using any script that touches the cluster layout (Slurm jobs and the Python files that read paths).
All commands below are run from `code/` unless stated otherwise.

## Level 0: offline checks (any machine with Python 3, no simulator, no GPU)

```
python3 test_logic.py ./gen_world.py          # 10 tests of the memory and the mission logic (toy simulator; not experimental results)
python3 analyze_exp.py --selftest
python3 analyze_verif.py --selftest
python3 compare_versions.py --selftest
python3 collect_verif.py --selftest
python3 test_controller.py --selftest         # needs numpy
python3 gen_world.py --seed 51 --catalog reliable --out /tmp/w --name test2_r51
diff /tmp/w/test2_r51.json ../examples/test2_r51.json     # must print nothing
```
`bash tools/check_release.sh` does all of this and also compares the analysis of the example files with `examples/expected_*.txt`.

## Level 1: re-analyse the released result files (standard library only)

Download the result archive (see `DATA.md`) and unpack it into `results/`. The analysis scripts read the CSV files; the world manifests are found through
`--worlds_dir` or `VLMNAV_BASE/vlm_nav/worlds`.

Patterns are expanded by the shell for `analyze_exp.py`, `analyze_failures.py` and `analyze_verif.py` (do not quote them); `compare_versions.py` and `legs_table.py` expand
them themselves (quote them). The file-name patterns are those of our runs; adjust them to your file names.

| Element of the paper | Command | Input |
|---|---|---|
| Navigation results, per scenario and condition (SR, decision accuracy, VLM calls, path, time; Wilson intervals; exact McNemar tests) | `python3 analyze_exp.py results/exp_test2_r[34]*_v2.csv` | V2 series, worlds 32-36 and 41-45 |
| Same, V1 series | `python3 analyze_exp.py results/exp_test_r1*_mainA.csv` | worlds 11-15 |
| Success per class | add `--by-class` | as above |
| Missions that declared a wrong goal, with the VLM log | add `--wrong` | as above |
| Ablation of the approach geometry (paired; four arms) | `python3 compare_versions.py --a "results/exp_test2_r[34]*_standoffonly.csv" --b "results/exp_test2_r[34]*_v1abl.csv" --cond FULL` (and the same for `_sweeponly`, `_v2`) | worlds 32-36 and 41-45 |
| Confirmation on fresh worlds | `python3 compare_versions.py --a "results/exp_test2_r5*_standoffonly.csv" --b "results/exp_test2_r5*_v1abl.csv" --cond FULL` | worlds 51-55 |
| Failure causes, per-look geometry, localisation error | `python3 analyze_failures.py results/exp_test2_r3*_v2.csv` (options `--looks`, `--bydist`) | V2 series |
| Controller legs per condition | `python3 legs_table.py "results/exp_test2_r3*_v2.csv"` | V2 series |
| Verification study of Moondream2 (per class, confusions, decision rules, multi-view, choice prompts) | `python3 analyze_verif.py --detail A,D results/verif_office_s[1-5].csv results/verif_office_s[1-5]_crop.csv results/verif_office_s[1-5]_choice.csv` (`--classes` restricts to the reliable classes) | the three verification CSVs per development world `office_s1..s5` |
| Cross-model study (Qwen2-VL-2B, SmolVLM) | `python3 analyze_verif.py --detail A,D results/xverif_qwen2vl_office_s[1-5].csv` and the same for `xverif_smolvlm_...` | cross-model CSVs |
| Figures of the paper | `cd paper_figures && python3 make_figures.py out data/test2_r32.json && python3 make_trace.py out && python3 make_frames.py out` | needs matplotlib and Pillow |

## Level 2: re-running the simulations (Slurm cluster with Apptainer, NVIDIA GPU)

You need the two containers, the workspace and the object models; none of the third-party parts is in this repository (see `THIRD_PARTY.md`), but `containers/` has the definition files and the scripts
that fetch or create them: `pull_ros.sh`, `build_ros_nav.sh`, `build_ros_nav2.sh`, `build_vlm.sh`, `build_vlm2.sh` (containers), `setup_catkin_ws.sh` then `build_ws.sh` (workspace with the camera patch),
`fetch_world_models.sh` (object models). See `containers/README.md` for the order. The TD3 policy files come from the original repository (`vlm_nav/TD3/`).

1. Put the files of `code/` into `$VLMNAV_BASE/vlm_nav/ours/` and create the patched environment (the original file is never modified):
   `python3 make_env_ours.py ../TD3/real_env_dd.py real_env_ours.py`, and the launch file the same way:
   `python3 make_launch_ours.py ../TD3/assets/multi_robot_scenario.launch multi_robot_scenario.launch`.
2. Edit `#SBATCH --account=YOUR_ACCOUNT` (and the partition) in the `*.sh` files.
3. Generate the worlds (deterministic):
   ```
   for s in 32 33 34 35 36 41 42 43 44 45 51 52 53 54 55; do
     python3 gen_world.py --seed $s --catalog reliable --out ../worlds --name test2_r$s
   done
   ```
4. Cache the extra VLMs once on a node with internet access: `bash download_models.sh`.
5. Controller test with the oracle goal: `WORLD=<name> STARTS=2 sbatch ctrl_test.sh` (writes `results/ctrl_<name>.csv`; four released policies are compared).
6. Verification dataset and study: `WORLD=<name> sbatch collect_verif.sh`, then `WORLD=<name> sbatch eval_moondream.sh` (prompts `A,B,C,D`, suffix `c` = centre crop)
   and `MODEL=qwen2vl|smolvlm WORLD=<name> sbatch eval_vlm.sh` (prompts `A,D,E`).
7. Navigation experiments (one job per world; each runs both scenarios and all five conditions). The series of the paper:

   | Series | Variables of `exp.sh` | Worlds |
   |---|---|---|
   | V2 (main) | `EPISODES=6 PROMPTS=A N_VIEWS=1 VIEW_RULE=any SWEEP=0.35 SEARCH_DIST=2.5 TAG=_v2` | `test2_r32..r36`, `test2_r41..r45` |
   | V1 geometry (ablation) | `... SWEEP=0 SEARCH_DIST=1.5 TAG=_v1abl` and `CONDS="FULL"` | same worlds, and `r51..r55` |
   | sweep only | `... SWEEP=0.35 SEARCH_DIST=1.5 TAG=_sweeponly` and `CONDS="FULL"` | worlds 32-36 and 41-45 |
   | standoff only | `... SWEEP=0 SEARCH_DIST=2.5 TAG=_standoffonly` and `CONDS="FULL"` | worlds 32-36, 41-45, 51-55 |

   Example: `CONDS="FULL" sbatch --export=ALL,WORLD=test2_r51,EPISODES=6,PROMPTS=A,N_VIEWS=1,VIEW_RULE=any,SWEEP=0,SEARCH_DIST=2.5,TAG=_standoffonly exp.sh`
   (values with spaces cannot be passed inside `--export`; put them in front of `sbatch` as above). Ports are derived from the job id, so several jobs can share a node.
   A job writes `results/exp_<world><TAG>.csv` and the frames of every verification to `results/frames_<world><TAG>/`.
   The worlds of the V1 series (11-15), the stress series (21-23, `--catalog full`) and the development worlds `office_s1..s5` (14-class catalogue) were generated the same way
   (seeds 1-5, 11-15, 21-23); their names are in the `world` column of the released CSV files and the manifests are in the archive (`worlds/`).

Do not change the code while jobs are queued: a job reads the files when it starts.
