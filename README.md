# Verification and recovery for memory-based object navigation (small VLM + learned controller)

Code, world generator, experimental protocol and analysis scripts for the paper

> *Verification and Recovery for Memory-Based Object Navigation with a Small VLM and a Learned Controller*
> (manuscript under review; the citation will be added here, see `CITATION.cff`).

The study measures how reliable a small vision-language model (Moondream2, with Qwen2-VL-2B and SmolVLM for comparison) is as a
verifier of a remembered object position at the robot's approach pose, and what a verification-and-repair layer does for a memory-based
object-goal system with a learned point-goal controller (TD3) in a Gazebo benchmark with duplicated classes and relocated objects.

## What is in this repository (and what is not)

| Directory | Content |
|---|---|
| `code/` | our own code: world generator, instance memory, verification and repair, simulation wrapper, experiment runner, VLM server, analysis and tests, Slurm jobs |
| `examples/` | one world of the study (seed 51, both arms of the ablation) with the output of the analysis scripts, used as a smoke test |
| `paper_figures/` | scripts that draw the figures of the paper from the reported numbers and logs, with the data they need |
| `requirements/` | the versions we used, per component |
| `tools/` | `check_release.sh` (offline checks) and `collect_results.sh` (collects the result files on the cluster) |
| `containers/` | where the Apptainer definition files belong (see the note there) |
| `results/` | placeholder; the result files are release assets of this repository (see `DATA.md`) |

**Not included, by design:** the original system and its TD3 weights, the Gazebo model collection, the camera packages, and the VLM weights. Obtain them
from their sources (`THIRD_PARTY.md`). **No API key is needed for any experiment (the VLM runs locally). Do not use or redistribute any API key that
may be present in the original repository.**

## Quick check without a simulator or GPU

```
bash tools/check_release.sh
```

runs the 10 offline tests of the mission logic, the self-tests of the analysis scripts, regenerates a world manifest and compares it with the one used in the
experiments (it must be identical), and re-runs the analysis on the example files and compares the output with the stored expected output.

## Data archives (release assets of `v1.0.0`)

| File | Content | SHA-256 |
|---|---|---|
| `results_and_manifests.tar.gz` | result files of all series, world manifests, `CONTENTS.md`, `SHA256SUMS.txt` | `7eb24cea95ab37d8c41ae2308d18ea7ebf611161030dc985a06322a2827ebcb2` |
| `verification_frames.tar.gz` | frames of the verification study (renderings of Gazebo models, CC BY 3.0, see `DATA.md`) | `e50d53d733a83dd5812988b744fb6e2f0b1e998276d977ffbd286f94ca42c60d` |

Check a download with `sha256sum <file>` (Linux) or `shasum -a 256 <file>` (macOS). `SHA256SUMS.txt` inside the first archive lists every file of both archives.

## Reproduce

See `REPRODUCE.md` (three levels: offline checks, re-analysis of the released results, re-running the simulations on a Slurm cluster) and `DATA.md`
(file formats). A table in `REPRODUCE.md` maps every table and figure of the paper to the command that produces it.

## Notes on reproducibility

* The world generator is deterministic: a manifest generated here (Python 3.12) for seed 51 is byte-identical to the one produced on the cluster
  (Python 3.8) and used in the experiments (checked by `tools/check_release.sh`).
* The memory of the experiments is built from the world manifest with Gaussian noise (`--sigma 0.2`), not by an exploration phase.
* The initial series V1 (worlds 11-15) was run with an earlier revision of `sim_env.py` that had no room-bounds check on localised positions; the released
  code always applies the check. The V1 geometry itself (`SWEEP=0`, `SEARCH_DIST=1.5`) can be run. The V1 result files are released as they were produced.
* Only path constants and the Slurm account of the files in `code/` differ from the code that produced the results (the paths are now read from the
  environment variable `VLMNAV_BASE`); the logic files are byte-identical. `CHECKSUMS_CODE.txt` lists which files are identical and which were changed.
* The version of the Gazebo model collection was not recorded when the experiments were run; `containers/world_models.sha256` identifies the model files that were used and `containers/fetch_world_models.sh` checks them.
* Simulation only. Real-robot behaviour is not evaluated.

## License and third-party components

Our code is under the MIT License (`LICENSE`). Third-party components keep their own licenses (`THIRD_PARTY.md`).
