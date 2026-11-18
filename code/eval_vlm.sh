#!/bin/bash
#SBATCH --job-name=eval_vlm
#SBATCH --partition=ic-express
#SBATCH --account=YOUR_ACCOUNT
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=01:30:00
#SBATCH --output=%x_%j.log
# Cross-model verification study: MODEL=qwen2vl|smolvlm  WORLD=office_s1..5  PROMPTS=A,D,E (default)
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
WORLD=${WORLD:-office_s1}
MODEL=${MODEL:-qwen2vl}
export HF_HOME=$BASE/hf_cache
mkdir -p $BASE/results
cd $BASE/vlm_nav/ours
apptainer exec --nv --bind $BASE $BASE/vlm2.sif python -u eval_moondream.py --model $MODEL \
    --frames $BASE/verif_$WORLD --out $BASE/results/xverif_${MODEL}_${WORLD}.csv --prompts ${PROMPTS:-A,D,E}
echo DONE
