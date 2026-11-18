#!/bin/bash
#SBATCH --job-name=eval_md
#SBATCH --partition=ic-express
#SBATCH --account=YOUR_ACCOUNT
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=00:40:00
#SBATCH --output=%x_%j.log
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
WORLD=${WORLD:-office_s1}
export HF_HOME=$BASE/hf_cache
export MOONDREAM_REVISION=5d6c926f44e26b07957b0dd315bbedcb4c17a5fe
mkdir -p $BASE/results
cd $BASE/vlm_nav/ours
apptainer exec --nv --bind $BASE $BASE/vlm2.sif python -u eval_moondream.py \
    --frames $BASE/verif_$WORLD --out $BASE/results/verif_${WORLD}${SUF:-}.csv --prompts ${PROMPTS:-A,B,C,D}
echo DONE
