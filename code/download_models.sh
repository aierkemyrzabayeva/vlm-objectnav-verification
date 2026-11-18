#!/bin/bash
# Run ONCE on the login node (needs internet): caches the two extra VLMs in $BASE/hf_cache.
BASE=${VLMNAV_BASE:?set VLMNAV_BASE to the project directory}
export HF_HOME=$BASE/hf_cache
apptainer exec --bind $BASE $BASE/vlm2.sif python -c "
from huggingface_hub import snapshot_download
for repo in ('Qwen/Qwen2-VL-2B-Instruct', 'HuggingFaceTB/SmolVLM-Instruct'):
    print('downloading', repo, flush=True)
    print(snapshot_download(repo), flush=True)
print('DOWNLOAD_DONE')
"
