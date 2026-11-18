#!/usr/bin/env python3
"""
vlm_probe.py - one-off check of the Moondream2 interface used by the baseline (vlm_server.py).

Prints: library versions, model commit hash (to pin the revision in the paper), which methods
exist (query / detect / point / caption), the RAW structure returned by each call, and timings.
It also runs the baseline's own "verify" prompt and a deliberately wrong class, so we see how
the model answers when the object is NOT there (false-confirmation behaviour).

Usage:  python3 vlm_probe.py IMAGE.png [class1 class2 ...]
Env:    MOONDREAM_REVISION (optional) pin a revision; HF_HOME cache dir
"""
import json
import os
import sys
import time

import torch
import transformers
from PIL import Image
from transformers import AutoModelForCausalLM

img_path = sys.argv[1]
classes = sys.argv[2:] or ["person", "table", "chair", "elephant"]
rev = os.environ.get("MOONDREAM_REVISION") or None
dev = "cuda" if torch.cuda.is_available() else "cpu"
print(f"transformers={transformers.__version__} torch={torch.__version__} device={dev} revision={rev}")


def show(title, obj):
    print(f"--- {title}")
    try:
        print(json.dumps(obj, indent=1, default=str)[:1500])
    except Exception:
        print(repr(obj)[:1500])


t0 = time.time()
model = AutoModelForCausalLM.from_pretrained("vikhyatk/moondream2", revision=rev, trust_remote_code=True)
try:
    model = model.to(dev)
except Exception as e:
    print("model.to(dev) failed (model may manage its own device):", e)
model.eval()
print(f"loaded in {time.time() - t0:.1f}s")
print("commit hash:", getattr(model.config, "_commit_hash", None))
print("has query/detect/point/caption:", [hasattr(model, m) for m in ("query", "detect", "point", "caption")])

image = Image.open(img_path).convert("RGB")
print("image size:", image.size, "file:", img_path)

for cls in classes:
    verify_prompt = (f'Does this image clearly show a "{cls}"?\n'
                     'Answer with ONLY "yes" or "no".\nAnswer:')
    if hasattr(model, "query"):
        t = time.time()
        out = model.query(image, verify_prompt)
        show(f"query verify '{cls}' ({time.time() - t:.1f}s)", out)
    if hasattr(model, "detect"):
        t = time.time()
        out = model.detect(image, cls)
        show(f"detect '{cls}' ({time.time() - t:.1f}s)", out)

if torch.cuda.is_available():
    print("peak GPU memory (GB):", round(torch.cuda.max_memory_allocated() / 1e9, 2))
print("PROBE_DONE")
