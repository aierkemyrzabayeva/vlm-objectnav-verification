#!/usr/bin/env python3
"""
vlm_verify_server.py - minimal HTTP server around Moondream2 for the experiments
(runs in the VLM container on the GPU node; the ROS container talks to it via localhost).

  GET  /health                         -> {"ok": true, "revision": "<commit hash>"}
  POST /verify  image, cls, prompt     -> {"answer": str, "yes": 0/1, "latency": s}     (prompts A-D, see eval_moondream.py)
  POST /detect  image, cls             -> {"objects": [{x_min, y_min, x_max, y_max}], "latency": s}   (normalised boxes)
"""
import argparse
import io
import os
import threading
import time

import torch
import uvicorn
from fastapi import FastAPI, Form, UploadFile
from PIL import Image
from transformers import AutoModelForCausalLM

from eval_moondream import PROMPTS, crop_center, parse_choice, parse_yes

app = FastAPI(title="moondream verify server")
LOCK = threading.Lock()                      # the model is not thread-safe
MODEL = None
REVISION = None


def load_model():
    global MODEL, REVISION
    rev = os.environ.get("MOONDREAM_REVISION") or None
    MODEL = AutoModelForCausalLM.from_pretrained("vikhyatk/moondream2", revision=rev, trust_remote_code=True)
    MODEL = MODEL.to("cuda" if torch.cuda.is_available() else "cpu").eval()
    REVISION = getattr(MODEL.config, "_commit_hash", rev)
    print("model ready, revision", REVISION, flush=True)


def _img(upload: UploadFile) -> Image.Image:
    return Image.open(io.BytesIO(upload.file.read())).convert("RGB")


@app.get("/health")
def health():
    return {"ok": MODEL is not None, "revision": REVISION}


@app.post("/verify")
def verify(image: UploadFile, cls: str = Form(...), prompt: str = Form("A"), options: str = Form("")):
    img = _img(image)
    base = prompt[0]
    if prompt.endswith("c"):                                  # e.g. "Ac": same question on the centre crop
        img = crop_center(img)
    with LOCK:
        t = time.time()
        out = MODEL.query(img, PROMPTS[base].format(cls=cls, options=options.replace(",", ", ")))
        lat = time.time() - t
    ans = out["answer"] if isinstance(out, dict) else str(out)
    yes = parse_choice(ans, cls, options.split(",")) if base == "E" else parse_yes(ans, base)
    return {"answer": ans, "yes": yes, "latency": lat}


@app.post("/detect")
def detect(image: UploadFile, cls: str = Form(...)):
    img = _img(image)
    with LOCK:
        t = time.time()
        out = MODEL.detect(img, cls)
        lat = time.time() - t
    return {"objects": out.get("objects", []) if isinstance(out, dict) else [], "latency": lat}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8123)
    a = ap.parse_args()
    load_model()
    uvicorn.run(app, host="127.0.0.1", port=a.port, log_level="warning")
