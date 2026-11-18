#!/usr/bin/env python3
"""
vlm_models.py - small VLMs behind one interface:  model.query(PIL image, question) -> {"answer": str}

  moondream : vikhyatk/moondream2 (the model of all experiments; revision from MOONDREAM_REVISION)
  qwen2vl   : Qwen/Qwen2-VL-2B-Instruct
  smolvlm   : HuggingFaceTB/SmolVLM-Instruct
Used by eval_moondream.py (--model NAME) for the cross-model verification study. Greedy decoding, at most 24 new tokens.
"""
import os

REPOS = {"qwen2vl": "Qwen/Qwen2-VL-2B-Instruct", "smolvlm": "HuggingFaceTB/SmolVLM-Instruct"}


class HFChat:
    def __init__(self, name, device):
        import torch
        from transformers import AutoProcessor
        repo = REPOS[name]
        self.device = device
        dtype = torch.bfloat16 if device == "cuda" else torch.float32
        if name == "qwen2vl":
            from transformers import Qwen2VLForConditionalGeneration
            # cap the number of visual tokens (the frames are 1280x720)
            self.processor = AutoProcessor.from_pretrained(repo, min_pixels=256 * 28 * 28, max_pixels=640 * 28 * 28)
            self.model = Qwen2VLForConditionalGeneration.from_pretrained(repo, torch_dtype=dtype)
        else:
            from transformers import AutoModelForVision2Seq
            self.processor = AutoProcessor.from_pretrained(repo, do_image_splitting=False)
            self.model = AutoModelForVision2Seq.from_pretrained(repo, torch_dtype=dtype)
        self.model = self.model.to(device).eval()
        self.repo = repo

    def query(self, image, question):
        import torch
        messages = [{"role": "user", "content": [{"type": "image"}, {"type": "text", "text": question}]}]
        text = self.processor.apply_chat_template(messages, add_generation_prompt=True)
        inputs = self.processor(text=[text], images=[image], return_tensors="pt").to(self.device)
        with torch.no_grad():
            out = self.model.generate(**inputs, max_new_tokens=24, do_sample=False)
        new = out[:, inputs["input_ids"].shape[1]:]
        return {"answer": self.processor.batch_decode(new, skip_special_tokens=True)[0].strip()}


def load_model(name):
    import torch
    device = "cuda" if torch.cuda.is_available() else "cpu"
    if name == "moondream":
        from transformers import AutoModelForCausalLM
        rev = os.environ.get("MOONDREAM_REVISION") or None
        model = AutoModelForCausalLM.from_pretrained("vikhyatk/moondream2", revision=rev, trust_remote_code=True)
        model = model.to(device).eval()
        print("model ready, revision", getattr(model.config, "_commit_hash", None), flush=True)
        return model
    if name in REPOS:
        m = HFChat(name, device)
        print("model ready:", m.repo, flush=True)
        return m
    raise SystemExit(f"unknown model {name}; choose from moondream, {', '.join(REPOS)}")
