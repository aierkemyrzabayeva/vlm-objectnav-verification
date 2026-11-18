#!/usr/bin/env python3
"""
eval_moondream.py - how reliable is the VLM's yes/no verification at the approach pose?

For each collected frame the VLM is asked about every object class of the world plus a few classes
that do not exist. Ground truth comes from the geometry stored in index.json (object positions at
capture time, camera pose and field of view):
  pos  : an instance of the class is well inside the field of view and not occluded
  amb  : partly visible / occluded / borderline  -> excluded from the statistics
  neg  : no instance of the class can be in the image

Key numbers (they decide whether verify-and-repair can work):
  * present condition, target class : P(yes | target in view)       (should be high)
  * moved   condition, target class : P(yes | target removed)       = FALSE CONFIRMATION rate (should be ~0)
  * absent classes                  : P(yes | class does not exist)

Usage (VLM container, GPU): python3 eval_moondream.py --frames DIR --out results/verif_office_s1.csv
       python3 eval_moondream.py --selftest        (pure Python, no model)
"""
import argparse
import csv
import json
import math
import os
import sys
import time

PROMPTS = {
    "A": 'Does this image clearly show a "{cls}"?\nAnswer with ONLY "yes" or "no".\nAnswer:',
    "B": "Is there a {cls} in this image? Answer yes or no.",
    # C: conservative - tells the model to say no when unsure
    "C": 'Look carefully at the image. Is a "{cls}" clearly visible in it? '
         'If you are not sure, answer "no". Answer with ONLY "yes" or "no".',
    # D: counting question - 0 means not present (counting is usually less prone to agreeing)
    "D": 'How many "{cls}" can you see in this image? Answer with a single number.',
    # E: closed-vocabulary choice - the model has to pick ONE name, so similar classes compete with each other
    "E": 'Look at the object in the center of the image. Which one of these is it: {options}? '
         'Answer with exactly one item from the list.',
}
ABSENT = ["elephant", "bicycle", "sofa"]
CROP = (0.25, 0.10, 0.75, 0.90)     # centre crop used by prompt keys ending with "c" (x0, y0, x1, y1 as fractions)


def crop_center(img):
    w, h = img.size
    return img.crop((int(CROP[0] * w), int(CROP[1] * h), int(CROP[2] * w), int(CROP[3] * h)))


def cropped_hfov(hfov_deg):
    """Horizontal FOV of the centre crop (the crop keeps the central CROP[2]-CROP[0] of the width)."""
    frac = CROP[2] - CROP[0]
    return math.degrees(2 * math.atan(frac * math.tan(math.radians(hfov_deg) / 2)))
CENTRAL = 0.8          # object centre must lie within this fraction of the half field of view
MAX_DIST = 5.0


def wilson(k, n, z=1.96):
    if n == 0:
        return float("nan"), float("nan")
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return c - h, c + h


def object_status(cam_xy_yaw, hfov_deg, o, others):
    """'vis' | 'amb' | 'no' for one object seen from the camera pose."""
    cx, cy, yaw = cam_xy_yaw
    dx, dy = o["x"] - cx, o["y"] - cy
    fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
    lat = -dx * math.sin(yaw) + dy * math.cos(yaw)
    dist = math.hypot(dx, dy)
    if fwd <= 0.1 or dist > MAX_DIST + 2.0:
        return "no"
    bearing = abs(math.atan2(lat, fwd))
    half = math.radians(hfov_deg) / 2.0
    a = math.atan2(o["radius"], dist)
    if bearing - a >= half:
        return "no"                                   # entirely outside the frame
    if bearing > CENTRAL * half or dist > MAX_DIST:
        return "amb"                                  # partly in frame / far away
    for p in others:                                  # occlusion by something closer on the line of sight
        if p["id"] == o["id"]:
            continue
        pdx, pdy = p["x"] - cx, p["y"] - cy
        pf = pdx * math.cos(yaw) + pdy * math.sin(yaw)
        pl = -pdx * math.sin(yaw) + pdy * math.cos(yaw)
        if 0.1 < pf < fwd and abs(pl - lat * pf / fwd) < p["radius"] + 0.2:
            return "amb"
    return "vis"


def label_class(frame, cam, cls):
    cx = frame["robot"][0] + cam["cam_forward"] * math.cos(frame["robot"][2])
    cy = frame["robot"][1] + cam["cam_forward"] * math.sin(frame["robot"][2])
    pose = (cx, cy, frame["robot"][2])
    sts = [object_status(pose, cam["hfov_deg"], o, frame["objects"]) for o in frame["objects"] if o["class"] == cls]
    if "vis" in sts:
        return "pos"
    if "amb" in sts:
        return "amb"
    return "neg"


def summarize(rows, prompt):
    rs = [r for r in rows if r["prompt"] == prompt and r["label"] != "amb"]
    out = [f"=== prompt {prompt}: {len(rs)} labelled queries (ambiguous excluded)"]

    def rate(sel, title):
        n = len(sel)
        k = sum(r["yes"] for r in sel)
        lo, hi = wilson(k, n)
        out.append(f"{title:<58s} yes={k}/{n} = {k / n if n else float('nan'):.2f}  (95% CI {lo:.2f}-{hi:.2f})")

    rate([r for r in rs if r["label"] == "pos"], "TPR   all classes in view")
    rate([r for r in rs if r["label"] == "neg" and r["kind"] == "world"], "FPR   world classes not in view")
    rate([r for r in rs if r["kind"] == "absent"], "FPR   classes that do not exist in the world")
    rate([r for r in rs if r["kind"] == "target" and r["condition"] == "present" and r["label"] == "pos"],
         "approach, object PRESENT  (target class): P(yes)")
    rate([r for r in rs if r["kind"] == "target" and r["condition"] == "moved" and r["label"] == "neg"],
         "approach, object MOVED    (target class): P(yes) = false confirmation")
    return "\n".join(out)


def selftest():
    cam = {"hfov_deg": 69.4, "cam_forward": 0.0}
    # robot at origin looking +x; the same geometry as in the probe frame analysed earlier
    objs = [{"id": "person_0", "class": "person", "x": 1.897, "y": 1.39, "radius": 0.35},
            {"id": "cone_0", "class": "cone", "x": 3.42, "y": 1.65, "radius": 0.25},
            {"id": "lamp_0", "class": "lamp", "x": 3.07, "y": -0.6, "radius": 0.30},
            {"id": "tree_0", "class": "tree", "x": 2.79, "y": -2.64, "radius": 0.6},
            {"id": "cab_0", "class": "cabinet", "x": -3.3, "y": 1.03, "radius": 0.45}]
    fr = {"robot": [0.0, 0.0, 0.0], "objects": objs}
    got = {c: label_class(fr, cam, c) for c in ("person", "cone", "lamp", "tree", "cabinet", "elephant")}
    # person: half outside the frame -> ambiguous; cone: its line of sight is partly blocked by the person
    assert got["person"] == "amb" and got["cone"] == "amb" and got["lamp"] == "pos", got
    assert got["cabinet"] == "neg" and got["elephant"] == "neg", got
    # without the person the cone is clearly visible
    fr0 = {"robot": [0.0, 0.0, 0.0], "objects": [o for o in objs if o["id"] != "person_0"]}
    assert label_class(fr0, cam, "cone") == "pos"
    # occlusion: a barrel right in front of the lamp makes it ambiguous
    fr2 = {"robot": [0.0, 0.0, 0.0], "objects": objs + [{"id": "b", "class": "barrel", "x": 1.5, "y": -0.3, "radius": 0.35}]}
    assert label_class(fr2, cam, "lamp") == "amb"
    rows = []
    for i in range(10):
        rows.append({"prompt": "A", "label": "pos", "kind": "target", "condition": "present", "yes": 1 if i < 9 else 0})
        rows.append({"prompt": "A", "label": "neg", "kind": "target", "condition": "moved", "yes": 1 if i < 2 else 0})
    s = summarize(rows, "A")
    assert "yes=9/10" in s and "yes=2/10" in s, s
    assert parse_yes("Yes.", "A") == 1 and parse_yes("no", "A") == 0
    assert parse_yes("2", "D") == 1 and parse_yes("0", "D") == 0 and parse_yes("There are 3 of them", "D") == 1
    assert parse_yes("none", "D") == 0 and parse_yes("one", "D") == 1
    opts = ["barrel", "traffic cone", "cabinet", "nothing"]
    assert parse_choice("traffic cone", "traffic cone", opts) == 1 and parse_choice("A barrel.", "traffic cone", opts) == 0
    assert parse_choice("barrel, traffic cone", "traffic cone", opts) == 0 and parse_choice("nothing", "cabinet", opts) == 0
    from PIL import Image as _I
    assert crop_center(_I.new("RGB", (1280, 720))).size == (640, 576)
    assert abs(cropped_hfov(85.2) - 2 * math.degrees(math.atan(0.5 * math.tan(math.radians(42.6))))) < 1e-9
    print("SELFTEST_OK", got)


def parse_choice(ans, cls, options):
    """1 if the answer names exactly one option and it is `cls`."""
    a = " " + str(ans).strip().lower().replace(".", " ").replace(",", " ") + " "
    found = []
    for opt in sorted(options, key=len, reverse=True):         # longest names first ("traffic cone" before "cone")
        o = " " + opt.lower() + " "
        if o in a:
            found.append(opt.lower())
            a = a.replace(o, " ")
    return int(len(found) == 1 and found[0] == cls.lower())


def parse_yes(ans, pk="A"):
    a = str(ans).strip().lower()
    if pk == "D":                                   # counting prompt: first integer >= 1 means present
        import re
        m = re.search(r"\d+", a)
        if m:
            return int(int(m.group(0)) >= 1)
        words = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5}
        return int(any(w in a.split() for w in words))
    return int(a.startswith("yes"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames")
    ap.add_argument("--out")
    ap.add_argument("--prompts", default="A,B,C,D", help="comma-separated; suffix c = centre crop, e.g. A,Ac,D,Dc")
    ap.add_argument("--model", default="moondream", help="moondream | qwen2vl | smolvlm (see vlm_models.py)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()

    from PIL import Image
    from vlm_models import load_model
    model = load_model(a.model)

    idx = json.load(open(os.path.join(a.frames, "index.json")))
    cam = idx["camera"]
    world_classes = sorted({o["class"] for o in idx["frames"][0]["objects"]})
    rows = []
    for fr in idx["frames"]:
        img = Image.open(os.path.join(a.frames, fr["file"])).convert("RGB")
        for cls in world_classes + ABSENT:
            kind = "absent" if cls in ABSENT else ("target" if cls == fr["target_class"] else "world")
            for pk in a.prompts.split(","):
                crop = pk.endswith("c")
                base = pk[0]
                if base == "E" and kind != "target":
                    continue                                     # the choice question is about the centre object only
                cam_k = dict(cam, hfov_deg=cropped_hfov(cam["hfov_deg"])) if crop else cam
                label = "neg" if kind == "absent" else label_class(fr, cam_k, cls)
                t = time.time()
                opts = world_classes + ["nothing"]
                out = model.query(crop_center(img) if crop else img, PROMPTS[base].format(cls=cls, options=", ".join(opts)))
                ans = out["answer"] if isinstance(out, dict) else str(out)
                rows.append({"frame": fr["file"], "condition": fr["condition"], "target": fr["target"],
                             "query_cls": cls, "kind": kind, "label": label, "prompt": pk, "answer": ans.strip(),
                             "yes": parse_choice(ans, cls, opts) if base == "E" else parse_yes(ans, base), "latency": round(time.time() - t, 3)})
        print(f"  {fr['file']} done", flush=True)
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    for pk in a.prompts.split(","):
        print(summarize(rows, pk), flush=True)
    lat = [r["latency"] for r in rows]
    print(f"queries={len(rows)} mean latency={sum(lat) / len(lat):.3f}s", flush=True)
    print("EVAL_DONE", flush=True)


if __name__ == "__main__":
    main()
