#!/usr/bin/env python3
"""
analyze_verif.py - break down the VLM verification results (CSV written by eval_moondream.py).

Sections
  1. per query class: TPR (class in view) and FPR (class of the world, not in view)
  2. at the approach pose: which targets are wrongly rejected (object present) or wrongly confirmed (moved)
  3. decision rules built from the two prompts (A, B, A-or-B, A-and-B): P(yes | present) and P(yes | moved)
  4. confusions: which OTHER classes were in view when the VLM said 'yes' to a class that was not in view
     (needs verif_<world>/index.json next to the results/ directory; found automatically)
  5. multi-view rules per target (any / majority / all of the 2-3 approach azimuths)
  6. confusion RATES: P(yes | asked class Y, the object in the centre of the view is class X), the pairs with the
     highest rates (the case that matters for verification: a similar object stands where the target should be)
  7. choice prompts (E / Ec): what the model calls the object in the centre of the view
  8. joint rules across result files of the same world/frame, e.g. 'Dc and E' or 'Dc unless E names another class'

Usage: python3 analyze_verif.py [--detail A,D] [--classes "cabinet,table,..."] results/verif_office_s1.csv [more.csv ...]
       (--classes keeps only the rows about these classes, e.g. the reliable set)
       (--detail: prompts shown in sections 1 and 4; sections 2, 3, 5 always cover all prompts)
       python3 analyze_verif.py --selftest
Pure Python (csv module only).
"""
import csv
import json
import math
import os
import sys
from collections import Counter, defaultdict


def wilson(k, n, z=1.96):
    if n == 0:
        return float("nan"), float("nan")
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def fmt(k, n):
    lo, hi = wilson(k, n)
    return f"{k:3d}/{n:<3d} = {k / n if n else float('nan'):.2f}  (CI {lo:.2f}-{hi:.2f})"


def load(paths):
    rows = []
    for p in paths:
        for r in csv.DictReader(open(p)):
            r["yes"] = int(r["yes"])
            r["src"] = p
            rows.append(r)
    return rows


def analyze(rows, detail=("A",)):
    out = []
    lab = [r for r in rows if r["label"] != "amb" and r["prompt"] in detail]

    out.append(f"== 1. per query class (prompt(s) {','.join(detail)})")
    by = defaultdict(lambda: {"pos": [0, 0], "neg": [0, 0]})
    for r in lab:
        if r["kind"] == "absent":
            continue
        by[r["query_cls"]][r["label"]][0] += r["yes"]
        by[r["query_cls"]][r["label"]][1] += 1
    out.append(f"{'class':<18s}{'TPR (in view)':<30s}FPR (world class, not in view)")
    for c in sorted(by):
        p, n = by[c]["pos"], by[c]["neg"]
        out.append(f"{c:<18s}{fmt(*p) if p[1] else '-':<30s}{fmt(*n) if n[1] else '-'}")

    out.append("\n== 2. approach pose, target class")
    tf = defaultdict(dict)                      # (src, frame) -> {prompt: yes}
    meta = {}
    for r in rows:
        if r["kind"] == "target":
            tf[(r["src"], r["frame"])][r["prompt"]] = r["yes"]
            meta[(r["src"], r["frame"])] = (r["condition"], r["target"], r["label"])
    for pk in sorted({p for d in tf.values() for p in d}):
        rej = sorted(meta[k][1] + "/" + k[1].rsplit("_", 1)[0][-12:] for k, d in tf.items()
                     if meta[k][0] == "present" and meta[k][2] == "pos" and d.get(pk) == 0)
        conf = sorted(meta[k][1] for k, d in tf.items()
                      if meta[k][0] == "moved" and meta[k][2] == "neg" and d.get(pk) == 1)
        out.append(f"prompt {pk}: wrongly REJECTED (object present): {rej or '-'}")
        out.append(f"prompt {pk}: wrongly CONFIRMED (object moved):  {conf or '-'}")

    out.append("\n== 3. decision rules at the approach pose")
    present_prompts = sorted({p for d in tf.values() for p in d})
    rules = {pk: ((pk,), (lambda d, pk=pk: d[pk])) for pk in present_prompts}
    if "A" in present_prompts and "B" in present_prompts:
        rules["A or B"] = (("A", "B"), lambda d: int(d["A"] or d["B"]))
        rules["A and B"] = (("A", "B"), lambda d: int(d["A"] and d["B"]))
    out.append(f"{'rule':<10s}{'P(yes | object present)':<34s}P(yes | object moved)  <- false confirmation")
    for name, (need, fn) in rules.items():
        pres = [fn(d) for k, d in tf.items() if meta[k][0] == "present" and meta[k][2] == "pos" and all(n in d for n in need)]
        mov = [fn(d) for k, d in tf.items() if meta[k][0] == "moved" and meta[k][2] == "neg" and all(n in d for n in need)]
        if pres and mov:
            out.append(f"{name:<10s}{fmt(sum(pres), len(pres)):<34s}{fmt(sum(mov), len(mov))}")

    out.append("\n== 4. confusions: classes in view when the VLM said 'yes' to a class that was NOT in view")
    out.append(confusions([r for r in rows if r["prompt"] in detail]))

    out.append("\n== 6. confusion rates: asked class Y while the centre object is class X (approach pose, object present)")
    pairs = defaultdict(lambda: [0, 0])
    tgt_cls = {}
    for r in rows:
        if r["kind"] == "target" and r["label"] == "pos":
            tgt_cls[(r["src"], r["frame"])] = r["query_cls"]
    for r in rows:
        if r["prompt"] in detail and r["kind"] == "world" and r["label"] == "neg" and r["condition"] == "present":
            x = tgt_cls.get((r["src"], r["frame"]))
            if x and x != r["query_cls"]:
                pairs[(r["query_cls"], x)][0] += r["yes"]
                pairs[(r["query_cls"], x)][1] += 1
    top = sorted(((k, v) for k, v in pairs.items() if v[1] >= 8), key=lambda kv: -kv[1][0] / kv[1][1])[:12]
    out.append(f"{'asked':<18s}{'centre object':<18s}P(yes)")
    for (y, x), (k, n) in top:
        out.append(f"{y:<18s}{x:<18s}{fmt(k, n)}")

    out.append("\n== 7. choice prompts: what the model calls the centre object (object present)")
    for pk in sorted({r["prompt"] for r in rows if r["prompt"][0] == "E"}):
        cnt, tot = defaultdict(Counter), Counter()
        for r in rows:
            if r["prompt"] == pk and r["kind"] == "target" and r["condition"] == "present" and r["label"] == "pos":
                cnt[r["query_cls"]][r["answer"].strip().strip(".").lower()] += 1
                tot[r["query_cls"]] += 1
        out.append(f"prompt {pk}")
        for x in sorted(tot):
            right = cnt[x].get(x.lower(), 0)
            wrong = ", ".join(f"{a} x{k}" for a, k in cnt[x].most_common() if a != x.lower())[:78]
            out.append(f"  centre={x:<18s} named correctly {right:>2d}/{tot[x]:<3d}  other answers: {wrong or '-'}")

    out.append("\n== 8. joint rules (same world and frame, prompts from different result files)")
    out.append(joint_rules(rows))

    out.append("\n== 5. multi-view rules per target (views = approach azimuths of the same object)")
    views = defaultdict(list)                   # (src, condition, target) -> [ {prompt: yes}, ... ]
    for k, d in tf.items():
        views[(k[0], meta[k][0], meta[k][1])].append((meta[k][2], d))
    rules5 = {"any view": lambda ys: int(sum(ys) >= 1),
              "majority": lambda ys: int(sum(ys) * 2 > len(ys)),
              "all views": lambda ys: int(sum(ys) == len(ys))}
    out.append(f"{'prompt/rule':<18s}{'P(yes | object present)':<34s}P(yes | object moved)")
    for pk in present_prompts:
        for name, fn in rules5.items():
            pres, mov = [], []
            for (src, cond, tgt), lst in views.items():
                if len(lst) < 2:
                    continue
                ys = [d[pk] for lab, d in lst if pk in d and ((cond == "present" and lab == "pos") or (cond == "moved" and lab == "neg"))]
                if len(ys) < 2:
                    continue
                (pres if cond == "present" else mov).append(fn(ys))
            if pres and mov:
                out.append(f"{pk + ' / ' + name:<18s}{fmt(sum(pres), len(pres)):<34s}{fmt(sum(mov), len(mov))}")
    return "\n".join(out)


def _world_of(src):
    import re
    name = os.path.basename(src)
    name = name[len("verif_"):] if name.startswith("verif_") else name
    name = name.rsplit(".", 1)[0]
    while re.search(r"_(crop|choice)$", name):
        name = name.rsplit("_", 1)[0]
    return name


def joint_rules(rows):
    """Rules that combine prompts stored in different files (matched by world and frame)."""
    J, meta = defaultdict(dict), {}
    for r in rows:
        if r["kind"] != "target":
            continue
        key = (_world_of(r["src"]), r["frame"])
        ans = r["answer"].strip().strip(".").lower()
        other = (r["prompt"][0] == "E" and not r["yes"] and ans not in ("", "nothing") and "nothing" not in ans)
        J[key][r["prompt"]] = (r["yes"], other)
        meta[key] = (r["condition"], r["label"])
    combos = [("Dc", "E", "and"), ("Dc", "Ec", "and"), ("Dc", "E", "veto"), ("Dc", "Ec", "veto"),
              ("D", "E", "veto"), ("D", "Ec", "veto"), ("A", "E", "veto"), ("Ac", "Ec", "veto")]
    lines = [f"{'rule':<22s}{'P(yes | object present)':<34s}P(yes | object moved)"]
    for a, b, mode in combos:
        pres, mov = [], []
        for key, d in J.items():
            if a not in d or b not in d:
                continue
            ya, yb, ob = d[a][0], d[b][0], d[b][1]
            v = int(ya and yb) if mode == "and" else int(ya and not ob)
            cond, lab = meta[key]
            if cond == "present" and lab == "pos":
                pres.append(v)
            elif cond == "moved" and lab == "neg":
                mov.append(v)
        if pres and mov:
            name = f"{a} and {b}" if mode == "and" else f"{a} unless {b} names other"
            lines.append(f"{name:<22s}{fmt(sum(pres), len(pres)):<34s}{fmt(sum(mov), len(mov))}")
    return "\n".join(lines)


def confusions(rows):
    try:
        from eval_moondream import object_status
    except Exception as e:                       # pragma: no cover
        return f"(skipped: cannot import eval_moondream: {e})"
    cache = {}

    def frames_of(src):
        if src not in cache:
            w = os.path.basename(src)
            w = w[len("verif_"):-4] if w.startswith("verif_") else w[:-4]
            root = os.path.dirname(os.path.dirname(os.path.abspath(src)))
            ipath = os.path.join(root, "verif_" + w, "index.json")
            while not os.path.exists(ipath) and "_" in w:             # strip suffixes such as _crop
                w = w.rsplit("_", 1)[0]
                ipath = os.path.join(root, "verif_" + w, "index.json")
            if os.path.exists(ipath):
                idx = json.load(open(ipath))
                cache[src] = (idx["camera"], {f["file"]: f for f in idx["frames"]})
            else:
                cache[src] = None
        return cache[src]

    pairs, nothing, total = Counter(), Counter(), Counter()
    for r in rows:
        if r["kind"] != "world" or r["label"] != "neg" or r["yes"] != 1:
            continue
        info = frames_of(r["src"])
        if info is None:
            return "(skipped: index.json not found next to the results directory)"
        cam, fr = info
        f = fr[r["frame"]]
        x = f["robot"][0] + cam["cam_forward"] * math.cos(f["robot"][2])
        y = f["robot"][1] + cam["cam_forward"] * math.sin(f["robot"][2])
        vis = sorted({o["class"] for o in f["objects"]
                      if object_status((x, y, f["robot"][2]), cam["hfov_deg"], o, f["objects"]) == "vis"})
        total[r["query_cls"]] += 1
        if not vis:
            nothing[r["query_cls"]] += 1
        for v in vis:
            pairs[(r["query_cls"], v)] += 1
    if not total:
        return "no false 'yes' answers on world classes"
    lines = [f"{'asked class':<18s}{'false yes':<11s}{'nothing else in view':<22s}most frequent classes in view"]
    for c, n in total.most_common():
        top = ", ".join(f"{v} x{k}" for (q, v), k in pairs.most_common() if q == c)[:80]
        lines.append(f"{c:<18s}{n:<11d}{nothing[c]:<22d}{top or '-'}")
    return "\n".join(lines)


def selftest():
    import os
    import tempfile
    p = os.path.join(tempfile.mkdtemp(), "t.csv")
    hdr = ["frame", "condition", "target", "query_cls", "kind", "label", "prompt", "answer", "yes", "latency"]
    rows = []
    for i in range(10):
        f = f"obj{i}_0_present.png"
        for pk, y in (("A", 1 if i < 7 else 0), ("B", 1 if i < 8 else 0)):
            rows.append([f, "present", f"obj{i}", "cone", "target", "pos", pk, "yes", y, 0.1])
        f = f"obj{i}_0_moved.png"
        for pk, y in (("A", 1 if i < 1 else 0), ("B", 1 if i < 3 else 0)):
            rows.append([f, "moved", f"obj{i}", "cone", "target", "neg", pk, "no", y, 0.1])
        rows.append([f, "moved", f"obj{i}", "elephant", "absent", "neg", "A", "no", 0, 0.1])
    with open(p, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(hdr)
        w.writerows(rows)
    s = analyze(load([p]))
    assert "  7/10" in s and "  8/10" in s and "  1/10" in s and "  3/10" in s, s
    assert "A and B" in s and "A or B" in s
    print("SELFTEST_OK")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        selftest()
    elif len(sys.argv) > 1:
        args = sys.argv[1:]
        detail, classes = ("A",), None
        while args and args[0] in ("--detail", "--classes"):
            if args[0] == "--detail":
                detail = tuple(args[1].split(","))
            else:
                classes = {c.strip() for c in args[1].split(",")}
            args = args[2:]
        rows = load(args)
        if classes:
            rows = [r for r in rows if r["query_cls"] in classes]
        print(analyze(rows, detail))
    else:
        print(__doc__)
