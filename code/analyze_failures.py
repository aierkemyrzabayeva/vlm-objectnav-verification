#!/usr/bin/env python3
"""
analyze_failures.py - why do the verification conditions (B2, FULL) fail in the relocation scenario S2?

For every S2 mission the VLM log (looks with their localised positions) is compared with the true new position of the
moved object (relocation target from the world manifest). A localisation counts as correct if it lies within
--tol metres (default 1.2) of the true position.

Failure categories of a mission that did not succeed
  never_localised    no look ever localised the object at its true position (perception / coverage miss)
  localised_unreached the object was localised correctly at least once and an approach failed (controller)
  localised_rejected  localised correctly, approached, but the verification rejected it or the mission ended otherwise
  of the 'localised_rejected' missions, how many had approach looks (tag 'face') that said yes AT the true place
  (these were rejected by the position-consistency test of the verification, not by the VLM)
Also reported: false hits (localisations that are not at the true position, e.g. a similar object) and the
distribution of the localisation error of all hits within 3 m of the true position.

Per-look analysis: for every approach look (tag face / peripheral) taken within 3 m of the true position, the geometry
(object position, robot pose, field of view) tells whether the object was IN VIEW; the table shows what the VLM
answered in each case (this separates verifier errors from navigation / geometry problems).

Usage: python3 analyze_failures.py results/exp_test_r1*_mainA.csv [--worlds_dir DIR] [--tol 1.2] [--hfov 85.2]
       python3 analyze_failures.py --selftest
"""
import csv
import json
import math
import os
import sys
import tempfile
from collections import Counter, defaultdict

WORLDS = os.path.join(os.environ.get("VLMNAV_BASE", "."), "vlm_nav", "worlds")


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def load_rows(paths):
    rows = []
    for p in paths:
        for r in csv.DictReader(open(p)):
            r["src"] = p
            rows.append(r)
    return rows


def analyze(rows, worlds_dir, tol=1.2, hfov=85.2):
    manifests = {}
    out = []
    stats = defaultdict(Counter)
    errs = defaultdict(list)
    looks = defaultdict(Counter)       # (cond, visibility) -> {'yes': n, 'no': n}
    from eval_moondream import object_status
    for r in rows:
        if r["scenario"] != "S2" or r["cond"] not in ("B2", "FULL"):
            continue
        w = r["world"]
        if w not in manifests:
            manifests[w] = json.load(open(os.path.join(worlds_dir, w + ".json")))
        man = manifests[w]
        true_pos = next((rl["to"] for rl in man["relocations"] if rl["id"] == r["target"]), None)
        if true_pos is None:
            continue
        try:
            log = json.loads(r.get("vlm_log", "[]"))
        except Exception:
            log = []
        hits = [e["pos"] for e in log if e.get("yes") and e.get("pos")]
        good = [h for h in hits if dist(h, true_pos) <= tol]
        bad = [h for h in hits if dist(h, true_pos) > tol]
        face_good = [e for e in log if e.get("tag") == "face" and e.get("yes") and e.get("pos")
                     and dist(e["pos"], true_pos) <= tol]
        errs[r["cond"]] += [dist(h, true_pos) for h in hits if dist(h, true_pos) <= 3.0]
        inst = next((i for i in man["instances"] if i["id"] == r["target"]), None)
        if inst is not None:
            tobj = {"id": inst["id"], "x": true_pos[0], "y": true_pos[1], "radius": inst["radius"]}
            for e in log:
                if e.get("tag") in ("face", "peripheral", "sweep") and e.get("robot"):
                    rx, ry, ryaw = e["robot"]
                    if dist((rx, ry), true_pos) <= 3.0:
                        cam = (rx + 0.55 * math.cos(ryaw), ry + 0.55 * math.sin(ryaw), ryaw)
                        st = object_status(cam, hfov, tobj, [])
                        looks[(r["cond"], st)]["yes" if e.get("yes") else "no"] += 1
        near = int(float(r["near"])) == 1
        declared = r["reason"] not in ("NOT_FOUND", "no_candidate")
        success = near and declared
        key = r["cond"]
        stats[key]["missions"] += 1
        stats[key]["false_hits"] += len(bad)
        stats[key]["hits"] += len(hits)
        if success:
            stats[key]["success"] += 1
            continue
        outcomes = r.get("outcomes", "")
        if not good:
            stats[key]["never_localised"] += 1
            cat = "never_localised"
        elif "unreachable" in outcomes.split(">"):
            stats[key]["localised_unreached"] += 1
            cat = "localised_unreached"
        else:
            stats[key]["localised_rejected"] += 1
            cat = "localised_rejected"
            if face_good:
                stats[key]["rejected_despite_face_yes"] += 1
        out.append(f"  {w} ep{r['episode']} {key:<4s} cls={r['cls']:<14s} {cat:<20s} outcomes={outcomes or '-'} "
                   f"correct_hits={len(good)} face_yes_at_truth={len(face_good)} false_hits={len(bad)}")
    lines = [f"{'cond':<6s}{'missions':>9s}{'success':>9s}{'never localised':>17s}{'localised, unreached':>22s}"
             f"{'localised, rejected':>21s}{'(yes at truth)':>16s}{'false hits/mission':>20s}"]
    for key in ("B2", "FULL"):
        st = stats[key]
        if not st["missions"]:
            continue
        lines.append(f"{key:<6s}{st['missions']:>9d}{st['success']:>9d}{st['never_localised']:>17d}"
                     f"{st['localised_unreached']:>22d}{st['localised_rejected']:>21d}{st['rejected_despite_face_yes']:>16d}"
                     f"{st['false_hits'] / st['missions']:>20.2f}")
    lines.append("")
    lines.append("localisation error of hits within 3 m of the true position (metres):")
    for key in ("B2", "FULL"):
        e = sorted(errs[key])
        if e:
            q = lambda p: e[min(len(e) - 1, int(p * len(e)))]
            lines.append(f"  {key:<5s} n={len(e):<4d} median={q(0.5):.2f}  p75={q(0.75):.2f}  p90={q(0.9):.2f}  max={e[-1]:.2f}  "
                         f"share <= 1.0 m: {sum(1 for x in e if x <= 1.0) / len(e):.2f}")
    lines.append("")
    lines.append("approach looks within 3 m of the true position: was the object in view (by geometry) and what did the VLM say?")
    lines.append(f"{'cond':<6s}{'object in view?':<18s}{'looks':>7s}{'VLM yes':>9s}{'VLM no':>8s}{'yes rate':>10s}")
    for key in ("B2", "FULL"):
        for st, name in (("vis", "yes (clearly)"), ("amb", "partly / far"), ("no", "no (out of view)")):
            c = looks[(key, st)]
            n = c["yes"] + c["no"]
            if n:
                lines.append(f"{key:<6s}{name:<18s}{n:>7d}{c['yes']:>9d}{c['no']:>8d}{c['yes'] / n:>10.2f}")
    return "\n".join(lines) + "\n\nfailed missions:\n" + "\n".join(out)


def looks_all(rows, worlds_dir, hfov=85.2):
    """Per-look analysis for BOTH scenarios: nearest true instance of the queried class within 3 m of the robot,
    is it clearly in view (geometry) and what did the VLM answer? Two tables: all looks, and first looks only
    (tag 'face'; the heading-sweep looks are taken only after a first 'no' and bias the yes-rate downwards)."""
    from eval_moondream import object_status
    manifests = {}
    tables = {"all looks": defaultdict(Counter), "first look only (tag face)": defaultdict(Counter)}
    by_class = defaultdict(Counter)
    for r in rows:
        if r["cond"] not in ("B2", "FULL"):
            continue
        w = r["world"]
        if w not in manifests:
            manifests[w] = json.load(open(os.path.join(worlds_dir, w + ".json")))
        man = manifests[w]
        moved = {rl["id"]: rl["to"] for rl in man["relocations"]} if r["scenario"] == "S2" else {}
        inst = []
        for i in man["instances"]:
            if i["class"] != r["cls"]:
                continue
            x, y = moved.get(i["id"], (i["x"], i["y"])) if i["id"] == r["target"] else (i["x"], i["y"])
            inst.append({"id": i["id"], "x": x, "y": y, "radius": i["radius"]})
        try:
            log = json.loads(r.get("vlm_log", "[]"))
        except Exception:
            log = []
        for e in log:
            if e.get("tag") not in ("face", "peripheral", "sweep") or not e.get("robot") or not inst:
                continue
            rx, ry, ryaw = e["robot"]
            o = min(inst, key=lambda t: dist((rx, ry), (t["x"], t["y"])))
            if dist((rx, ry), (o["x"], o["y"])) > 3.0:
                continue
            cam = (rx + 0.55 * math.cos(ryaw), ry + 0.55 * math.sin(ryaw), ryaw)
            st = object_status(cam, hfov, o, [])
            key = (r["scenario"], r["cond"], st)
            tables["all looks"][key]["yes" if e.get("yes") else "no"] += 1
            if e.get("tag") == "face":
                tables["first look only (tag face)"][key]["yes" if e.get("yes") else "no"] += 1
                if st == "vis":
                    by_class[(r["scenario"], r["cls"])]["yes" if e.get("yes") else "no"] += 1
    out = []
    for title, table in tables.items():
        out.append(f"-- {title}")
        out.append(f"{'scen':<5s}{'cond':<6s}{'object in view?':<18s}{'looks':>7s}{'VLM yes':>9s}{'VLM no':>8s}{'yes rate':>10s}")
        for sc in ("S1", "S2"):
            for key in ("B2", "FULL"):
                for st, name in (("vis", "yes (clearly)"), ("amb", "partly / far"), ("no", "no (out of view)")):
                    c = table[(sc, key, st)]
                    n = c["yes"] + c["no"]
                    if n:
                        out.append(f"{sc:<5s}{key:<6s}{name:<18s}{n:>7d}{c['yes']:>9d}{c['no']:>8d}{c['yes'] / n:>10.2f}")
        out.append("")
    out.append("-- first look, object clearly in view, by class (B2 and FULL together)")
    out.append(f"{'scen':<5s}{'class':<18s}{'looks':>7s}{'VLM yes':>9s}{'yes rate':>10s}")
    for (sc, cls) in sorted(by_class):
        c = by_class[(sc, cls)]
        n = c["yes"] + c["no"]
        out.append(f"{sc:<5s}{cls:<18s}{n:>7d}{c['yes']:>9d}{c['yes'] / n:>10.2f}")
    return "\n".join(out)


def looks_by_distance(rows, worlds_dir, hfov=85.2):
    """First looks (tag face) by distance between robot and the nearest true instance: how often is the object clearly in view?"""
    from eval_moondream import object_status
    manifests, bins = {}, [(0.0, 1.25), (1.25, 1.75), (1.75, 2.25), (2.25, 2.75), (2.75, 3.5)]
    table = defaultdict(Counter)
    for r in rows:
        if r["cond"] not in ("B2", "FULL"):
            continue
        w = r["world"]
        if w not in manifests:
            manifests[w] = json.load(open(os.path.join(worlds_dir, w + ".json")))
        man = manifests[w]
        moved = {rl["id"]: rl["to"] for rl in man["relocations"]} if r["scenario"] == "S2" else {}
        inst = []
        for i in man["instances"]:
            if i["class"] != r["cls"]:
                continue
            x, y = moved.get(i["id"], (i["x"], i["y"])) if i["id"] == r["target"] else (i["x"], i["y"])
            inst.append({"id": i["id"], "x": x, "y": y, "radius": i["radius"]})
        try:
            log = json.loads(r.get("vlm_log", "[]"))
        except Exception:
            log = []
        for e in log:
            if e.get("tag") != "face" or not e.get("robot") or not inst:
                continue
            rx, ry, ryaw = e["robot"]
            o = min(inst, key=lambda t: dist((rx, ry), (t["x"], t["y"])))
            d = dist((rx, ry), (o["x"], o["y"]))
            for lo, hi in bins:
                if lo <= d < hi:
                    cam = (rx + 0.55 * math.cos(ryaw), ry + 0.55 * math.sin(ryaw), ryaw)
                    st = object_status(cam, hfov, o, [])
                    c = table[(r["scenario"], lo, hi)]
                    c["looks"] += 1
                    c["vis"] += int(st == "vis")
                    c["yes_vis"] += int(st == "vis" and bool(e.get("yes")))
    lines = [f"{'scen':<5s}{'distance (m)':<14s}{'first looks':>12s}{'clearly in view':>17s}{'share':>7s}{'yes | in view':>15s}"]
    for sc in ("S1", "S2"):
        for lo, hi in bins:
            c = table[(sc, lo, hi)]
            if c["looks"]:
                lines.append(f"{sc:<5s}{f'{lo:.2f}-{hi:.2f}':<14s}{c['looks']:>12d}{c['vis']:>17d}{c['vis'] / c['looks']:>7.2f}"
                             f"{(str(c['yes_vis']) + '/' + str(c['vis'])):>15s}")
    return "\n".join(lines)


def selftest():
    d = tempfile.mkdtemp()
    json.dump({"relocations": [{"id": "cabinet_0", "to": [-1.0, -3.0]}],
               "instances": [{"id": "cabinet_0", "x": 2.0, "y": 2.0, "radius": 0.45, "class": "cabinet"}]},
              open(os.path.join(d, "w.json"), "w"))
    hdr = ["world", "scenario", "episode", "cond", "cls", "target", "near", "reason", "outcomes", "vlm_log"]
    rows = [
        ["w", "S2", "0", "FULL", "cabinet", "cabinet_0", "1", "found_by_search", "not_found>verified",
         json.dumps([{"tag": "face", "yes": 1, "pos": [-0.8, -3.1], "robot": [-1.0, -1.5, -1.57]}])],  # success, object clearly in view
        ["w", "S2", "1", "FULL", "cabinet", "cabinet_0", "0", "NOT_FOUND", "not_found",
         json.dumps([{"tag": "face", "yes": 0, "pos": None, "robot": [-1.0, -1.5, 1.57]}])],           # never localised, robot looks away
        ["w", "S2", "2", "FULL", "cabinet", "cabinet_0", "0", "NOT_FOUND", "not_found>unreachable",
         json.dumps([{"yes": 1, "pos": [-1.1, -3.2]}])],                                    # localised, unreached
        ["w", "S2", "3", "FULL", "cabinet", "cabinet_0", "0", "NOT_FOUND", "not_found>not_found",
         json.dumps([{"tag": "face", "yes": 1, "pos": [-1.0, -2.8]}, {"tag": "scan", "yes": 1, "pos": [3.0, 3.0]}])],  # rejected despite a face-yes + a false hit
    ]
    p = os.path.join(d, "exp.csv")
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(hdr)
        w.writerows(rows)
    txt = analyze(load_rows([p]), d, hfov=85.2)
    assert "never_localised" in txt and "localised_unreached" in txt and "localised_rejected" in txt, txt
    first = [l for l in txt.splitlines() if l.startswith("FULL")][0].split()
    assert first[1:7] == ["4", "1", "1", "1", "1", "1"], first   # missions, success, never, unreached, rejected, yes-at-truth
    assert "localisation error" in txt and "median=" in txt
    assert "approach looks within 3 m" in txt
    vis_line = [l for l in txt.splitlines() if l.startswith("FULL") and "clearly" in l][0].split()
    assert vis_line[-4:] == ["1", "1", "0", "1.00"], vis_line     # one look with the object in view, VLM said yes
    # looks_all: one S2 look with the object clearly in view and answered yes
    rows2 = load_rows([p])
    txt2 = looks_all(rows2, d)
    assert "S2" in txt2 and "yes (clearly)" in txt2 and "first look only" in txt2 and "by class" in txt2, txt2
    assert "first looks" in looks_by_distance(rows2, d)
    print("SELFTEST_OK")


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "--selftest":
        selftest()
    elif a:
        wd, tol, hfov, looks_flag, bydist_flag = WORLDS, 1.2, 85.2, False, False
        paths = []
        i = 0
        while i < len(a):
            if a[i] == "--worlds_dir":
                wd = a[i + 1]; i += 2
            elif a[i] == "--tol":
                tol = float(a[i + 1]); i += 2
            elif a[i] == "--bydist":
                bydist_flag = True; i += 1
            elif a[i] == "--looks":
                looks_flag = True; i += 1
            elif a[i] == "--hfov":
                hfov = float(a[i + 1]); i += 2
            else:
                paths.append(a[i]); i += 1
        rows_all = load_rows(paths)
        print(analyze(rows_all, wd, tol, hfov))
        if bydist_flag:
            print("\n== first looks by distance to the true object")
            print(looks_by_distance(rows_all, wd, hfov))
        if looks_flag:
            print("\n== per-look analysis, both scenarios (nearest true instance of the queried class within 3 m)")
            print(looks_all(rows_all, wd, hfov))
    else:
        print(__doc__)
