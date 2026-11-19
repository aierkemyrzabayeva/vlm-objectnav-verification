#!/usr/bin/env python3
"""
selftest_check_paper_numbers.py - tests check_paper_numbers.py on SYNTHETIC result files that are constructed to give exactly the numbers printed in the paper
(not experimental data). The checker must accept them and must report a difference when one value is changed.
Usage:  python3 tools/selftest_check_paper_numbers.py
"""
import csv
import importlib.util
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("chk", os.path.join(HERE, "check_paper_numbers.py"))
chk = importlib.util.module_from_spec(spec)
spec.loader.exec_module(chk)

W34 = [32, 33, 34, 35, 36, 41, 42, 43, 44, 45]
W5 = [51, 52, 53, 54, 55]
NAV_FIELDS = ["world", "scenario", "episode", "cond", "near", "reason", "decision_ok", "vlm_calls", "path", "sim_s"]
R = range


def row(world, sc, i, cond, sr, dec, calls, path, tim):
    return {"world": f"test2_r{world}", "scenario": sc, "episode": i % 6, "cond": cond, "near": int(sr), "reason": "verified" if sr else "NOT_FOUND",
            "decision_ok": int(dec), "vlm_calls": calls, "path": path, "sim_s": tim}


def write(path, fields, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(rows)


def build(d):
    # exact sets of successful episodes (indices 0..59) that realise the discordant counts printed in the paper
    SR = {"so": set(R(0, 48)), "v1": set(R(0, 34)) | set(R(48, 54)), "v2": set(R(0, 42)) | set(R(48, 52)), "sw": set(R(0, 33)) | set(R(34, 41))}
    DEC = {"so": set(R(0, 50)), "v1": set(R(0, 36)) | {50, 51, 52}, "sw": set(R(0, 41)), "v2": set(R(0, 46))}
    arm = {"v1abl": "v1", "sweeponly": "sw", "standoffonly": "so", "v2": "v2"}
    # V2 series with all five conditions
    for wi, w in enumerate(W34):
        rows = []
        for (sc, cond), (sr, dec, calls, path, tim) in chk.NAV_V2.items():
            for e in range(6):
                i = wi * 6 + e
                if (sc, cond) == ("S2", "FULL"):
                    s, dd = i in SR["v2"], i in DEC["v2"]
                else:
                    s, dd = i < sr[0], i < dec[0]
                rows.append(row(w, sc, i, cond, s, dd, calls, path, tim))
        write(os.path.join(d, f"exp_test2_r{w}_v2.csv"), NAV_FIELDS, rows)
    # ablation arms (condition FULL only)
    for name, a in arm.items():
        if name == "v2":
            continue
        sr, dec, calls, path, tim, s1 = chk.ABL[name]
        for wi, w in enumerate(W34):
            rows = []
            for e in range(6):
                i = wi * 6 + e
                rows.append(row(w, "S2", i, "FULL", i in SR[a], i in DEC[a], calls, path, tim))
                rows.append(row(w, "S1", i, "FULL", i < s1[0], True, 4.0, 6.0, 40))
            write(os.path.join(d, f"exp_test2_r{w}_{name}.csv"), NAV_FIELDS, rows)
    # confirmation on fresh worlds
    CS = {"so": (set(R(0, 20)), set(R(0, 25)), set(R(0, 28))), "v1": (set(R(0, 7)) | {20, 21, 22}, set(R(0, 9)) | {25}, set(R(0, 23)) | {28})}
    for name, a in (("c_v1abl", "v1"), ("c_standoffonly", "so")):
        sr, dec, calls, path, tim, s1 = chk.ABL[name]
        for wi, w in enumerate(W5):
            rows = []
            for e in range(6):
                i = wi * 6 + e
                rows.append(row(w, "S2", i, "FULL", i in CS[a][0], i in CS[a][1], calls, path, tim))
                rows.append(row(w, "S1", i, "FULL", i in CS[a][2], True, 4.0, 6.0, 40))
            write(os.path.join(d, f"exp_test2_r{w}_{name[2:]}.csv"), NAV_FIELDS, rows)
    # verification study
    vrows = []
    n = 0
    for pr, ((pk, pn), (rk, rn)) in chk.SINGLE.items():
        for j in range(pn):
            vrows.append({"frame": f"f{n}", "condition": "present", "target": "x", "query_cls": "x", "kind": "target", "label": "pos", "prompt": pr, "answer": "", "yes": int(j < pk), "latency": 0}); n += 1
        for j in range(rn):
            vrows.append({"frame": f"f{n}", "condition": "moved", "target": "x", "query_cls": "x", "kind": "target", "label": "neg", "prompt": pr, "answer": "", "yes": int(j < rk), "latency": 0}); n += 1
        for lab in ("amb", "pos"):   # a removed-object frame with another instance in view must not be counted
            vrows.append({"frame": f"f{n}", "condition": "moved", "target": "x", "query_cls": "x", "kind": "target", "label": lab, "prompt": pr, "answer": "", "yes": 1, "latency": 0}); n += 1
        vrows.append({"frame": f"f{n}", "condition": "moved", "target": "x", "query_cls": "y", "kind": "absent", "label": "neg", "prompt": pr, "answer": "", "yes": 1, "latency": 0}); n += 1
    write(os.path.join(d, "verif_office_s1.csv"), ["frame", "condition", "target", "query_cls", "kind", "label", "prompt", "answer", "yes", "latency"], vrows)


def run(d):
    p = subprocess.run([sys.executable, os.path.join(HERE, "check_paper_numbers.py"), d], capture_output=True, text=True)
    return p.returncode, p.stdout


def main():
    with tempfile.TemporaryDirectory() as d:
        build(d)
        code, out = run(d)
        last = [l for l in out.splitlines() if "checks agree" in l]
        assert code == 0 and "DIFF" not in out, "the checker rejected data that match the paper:\n" + out
        print("synthetic data that match the paper: accepted;", last[0])
        # change one episode of the main result (S2 FULL, V2): the checker must notice
        f = os.path.join(d, "exp_test2_r32_v2.csv")
        rows = list(csv.DictReader(open(f)))
        for r in rows:
            if r["scenario"] == "S2" and r["cond"] == "FULL" and r["near"] == "1":
                r["near"] = "0"; r["reason"] = "NOT_FOUND"; break
        write(f, NAV_FIELDS, rows)
        code, out = run(d)
        assert code == 1 and "DIFF" in out, "the checker did not notice a changed episode"
        print("one changed episode: detected;", [l for l in out.splitlines() if "checks agree" in l][0])
        # change one answer of the single-look table
        build(d)
        f = os.path.join(d, "verif_office_s1.csv")
        rows = list(csv.DictReader(open(f)))
        for r in rows:
            if r["prompt"] == "A" and r["condition"] == "present" and r["yes"] == "1":
                r["yes"] = "0"; break
        write(f, ["frame", "condition", "target", "query_cls", "kind", "label", "prompt", "answer", "yes", "latency"], rows)
        code, out = run(d)
        assert code == 1 and "DIFF" in out, "the checker did not notice a changed answer"
        print("one changed answer: detected")
    print("SELFTEST_OK")


if __name__ == "__main__":
    main()
