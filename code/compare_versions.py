#!/usr/bin/env python3
"""
compare_versions.py - paired comparison of ONE condition between two sets of result files that cover the same episodes
(for example FULL of the revised configuration against FULL with the initial approach geometry).

Missions are paired by (world, scenario, episode). Reported per scenario: SR, decision accuracy, VLM calls, path, simulated
time for both sets, and the discordant pairs with the exact two-sided McNemar p-value.

Usage:
  python3 compare_versions.py --a "results/exp_test2_r3*_v2.csv" --b "results/exp_test2_r3*_v1abl.csv" [--cond FULL]
  python3 compare_versions.py --selftest
"""
import csv
import glob
import math
import sys
import tempfile
import os

NO_GOAL = ("NOT_FOUND", "no_candidate")


def wilson(k, n, z=1.96):
    if n == 0:
        return float("nan"), float("nan")
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def mcnemar_exact(b, c):
    n = b + c
    if n == 0:
        return 1.0
    return min(1.0, 2.0 * sum(math.comb(n, i) for i in range(min(b, c) + 1)) / 2.0 ** n)


def load(pattern, cond):
    rows = {}
    for f in sorted(glob.glob(pattern)):
        for r in csv.DictReader(open(f)):
            if r["cond"] != cond:
                continue
            near = int(float(r["near"]))
            r["sr"] = int(near and r["reason"] not in NO_GOAL)
            r["dec"] = int(float(r["decision_ok"]))
            for k in ("vlm_calls", "path", "sim_s"):
                r[k] = float(r[k])
            rows[(r["world"], r["scenario"], r["episode"])] = r
    return rows


def fmt(k, n):
    lo, hi = wilson(k, n)
    return f"{k:>2d}/{n:<2d} = {k / n if n else float('nan'):.2f} [{lo:.2f},{hi:.2f}]"


def compare(a, b):
    out = []
    keys = sorted(set(a) & set(b))
    out.append(f"paired missions: {len(keys)} (A only: {len(set(a) - set(b))}, B only: {len(set(b) - set(a))})")
    for sc in sorted({k[1] for k in keys}):
        ks = [k for k in keys if k[1] == sc]
        out.append(f"\n== scenario {sc}: {len(ks)} paired episodes")
        out.append(f"{'':<4s}{'SR':<24s}{'decision accuracy':<24s}{'VLM calls':>10s}{'path m':>8s}{'sim s':>8s}")
        for name, d in (("A", a), ("B", b)):
            out.append(f"{name:<4s}{fmt(sum(d[k]['sr'] for k in ks), len(ks)):<24s}{fmt(sum(d[k]['dec'] for k in ks), len(ks)):<24s}"
                       f"{sum(d[k]['vlm_calls'] for k in ks) / len(ks):>10.1f}{sum(d[k]['path'] for k in ks) / len(ks):>8.1f}"
                       f"{sum(d[k]['sim_s'] for k in ks) / len(ks):>8.0f}")
        for m, label in (("sr", "SR"), ("dec", "decision")):
            x = sum(1 for k in ks if a[k][m] and not b[k][m])
            y = sum(1 for k in ks if b[k][m] and not a[k][m])
            out.append(f"  {label:<9s}: A only {x}, B only {y}, exact McNemar p = {mcnemar_exact(x, y):.3f}")
    return "\n".join(out)


def selftest():
    d = tempfile.mkdtemp()
    hdr = ["world", "scenario", "episode", "cond", "near", "reason", "decision_ok", "vlm_calls", "path", "sim_s"]

    def write(name, flags):
        with open(os.path.join(d, name), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(hdr)
            for i, ok in enumerate(flags):
                w.writerow(["w", "S2", str(i), "FULL", int(ok), "verified" if ok else "NOT_FOUND", int(ok), 10, 15.0, 100])
    write("a.csv", [1] * 8 + [0] * 2)             # A succeeds in 8 of 10
    write("b.csv", [1] * 3 + [0] * 7)             # B in 3 of 10 (the same 3)
    a, b = load(os.path.join(d, "a.csv"), "FULL"), load(os.path.join(d, "b.csv"), "FULL")
    txt = compare(a, b)
    assert "A only 5, B only 0" in txt and "p = 0.062" in txt, txt
    assert abs(mcnemar_exact(5, 0) - 0.0625) < 1e-9
    print("SELFTEST_OK")


if __name__ == "__main__":
    args = sys.argv[1:]
    if args and args[0] == "--selftest":
        selftest()
    elif "--a" in args and "--b" in args:
        cond = args[args.index("--cond") + 1] if "--cond" in args else "FULL"
        print(compare(load(args[args.index("--a") + 1], cond), load(args[args.index("--b") + 1], cond)))
    else:
        print(__doc__)
