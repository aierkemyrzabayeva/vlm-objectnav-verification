#!/usr/bin/env python3
"""
analyze_exp.py - summarise the experiment CSVs written by run_experiment.py.

Metrics
  SR           PRIMARY success: the robot ends within radius + 1.0 m of a true instance of the queried class AND the
               mission declared a goal (did not end with NOT_FOUND). Standing next to an object by chance while
               searching does not count.
  near         raw proximity (without the declaration requirement)
  success      strict: controller reached its goal AND near
  decision_ok  the mission's final goal lies at a true instance (independent of the controller)
Cost: VLM calls, path length, simulated time. Paired comparisons: exact McNemar test on episodes run under
both conditions (same world, scenario, episode).

Usage: python3 analyze_exp.py results/exp_office_s*.csv [--by-class] [--wrong]
       --logs COND prints every mission of that condition with its legs and the VLM log (e.g. --logs FULL)
       --wrong lists missions that declared a goal (verified / found_by_search) which is NOT at a true instance,
       with the VLM log (robot pose, answer, localised position) - for debugging false confirmations
       python3 analyze_exp.py --selftest
"""
import csv
import math
import sys
from collections import Counter, defaultdict

PAIRS = [("FULL", "B0"), ("FULL", "B1"), ("FULL", "B1r"), ("FULL", "B2"), ("B2", "B1r"), ("B1r", "B1"), ("B2", "B0"), ("B1", "B0")]


def wilson(k, n, z=1.96):
    if n == 0:
        return float("nan"), float("nan")
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def mcnemar_exact(b, c):
    """Two-sided exact p-value for discordant pairs b (A only) and c (B only)."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    p = 2.0 * sum(math.comb(n, i) for i in range(k + 1)) / (2.0 ** n)
    return min(1.0, p)


def load(paths):
    rows = []
    for p in paths:
        for r in csv.DictReader(open(p)):
            r["src"] = p
            for k in ("success", "decision_ok", "near", "reached", "vlm_calls", "n_attempts"):
                r[k] = int(float(r[k]))
            r["sr"] = int(r["near"] and r["reason"] not in ("NOT_FOUND", "no_candidate"))
            for k in ("path", "sim_s", "wall_s", "vlm_s"):
                r[k] = float(r[k])
            rows.append(r)
    return rows


def rate(rows, key):
    k = sum(r[key] for r in rows)
    n = len(rows)
    lo, hi = wilson(k, n)
    return f"{k:3d}/{n:<3d} {k / n if n else float('nan'):4.2f} [{lo:.2f},{hi:.2f}]"


def mean(rows, key):
    return sum(r[key] for r in rows) / len(rows) if rows else float("nan")


def analyze(rows, by_class=False):
    out = []
    scenarios = sorted({r["scenario"] for r in rows})
    conds = [c for c in ("B0", "B1", "B1r", "B2", "FULL") if any(r["cond"] == c for r in rows)]
    for sc in scenarios:
        out.append(f"\n===== scenario {sc}")
        out.append(f"{'cond':<6s}{'n':>4s}  {'SR (primary)':<22s}{'raw near':<22s}{'decision ok':<22s}"
                   f"{'VLM calls':>10s}{'path m':>8s}{'sim s':>8s}")
        for c in conds:
            rs = [r for r in rows if r["scenario"] == sc and r["cond"] == c]
            if not rs:
                continue
            out.append(f"{c:<6s}{len(rs):>4d}  {rate(rs, 'sr'):<22s}{rate(rs, 'near'):<22s}{rate(rs, 'decision_ok'):<22s}"
                       f"{mean(rs, 'vlm_calls'):>10.1f}{mean(rs, 'path'):>8.1f}{mean(rs, 'sim_s'):>8.0f}")
        out.append("termination reasons: " + "; ".join(
            f"{c}: {dict(Counter(r['reason'] for r in rows if r['scenario'] == sc and r['cond'] == c))}" for c in conds))

        out.append("paired comparisons (same episodes), metric SR / decision_ok:")
        for a, b in PAIRS:
            ka = {(r["src"], r["episode"]): r for r in rows if r["scenario"] == sc and r["cond"] == a}
            kb = {(r["src"], r["episode"]): r for r in rows if r["scenario"] == sc and r["cond"] == b}
            common = sorted(set(ka) & set(kb))
            if not common:
                continue
            txt = []
            for m in ("sr", "decision_ok"):
                x = sum(1 for k in common if ka[k][m] and not kb[k][m])
                y = sum(1 for k in common if kb[k][m] and not ka[k][m])
                txt.append(f"{m}: {a} only {x}, {b} only {y}, p={mcnemar_exact(x, y):.3f}")
            out.append(f"  {a:>4s} vs {b:<3s} n={len(common):<3d} " + " | ".join(txt))

        if by_class:
            out.append("per class (SR by condition):")
            for cls in sorted({r["cls"] for r in rows if r["scenario"] == sc}):
                cells = []
                for c in conds:
                    rs = [r for r in rows if r["scenario"] == sc and r["cls"] == cls and r["cond"] == c]
                    cells.append(f"{c} {sum(r['sr'] for r in rs)}/{len(rs)}" if rs else f"{c} -")
                out.append(f"  {cls:<18s}" + "   ".join(cells))
    return "\n".join(out)


def wrong_confirms(rows):
    import json
    out = ["\n===== declared goals that are not at a true instance (decision_ok = 0)"]
    n = 0
    for r in rows:
        if r["decision_ok"] == 0 and r["reason"] in ("verified", "found_by_search"):
            n += 1
            out.append(f"{r['scenario']} ep{r['episode']} {r['cond']:<4s} cls={r['cls']} target={r['target']} "
                       f"reason={r['reason']} outcomes={r.get('outcomes', '')} scene_dev={r.get('scene_dev', '?')}")
            try:
                for e in json.loads(r.get("vlm_log", "[]")):
                    out.append(f"      look {e['k']}: yes={e['yes']} robot={e['robot']} localised={e['pos']} frame={e['frame']}")
            except Exception:
                pass
    out.append(f"({n} missions)")
    return "\n".join(out)


def show_logs(rows, cond):
    import json
    out = [f"\n===== mission logs, condition {cond}"]
    for r in rows:
        if r["cond"] != cond:
            continue
        legs = []
        try:
            legs = json.loads(r.get("legs", "[]"))
        except Exception:
            pass
        reasons = Counter(l.get("reason") for l in legs)
        out.append(f"{r['scenario']} ep{r['episode']} cls={r['cls']} -> {r['reason']} outcomes={r.get('outcomes', '')} "
                   f"decision_ok={r['decision_ok']} vlm={r['vlm_calls']} path={r['path']} recoveries={r.get('recoveries', '?')} "
                   f"legs={dict(reasons)} scene_dev={r.get('scene_dev', '?')}")
        try:
            for e in json.loads(r.get("vlm_log", "[]")):
                out.append(f"      look {e['k']:>2} {e.get('tag', ''):<10s} yes={e['yes']} robot={e['robot']} localised={e['pos']}")
        except Exception:
            pass
    return "\n".join(out)


def selftest():
    import os
    import tempfile
    assert abs(mcnemar_exact(0, 0) - 1.0) < 1e-12
    assert abs(mcnemar_exact(5, 0) - 0.0625) < 1e-9          # 2 * (1/32)
    assert abs(mcnemar_exact(3, 3) - 1.0) < 1e-9
    p = os.path.join(tempfile.mkdtemp(), "exp_t.csv")
    hdr = ["world", "scenario", "episode", "cond", "cls", "success", "decision_ok", "near", "reached", "reason",
           "n_attempts", "vlm_calls", "vlm_s", "path", "sim_s", "wall_s"]
    rows = []
    for ep in range(10):
        for cond, near in (("B0", 0), ("FULL", 1 if ep < 8 else 0)):
            rows.append(["w", "S2", ep, cond, "cone", near, near, near, near, "x", 1, 5 if cond == "FULL" else 0,
                         1.0, 10.0, 30.0, 60.0])
    with open(p, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(hdr)
        w.writerows(rows)
    txt = analyze(load([p]), by_class=True)
    assert "FULL vs B0" in txt and "FULL only 8, B0 only 0" in txt and "p=0.008" in txt, txt
    print("SELFTEST_OK")


if __name__ == "__main__":
    args = sys.argv[1:]
    if args and args[0] == "--selftest":
        selftest()
    elif args:
        by_class = "--by-class" in args
        rows = load([a for a in args if not a.startswith("--") and a not in ("FULL", "B0", "B1", "B1r", "B2")])
        print(analyze(rows, by_class))
        if "--wrong" in args:
            print(wrong_confirms(rows))
        if "--logs" in args:
            print(show_logs(rows, args[args.index("--logs") + 1]))
    else:
        print(__doc__)
