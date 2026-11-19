#!/usr/bin/env python3
"""
check_paper_numbers.py - recompute the headline numbers of the paper from the released CSV files and compare them with the values printed in the paper.

This is an independent re-implementation (standard library only; it does not call analyze_exp.py), so it also checks the analysis scripts.

Usage:  python3 tools/check_paper_numbers.py PATH/TO/results      (the results/ folder of the unpacked data archive)
Output: one line per check: OK or DIFF, the value in the paper and the recomputed value; the exit code is 0 if all checks agree.

Definitions (as in the paper and DATA.md):
  SR (primary)       the robot ends within radius + 1.0 m of a true instance (column near) AND the mission declared a goal (reason is not NOT_FOUND / no_candidate)
  decision accuracy  column decision_ok
  paired tests       exact two-sided McNemar test on identical episodes (world, scenario, episode, condition FULL)
  single look        Moondream2, target class, frames with the object present (label pos) or removed (label neg)
"""
import csv
import glob
import math
import os
import sys

# values printed in the tables of the paper (revised configuration V2, ablation, single look), parsed from the manuscript on 18 November 2026
NAV_V2 = {('S1', 'B0'): ((55, 60), (60, 60), 0.0, 4.4, 20), ('S1', 'B1'): ((57, 60), (60, 60), 0.0, 3.3, 17), ('S1', 'B1r'): ((58, 60), (60, 60), 0.0, 3.7, 26), ('S1', 'B2'): ((56, 60), (55, 60), 4.2, 7.3, 44), ('S1', 'FULL'): ((55, 60), (54, 60), 4.2, 6.6, 42), ('S2', 'B0'): ((1, 60), (0, 60), 0.0, 4.4, 15), ('S2', 'B1'): ((0, 60), (0, 60), 0.0, 4.4, 14), ('S2', 'B1r'): ((0, 60), (0, 60), 0.0, 5.1, 23), ('S2', 'B2'): ((50, 60), (52, 60), 12.3, 15.4, 75), ('S2', 'FULL'): ((46, 60), (46, 60), 12.6, 13.6, 70)}
SINGLE = {'A': ((77, 118), (5, 116)), 'Ac': ((87, 118), (5, 117)), 'B': ((86, 118), (12, 116)), 'C': ((70, 118), (5, 116)), 'D': ((101, 118), (13, 116)), 'Dc': ((102, 118), (11, 117))}
ABL = {'v1abl': ((40, 60), (39, 60), 12.7, 18.8, 90, (55, 60)), 'sweeponly': ((40, 60), (41, 60), 14.6, 16.5, 85, (57, 60)), 'standoffonly': ((48, 60), (50, 60), 9.5, 14.2, 74, (55, 60)), 'v2': ((46, 60), (46, 60), 12.6, 13.6, 70, (55, 60)), 'c_v1abl': ((10, 30), (10, 30), 13.9, 24.6, 117, (24, 30)), 'c_standoffonly': ((20, 30), (25, 30), 8.8, 16.9, 83, (28, 30))}


# paired discordant counts printed in the paper: (a-only, b-only, p as printed)
PAIRS = [
    ("ablation S2 SR: standoff only vs V1 geometry", ("standoffonly", "v1abl"), "S2", "sr", 14, 6, "0.12"),
    ("ablation S2 decision: standoff only vs V1 geometry", ("standoffonly", "v1abl"), "S2", "dec", 14, 3, "0.013"),
    ("ablation S2 SR: sweep only vs V1 geometry", ("sweeponly", "v1abl"), "S2", "sr", 7, 7, "1"),
    ("ablation S2 SR: V2 vs standoff only", ("v2", "standoffonly"), "S2", "sr", 4, 6, "0.75"),
]
CONFIRM = [
    ("confirmation S2 SR: standoff only vs V1 geometry", "S2", "sr", 13, 3, "0.021"),
    ("confirmation S2 decision: standoff only vs V1 geometry", "S2", "dec", 16, 1, "<0.001"),
    ("confirmation S1 SR: standoff only vs V1 geometry", "S1", "sr", 5, 1, "0.22"),
]


def wilson(k, n, z=1.96):
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return c - h, c + h


def mcnemar(b, c):
    n = b + c
    return 1.0 if n == 0 else min(1.0, 2.0 * sum(math.comb(n, i) for i in range(min(b, c) + 1)) / 2.0 ** n)


def p_ok(p, printed):
    if printed.startswith("<"):
        return p < float(printed[1:])
    return abs(p - float(printed)) <= (0.0065 if float(printed) >= 0.1 else 0.0006)


def load_nav(rdir, pattern):
    rows = []
    for f in sorted(glob.glob(os.path.join(rdir, pattern))):
        for r in csv.DictReader(open(f)):
            r["sr"] = int(int(r["near"]) == 1 and r["reason"] not in ("NOT_FOUND", "no_candidate"))
            r["dec"] = int(r["decision_ok"])
            r["calls"] = float(r["vlm_calls"]); r["pathm"] = float(r["path"]); r["sims"] = float(r["sim_s"])
            rows.append(r)
    return rows


def main():
    rdir = sys.argv[1] if len(sys.argv) > 1 else "results"
    res = []   # (ok, text)

    def check(ok, name, paper, got):
        res.append(bool(ok))
        print(f"{'OK  ' if ok else 'DIFF'}  {name:62s} paper: {paper:<26s} recomputed: {got}")

    # ---------- 1. navigation results V2 (worlds 32-36 and 41-45) ----------
    v2 = load_nav(rdir, "exp_test2_r[34]*_v2.csv")
    print(f"\n== Navigation, revised configuration V2: {len(v2)} rows from exp_test2_r[34]*_v2.csv")
    for (sc, cond), (sr, dec, calls, path, tim) in NAV_V2.items():
        cell = [r for r in v2 if r["scenario"] == sc and r["cond"] == cond]
        if not cell:
            print(f"SKIP  {sc} {cond}: no rows"); continue
        n = len(cell)
        k_sr = sum(r["sr"] for r in cell); k_dec = sum(r["dec"] for r in cell)
        m_calls = sum(r["calls"] for r in cell) / n; m_path = sum(r["pathm"] for r in cell) / n; m_time = sum(r["sims"] for r in cell) / n
        check((k_sr, n) == sr, f"{sc} {cond} SR", f"{sr[0]}/{sr[1]}", f"{k_sr}/{n}")
        check((k_dec, n) == dec, f"{sc} {cond} decision accuracy", f"{dec[0]}/{dec[1]}", f"{k_dec}/{n}")
        check(abs(m_calls - calls) <= 0.051, f"{sc} {cond} VLM calls", f"{calls}", f"{m_calls:.2f}")
        check(abs(m_path - path) <= 0.051, f"{sc} {cond} path (m)", f"{path}", f"{m_path:.2f}")
        check(abs(m_time - tim) <= 0.51, f"{sc} {cond} simulated time (s)", f"{tim}", f"{m_time:.1f}")
    for (sc, cond) in (("S2", "FULL"), ("S2", "B2")):
        cell = [r for r in v2 if r["scenario"] == sc and r["cond"] == cond]
        if cell:
            k = sum(r["sr"] for r in cell); lo, hi = wilson(k, len(cell))
            print(f"      {sc} {cond}: SR {k}/{len(cell)} = {k/len(cell):.2f}, Wilson 95% interval [{lo:.2f}, {hi:.2f}]")

    # ---------- 2. ablation of the approach geometry ----------
    series = {"v1abl": "exp_test2_r[34]*_v1abl.csv", "sweeponly": "exp_test2_r[34]*_sweeponly.csv", "standoffonly": "exp_test2_r[34]*_standoffonly.csv",
              "v2": "exp_test2_r[34]*_v2.csv", "c_v1abl": "exp_test2_r5*_v1abl.csv", "c_standoffonly": "exp_test2_r5*_standoffonly.csv"}
    data = {k: [r for r in load_nav(rdir, p) if r["cond"] == "FULL"] for k, p in series.items()}
    print("\n== Ablation of the approach geometry (condition FULL)")
    for key, (sr, dec, calls, path, tim, s1) in ABL.items():
        rows = data[key]
        s2 = [r for r in rows if r["scenario"] == "S2"]; s1r = [r for r in rows if r["scenario"] == "S1"]
        if not s2:
            print(f"SKIP  {key}: no rows"); continue
        n = len(s2)
        check((sum(r["sr"] for r in s2), n) == sr, f"{key} S2 SR", f"{sr[0]}/{sr[1]}", f"{sum(r['sr'] for r in s2)}/{n}")
        check((sum(r["dec"] for r in s2), n) == dec, f"{key} S2 decision accuracy", f"{dec[0]}/{dec[1]}", f"{sum(r['dec'] for r in s2)}/{n}")
        check(abs(sum(r["calls"] for r in s2) / n - calls) <= 0.051, f"{key} S2 VLM calls", f"{calls}", f"{sum(r['calls'] for r in s2) / n:.2f}")
        check(abs(sum(r["pathm"] for r in s2) / n - path) <= 0.051, f"{key} S2 path (m)", f"{path}", f"{sum(r['pathm'] for r in s2) / n:.2f}")
        check(abs(sum(r["sims"] for r in s2) / n - tim) <= 0.51, f"{key} S2 simulated time (s)", f"{tim}", f"{sum(r['sims'] for r in s2) / n:.1f}")
        check((sum(r["sr"] for r in s1r), len(s1r)) == s1, f"{key} S1 SR", f"{s1[0]}/{s1[1]}", f"{sum(r['sr'] for r in s1r)}/{len(s1r)}")

    def paired(a_rows, b_rows, sc, field):
        ka = {(r["world"], r["scenario"], r["episode"]): r[field] for r in a_rows if r["scenario"] == sc}
        kb = {(r["world"], r["scenario"], r["episode"]): r[field] for r in b_rows if r["scenario"] == sc}
        common = ka.keys() & kb.keys()
        return (sum(1 for k in common if ka[k] and not kb[k]), sum(1 for k in common if kb[k] and not ka[k]), len(common))

    print("\n== Paired exact McNemar tests")
    for name, (a, b), sc, field, pa, pb, pp in PAIRS:
        x, y, n = paired(data[a], data[b], sc, field)
        if n == 0:
            print(f"SKIP  {name}: no paired episodes"); continue
        p = mcnemar(x, y)
        check((x, y) == (pa, pb) and p_ok(p, pp), name, f"{pa} vs {pb}, p={pp}", f"{x} vs {y}, p={p:.4f} (n={n})")
    for name, sc, field, pa, pb, pp in CONFIRM:
        x, y, n = paired(data["c_standoffonly"], data["c_v1abl"], sc, field)
        if n == 0:
            print(f"SKIP  {name}: no paired episodes"); continue
        p = mcnemar(x, y)
        check((x, y) == (pa, pb) and p_ok(p, pp), name, f"{pa} vs {pb}, p={pp}", f"{x} vs {y}, p={p:.4f} (n={n})")
    for name, field, pa, pb, pp in (("pooled 90 episodes S2 SR: standoff only vs V1 geometry", "sr", 27, 9, "0.004"),
                                    ("pooled 90 episodes S2 decision: standoff only vs V1 geometry", "dec", 30, 4, "<0.00001")):
        x1, y1, n1 = paired(data["standoffonly"], data["v1abl"], "S2", field)
        x2, y2, n2 = paired(data["c_standoffonly"], data["c_v1abl"], "S2", field)
        if n1 + n2 == 0:
            print(f"SKIP  {name}: no paired episodes"); continue
        p = mcnemar(x1 + x2, y1 + y2)
        check((x1 + x2, y1 + y2) == (pa, pb) and p_ok(p, pp), name, f"{pa} vs {pb}, p={pp}", f"{x1 + x2} vs {y1 + y2}, p={p:.6f} (n={n1 + n2})")

    # ---------- 3. single look of the verifier (Moondream2) ----------
    rows = []
    for f in sorted(glob.glob(os.path.join(rdir, "verif_office_s*.csv"))):
        rows += list(csv.DictReader(open(f)))
    print(f"\n== Single look at the approach pose: {len(rows)} rows from verif_office_s*.csv")
    for pr, ((pk, pn), (rk, rn)) in SINGLE.items():
        pres = [r for r in rows if r["prompt"] == pr and r["kind"] == "target" and r["condition"] == "present" and r["label"] == "pos"]
        remo = [r for r in rows if r["prompt"] == pr and r["kind"] == "target" and r["condition"] == "moved" and r["label"] == "neg"]
        if not pres:
            print(f"SKIP  prompt {pr}: no rows"); continue
        gp = (sum(int(r["yes"]) for r in pres), len(pres)); gr = (sum(int(r["yes"]) for r in remo), len(remo))
        check(gp == (pk, pn), f"prompt {pr}: object present, yes", f"{pk}/{pn}", f"{gp[0]}/{gp[1]}")
        check(gr == (rk, rn), f"prompt {pr}: object removed, yes", f"{rk}/{rn}", f"{gr[0]}/{gr[1]}")

    ok = sum(res)
    print(f"\n{ok} of {len(res)} checks agree with the paper" + ("" if ok == len(res) else f"; {len(res) - ok} DIFFER (see the DIFF lines)"))
    sys.exit(0 if ok == len(res) and res else 1)


if __name__ == "__main__":
    main()
