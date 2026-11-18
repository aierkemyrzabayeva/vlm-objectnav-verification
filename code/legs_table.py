#!/usr/bin/env python3
"""legs_table.py - controller legs per condition: python3 legs_table.py "results/exp_test2_r3*_v2.csv" """
import csv, glob, json, sys
from collections import Counter, defaultdict

rows = []
for f in sorted(glob.glob(sys.argv[1])):
    rows += list(csv.DictReader(open(f)))
agg, n = defaultdict(Counter), Counter()
for r in rows:
    k = (r["scenario"], r["cond"])
    n[k] += 1
    try:
        for leg in json.loads(r["legs"]):
            agg[k][leg["reason"]] += 1
        agg[k]["recoveries"] += int(float(r.get("recoveries", 0)))
    except Exception:
        pass
print(f"{'scen':<5s}{'cond':<6s}{'missions':>9s}{'legs/mission':>14s}{'goal':>7s}{'stuck':>7s}{'collision':>10s}{'max_steps':>10s}{'recov/mission':>14s}")
for k in sorted(n):
    c = agg[k]
    legs = c["goal"] + c["stuck"] + c["collision"] + c["max_steps"]
    print(f"{k[0]:<5s}{k[1]:<6s}{n[k]:>9d}{legs / n[k]:>14.1f}{c['goal']:>7d}{c['stuck']:>7d}{c['collision']:>10d}{c['max_steps']:>10d}{c['recoveries'] / n[k]:>14.1f}")
