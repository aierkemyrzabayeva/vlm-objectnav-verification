#!/usr/bin/env python3
"""make_trace.py - draws one relocation mission (S2) from the logs: robot poses of the VLM looks (vlm_log),
localised hits and the leg targets, for two arms of the same episode. Usage: python3 make_trace.py OUTDIR"""
import json, math, sys
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle

OUT = sys.argv[1] if len(sys.argv) > 1 else ".."
plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "DejaVu Serif"], "mathtext.fontset": "stix",
                     "font.size": 7, "axes.labelsize": 7, "axes.titlesize": 7.2, "legend.fontsize": 6.3, "xtick.labelsize": 6.5,
                     "ytick.labelsize": 6.5, "axes.linewidth": 0.6, "pdf.fonttype": 42, "savefig.bbox": "tight", "savefig.pad_inches": 0.03})
G, R = "#009E73", "#D55E00"
W = json.load(open("data/test2_r51.json"))
abbr = {"cabinet": "CAB", "cardboard box": "BOX", "fire hydrant": "HYD", "person": "PER", "postbox": "PST", "table": "TAB", "trash can": "BIN", "tree": "TRE"}
mv = {r["id"]: r for r in W["relocations"]}
EP = 4
arms = [("(a) V1 geometry (standoff 1.5 m, no sweep)", "data/exp_test2_r51_v1abl.csv"),
        ("(b) standoff 2.5 m, no sweep", "data/exp_test2_r51_standoffonly.csv")]
half = W["room_size"] / 2.0
fig, axs = plt.subplots(1, 2, figsize=(7.0, 3.5))
for ax, (title, f) in zip(axs, arms):
    d = pd.read_csv(f)
    r = d[(d.scenario == "S2") & (d.episode == EP)].iloc[0]
    m = mv[r.target]
    ax.set_xlim(-half - 0.2, half + 0.2); ax.set_ylim(-half - 0.2, half + 0.2); ax.set_aspect("equal")
    ax.add_patch(Rectangle((-half, -half), 2 * half, 2 * half, fill=False, lw=1.2, ec="black"))
    ax.tick_params(length=2); ax.set_xlabel("x (m)")
    for i in W["instances"]:
        if i["id"] == r.target:
            continue
        ax.add_patch(Circle((i["x"], i["y"]), i["radius"], fc="#dddddd", ec="#999999", lw=0.4, zorder=1))
        ax.text(i["x"], i["y"], abbr[i["class"]], ha="center", va="center", fontsize=4.8, color="#666666", zorder=2)
    # moved object: old place (dashed) and true new place (filled)
    rad = [i for i in W["instances"] if i["id"] == r.target][0]["radius"]
    ax.add_patch(Circle(m["from"], rad, fc="none", ec="#CC79A7", lw=1.0, ls="--", zorder=3))
    ax.plot(*m["from"], marker="x", ms=4.5, mew=1.0, color="black", zorder=6)
    ax.add_patch(Circle(m["to"], rad, fc="#CC79A7", ec="black", lw=0.8, zorder=3))
    ax.text(m["to"][0], m["to"][1], "PST", ha="center", va="center", fontsize=4.8, zorder=4)
    # leg targets (planned way-points), consecutive duplicates removed
    legs = json.loads(r.legs)
    pts = [(r.sx, r.sy)]
    for l in legs:
        p = tuple(l["target"])
        if math.dist(p, pts[-1]) > 0.3:
            pts.append(p)
    ax.plot([p[0] for p in pts], [p[1] for p in pts], ls=":", lw=0.7, color="#777777", zorder=2)
    ax.plot(r.sx, r.sy, marker="*", ms=7, color="black", zorder=7)
    # looks
    for L in json.loads(r.vlm_log):
        x, y, yaw = L["robot"]
        col = G if L["yes"] else R
        lw = 1.8 if L["tag"] == "face" else 0.9
        ax.annotate("", xy=(x + 0.55 * math.cos(yaw), y + 0.55 * math.sin(yaw)), xytext=(x, y),
                    arrowprops=dict(arrowstyle="-|>", lw=lw, color=col, mutation_scale=6), zorder=8)
        ax.plot(x, y, marker="o", ms=2.6, color=col, zorder=8)
        ax.text(x - 0.28 * math.sin(yaw) - 0.1, y + 0.28 * math.cos(yaw) + 0.05, str(L["k"]), fontsize=5.6, color=col, zorder=9)
        if L["pos"]:
            ax.plot([x, L["pos"][0]], [y, L["pos"][1]], lw=0.4, color=col, ls="-", alpha=0.5, zorder=4)
            ax.plot(*L["pos"], marker="D", ms=3.4, mfc="white", mec=col, mew=0.9, zorder=9)
    ok = "success" if int(r.success) else "no declared goal"
    ax.set_title(f"{title}\n{ok}; {len(json.loads(r.vlm_log))} looks, {int(r.vlm_calls)} VLM calls, {r.path:.1f} m", fontsize=6.8)
axs[0].set_ylabel("y (m)")
h = [Line2D([], [], color=G, lw=1.4, marker=">", ms=3, label="look, VLM answered yes"),
     Line2D([], [], color=R, lw=1.4, marker=">", ms=3, label="look, VLM answered no"),
     Line2D([], [], color="black", lw=1.8, label="look at a candidate (thick) / scan look (thin)"),
     Line2D([], [], marker="D", mfc="white", mec="black", ls="none", ms=3.4, label="localised position of a yes"),
     Line2D([], [], color="#777777", ls=":", lw=0.9, label="leg targets from the start (star)"),
     Line2D([], [], marker="x", color="black", ls="none", ms=4, label="remembered (stale) place of the postbox")]
fig.legend(handles=h, loc="lower center", ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.12), columnspacing=1.2)
fig.savefig(f"{OUT}/fig_trace.pdf"); fig.savefig("/tmp/fig_trace.png", dpi=170)
print("saved")
