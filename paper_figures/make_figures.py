#!/usr/bin/env python3
"""
make_figures.py - builds the data figures of the paper as vector PDFs (and PNG previews).
All numbers are the ones reported in the tables of the paper (counts k/n); Wilson 95% intervals are computed here.
The world layout is drawn from the manifest written by gen_world.py (seed 32, catalogue "reliable").

Usage: python3 make_figures.py OUTDIR MANIFEST.json
"""
import json
import math
import random
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Rectangle

OUT = sys.argv[1] if len(sys.argv) > 1 else "."
MAN = sys.argv[2] if len(sys.argv) > 2 else "test2_r32.json"

plt.rcParams.update({
    "font.family": "serif", "font.serif": ["STIXGeneral", "DejaVu Serif"], "mathtext.fontset": "stix",
    "font.size": 7, "axes.labelsize": 7, "axes.titlesize": 7.5, "legend.fontsize": 6.3,
    "xtick.labelsize": 6.5, "ytick.labelsize": 6.5, "axes.linewidth": 0.6, "pdf.fonttype": 42,
    "hatch.linewidth": 0.45, "xtick.major.width": 0.5, "ytick.major.width": 0.5, "savefig.bbox": "tight", "savefig.pad_inches": 0.03,
})
OI = {"blue": "#0072B2", "orange": "#E69F00", "green": "#009E73", "vermillion": "#D55E00",
      "purple": "#CC79A7", "sky": "#56B4E9", "yellow": "#F0E442", "gray": "#8c8c8c"}


def wilson(k, n, z=1.96):
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def err(k, n):
    lo, hi = wilson(k, n)
    p = k / n
    return max(0.0, p - lo), max(0.0, hi - p)


def save(fig, name):
    fig.savefig(f"{OUT}/{name}.pdf")
    fig.savefig(f"/tmp/{name}.png", dpi=170)
    plt.close(fig)
    print("saved", name)


# ------------------------------------------------------------------ Fig. world layout
def fig_world():
    m = json.load(open(MAN))
    half = m["room_size"] / 2.0
    classes = sorted({i["class"] for i in m["instances"]})
    abbr = {"cabinet": "CAB", "cardboard box": "BOX", "fire hydrant": "HYD", "person": "PER", "postbox": "PST",
            "table": "TAB", "trash can": "BIN", "tree": "TRE"}
    cols = dict(zip(classes, [OI["blue"], OI["orange"], OI["green"], OI["vermillion"], OI["purple"], OI["sky"],
                              OI["yellow"], OI["gray"]]))
    dup = {i["class"] for i in m["instances"] if sum(1 for j in m["instances"] if j["class"] == i["class"]) > 1}
    moved = {r["id"]: r for r in m["relocations"]}
    rng = random.Random(3)

    fig, axs = plt.subplots(1, 2, figsize=(7.0, 3.3))
    for ax in axs:
        ax.set_xlim(-half - 0.2, half + 0.2)
        ax.set_ylim(-half - 0.2, half + 0.2)
        ax.set_aspect("equal")
        ax.add_patch(Rectangle((-half, -half), 2 * half, 2 * half, fill=False, lw=1.2, ec="black"))
        ax.set_xlabel("x (m)")
        ax.plot([0], [0], marker="*", ms=7, color="black", ls="none", zorder=6)
        ax.tick_params(length=2)
    axs[0].set_ylabel("y (m)")

    # (a) S1: duplicated classes and the (noisy) memory
    ax = axs[0]
    ax.set_title("(a) Memory built from the manifest (S1: duplicates)")
    for i in m["instances"]:
        ax.add_patch(Circle((i["x"], i["y"]), i["radius"], fc=cols[i["class"]], ec="black",
                            lw=1.6 if i["class"] in dup else 0.5, alpha=0.9, zorder=3))
        ax.text(i["x"], i["y"], abbr[i["class"]], ha="center", va="center", fontsize=5.2, zorder=5)
        for _ in range(3):
            dx = max(-0.5, min(0.5, rng.gauss(0, 0.2)))
            dy = max(-0.5, min(0.5, rng.gauss(0, 0.2)))
            ax.plot(i["x"] + dx, i["y"] + dy, marker="x", ms=2.4, mew=0.5, color="black", zorder=4)

    # (b) S2: relocation
    ax = axs[1]
    ax.set_title("(b) Relocation (S2): the memory is stale")
    for i in m["instances"]:
        if i["id"] in moved:
            r = moved[i["id"]]
            fx, fy = r["from"]
            tx, ty = r["to"]
            ax.add_patch(Circle((fx, fy), i["radius"], fc="none", ec=cols[i["class"]], lw=1.0, ls="--", zorder=3))
            ax.plot(fx, fy, marker="x", ms=4.5, mew=1.0, color="black", zorder=6)
            ax.add_patch(Circle((tx, ty), i["radius"], fc=cols[i["class"]], ec="black", lw=0.9, alpha=0.95, zorder=4))
            ax.text(tx, ty, abbr[i["class"]], ha="center", va="center", fontsize=5.2, zorder=5)
            ax.annotate("", xy=(tx, ty), xytext=(fx, fy), arrowprops=dict(arrowstyle="->", lw=0.9, color="black",
                                                                              shrinkA=7, shrinkB=7), zorder=5)
        else:
            ax.add_patch(Circle((i["x"], i["y"]), i["radius"], fc=cols[i["class"]], ec="black", lw=0.4, alpha=0.35,
                                zorder=2))
            ax.text(i["x"], i["y"], abbr[i["class"]], ha="center", va="center", fontsize=5.0, color="#444444",
                    zorder=3)

    handles = [Patch(fc=cols[c], ec="black", lw=0.5, label=f"{abbr[c]} {c}") for c in classes]
    handles += [Line2D([], [], marker="*", color="black", ls="none", ms=6, label="robot start (nominal)"),
                Line2D([], [], marker="x", color="black", ls="none", ms=4, label="remembered position (a) / stale place (b)"),
                Patch(fc="none", ec="black", lw=1.6, label="duplicated class")]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False, bbox_to_anchor=(0.5, -0.12),
               columnspacing=1.2, handlelength=1.4)
    save(fig, "fig_world")


# ------------------------------------------------------------------ Fig. main results
def fig_results():
    conds = ["B0", "B1", "B1r", "B2", "FULL"]
    data = {"S1": {"sr": [55, 57, 58, 56, 55], "dec": [60, 60, 60, 55, 54]},
            "S2": {"sr": [1, 0, 0, 50, 46], "dec": [0, 0, 0, 52, 46]}}
    colors = [OI["gray"], OI["gray"], OI["gray"], OI["sky"], OI["blue"]]
    titles = {"S1": "(a) S1: duplicates, scene unchanged", "S2": "(b) S2: an object has moved"}
    fig, axs = plt.subplots(1, 2, figsize=(7.0, 2.45), sharey=True)
    n = 60
    w = 0.36
    for ax, sc in zip(axs, ("S1", "S2")):
        for j, c in enumerate(conds):
            for off, key, hatch in ((-w / 2, "sr", None), (w / 2, "dec", "////")):
                k = data[sc][key][j]
                e = err(k, n)
                ax.bar(j + off, k / n, w, color=colors[j], edgecolor="black", lw=0.5, hatch=hatch, zorder=3)
                ax.errorbar(j + off, k / n, yerr=[[e[0]], [e[1]]], fmt="none", ecolor="black", elinewidth=0.6,
                            capsize=1.6, zorder=4)
        ax.set_xticks(range(len(conds)))
        ax.set_xticklabels(conds)
        ax.set_ylim(0, 1.08)
        ax.set_title(titles[sc])
        ax.grid(axis="y", lw=0.3, ls=":", zorder=0)
    axs[0].set_ylabel("rate (95% Wilson interval), n = 60")
    handles = [Patch(fc="white", ec="black", lw=0.5, label="success rate (SR)"),
               Patch(fc="white", ec="black", lw=0.5, hatch="////", label="decision accuracy"),
               Patch(fc=OI["gray"], ec="black", lw=0.5, label="no verification (B0, B1, B1r)"),
               Patch(fc=OI["sky"], ec="black", lw=0.5, label="verification, one entry per class (B2)"),
               Patch(fc=OI["blue"], ec="black", lw=0.5, label="verification, instance memory (FULL)")]
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.2))
    save(fig, "fig_results")


# ------------------------------------------------------------------ Fig. ablation
def fig_ablation():
    fig, axs = plt.subplots(1, 2, figsize=(7.0, 2.35), gridspec_kw={"width_ratios": [1.15, 1]})
    # (a) 2x2 on 60 paired S2 episodes
    ax = axs[0]
    n = 60
    sr = {(1.5, "off"): 40, (1.5, "on"): 40, (2.5, "off"): 48, (2.5, "on"): 46}
    dec = {(1.5, "off"): 39, (1.5, "on"): 41, (2.5, "off"): 50, (2.5, "on"): 46}
    w = 0.34
    for gi, so in enumerate((1.5, 2.5)):
        for bi, sw in enumerate(("off", "on")):
            x = gi + (bi - 0.5) * (w + 0.04)
            k = sr[(so, sw)]
            e = err(k, n)
            ax.bar(x, k / n, w, color=OI["blue"] if sw == "on" else OI["sky"], edgecolor="black", lw=0.5, zorder=3)
            ax.errorbar(x, k / n, yerr=[[e[0]], [e[1]]], fmt="none", ecolor="black", elinewidth=0.6, capsize=1.6, zorder=4)
            ax.plot(x, dec[(so, sw)] / n, marker="D", ms=3.6, color="white", mec="black", mew=0.7, zorder=5)
    ax.set_xticks([0, 1])
    ax.set_xticklabels(["standoff 1.5 m\n(V1 geometry)", "standoff 2.5 m"])
    ax.set_ylim(0, 1.3)
    ax.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
    ax.set_ylabel("S2, FULL (n = 60 paired episodes)")
    ax.set_title("(a) heading sweep and standoff (worlds 32\u201336, 41\u201345)")
    ax.grid(axis="y", lw=0.3, ls=":", zorder=0)
    ax.legend(handles=[Patch(fc=OI["sky"], ec="black", lw=0.5, label="no sweep"),
                       Patch(fc=OI["blue"], ec="black", lw=0.5, label="sweep $\\pm20^\\circ$ (V2 at 2.5 m)"),
                       Line2D([], [], marker="D", ms=3.6, mfc="white", mec="black", ls="none", label="decision accuracy")],
              loc="upper center", ncol=3, frameon=False, fontsize=5.8, columnspacing=0.9, handlelength=1.2)
    # (b) confirmation on fresh worlds 51-55
    ax = axs[1]
    n = 30
    pairs = {"SR": (10, 20), "decision accuracy": (10, 25)}
    for gi, (name, (a_, b_)) in enumerate(pairs.items()):
        for bi, (k, col, lab) in enumerate(((a_, OI["gray"], "V1 geometry"), (b_, OI["green"], "standoff 2.5 m"))):
            x = gi + (bi - 0.5) * (w + 0.04)
            e = err(k, n)
            ax.bar(x, k / n, w, color=col, edgecolor="black", lw=0.5, zorder=3)
            ax.errorbar(x, k / n, yerr=[[e[0]], [e[1]]], fmt="none", ecolor="black", elinewidth=0.6, capsize=1.6, zorder=4)
    ax.set_xticks([0, 1])
    ax.set_xticklabels(list(pairs.keys()))
    ax.set_ylim(0, 1.3)
    ax.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
    ax.set_title("(b) confirmation on fresh worlds 51\u201355 (n = 30)")
    ax.grid(axis="y", lw=0.3, ls=":", zorder=0)
    ax.legend(handles=[Patch(fc=OI["gray"], ec="black", lw=0.5, label="V1 geometry (1.5 m, no sweep)"),
                       Patch(fc=OI["green"], ec="black", lw=0.5, label="standoff 2.5 m, no sweep")],
              loc="upper center", frameon=False, fontsize=5.8, handlelength=1.2)
    save(fig, "fig_ablation")


# ------------------------------------------------------------------ Fig. verifier reliability (3 models)
def fig_verifier():
    pts = {  # model: {prompt: (false confirmation, recall)}
        "Moondream2": {"A": (5 / 116, 77 / 118), "Ac": (5 / 117, 87 / 118), "B": (12 / 116, 86 / 118), "C": (5 / 116, 70 / 118),
                       "D": (13 / 116, 101 / 118), "Dc": (11 / 117, 102 / 118), "E": (1 / 117, 55 / 118),
                       "Ec": (3 / 117, 63 / 118)},
        "Qwen2-VL-2B": {"A": (4 / 116, 82 / 118), "D": (21 / 116, 110 / 118), "E": (1 / 116, 48 / 118)},
        "SmolVLM": {"A": (15 / 116, 84 / 118), "D": (24 / 116, 98 / 118), "E": (1 / 116, 47 / 118)},
    }
    style = {"Moondream2": ("o", OI["blue"]), "Qwen2-VL-2B": ("s", OI["vermillion"]), "SmolVLM": ("^", OI["green"])}
    fig, ax = plt.subplots(figsize=(3.45, 3.0))
    for model, d in pts.items():
        mk, col = style[model]
        for pr, (x, y) in d.items():
            ax.plot(x, y, marker=mk, ms=4.6, color=col, mec="black", mew=0.4, ls="none", zorder=4)
            dx, dy = {"Moondream2": (3.0, 2.0), "Qwen2-VL-2B": (-8.5, 2.5), "SmolVLM": (3.0, -6.5)}[model]
            ax.annotate(pr, (x, y), xytext=(dx, dy), textcoords="offset points", fontsize=5.8, zorder=5)
    ax.set_xlim(-0.005, 0.25)
    ax.set_ylim(0.3, 1.0)
    ax.set_xlabel("removed object wrongly confirmed")
    ax.set_ylabel("present object confirmed")
    ax.grid(lw=0.3, ls=":", zorder=0)
    ax.legend(handles=[Line2D([], [], marker=style[k][0], color=style[k][1], mec="black", mew=0.4, ls="none", ms=4.6, label=k)
                       for k in pts], loc="lower right", frameon=True, framealpha=0.95, edgecolor="none")
    save(fig, "fig_verifier")


# ------------------------------------------------------------------ Fig. failure causes
def fig_failures():
    groups = [("V1\n11\u201315", "B2", (13, 2, 4, 11)), ("V1\n11\u201315", "FULL", (14, 4, 2, 10)),
              ("V2\n32\u201336", "B2", (28, 1, 1, 0)), ("V2\n32\u201336", "FULL", (27, 3, 0, 0)),
              ("V2\n41\u201345", "B2", (22, 6, 0, 2)), ("V2\n41\u201345", "FULL", (19, 8, 0, 3))]
    labels = ["success", "never localised", "localised, not reached", "localised, rejected"]
    cols = [OI["green"], OI["orange"], OI["sky"], OI["vermillion"]]
    hat = [None, "..", "\\\\", "xx"]
    fig, ax = plt.subplots(figsize=(3.45, 2.75))
    xs = []
    x = 0.0
    prev = None
    for g, c, vals in groups:
        if prev is not None and g != prev:
            x += 0.5
        xs.append(x)
        bottom = 0
        for v, col, h in zip(vals, cols, hat):
            ax.bar(x, v, 0.8, bottom=bottom, color=col, edgecolor="black", lw=0.5, hatch=h, zorder=3)
            if v >= 3:
                ax.text(x, bottom + v / 2, str(v), ha="center", va="center", fontsize=6,
                        bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.9), zorder=6)
            bottom += v
        prev = g
        x += 1.0
    ax.set_xticks(xs)
    ax.set_xticklabels([f"{c}" for _, c, _ in groups])
    for gname, idx in (("V1, worlds 11\u201315", (0, 1)), ("V2, 32\u201336", (2, 3)), ("V2, 41\u201345", (4, 5))):
        ax.text((xs[idx[0]] + xs[idx[1]]) / 2, -6.3, gname, ha="center", va="top", fontsize=6)
    ax.set_ylim(0, 31)
    ax.set_yticks([0, 5, 10, 15, 20, 25, 30])
    ax.set_ylabel("S2 missions (of 30)")
    ax.grid(axis="y", lw=0.3, ls=":", zorder=0)
    ax.legend(handles=[Patch(fc=c, ec="black", lw=0.5, hatch=h, label=l) for c, h, l in zip(cols, hat, labels)],
              loc="upper center", bbox_to_anchor=(0.5, -0.24), ncol=2, frameon=False, fontsize=5.9)
    save(fig, "fig_failures")


if __name__ == "__main__":
    fig_world()
    fig_results()
    fig_ablation()
    fig_verifier()
    fig_failures()
