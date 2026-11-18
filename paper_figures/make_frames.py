#!/usr/bin/env python3
"""make_frames.py - camera-frame figures. fig_frames: the S2 mission of the trace figure in two arms; fig_confusion: barrel and cone.
Usage: python3 make_frames.py OUTDIR"""
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
OUT = sys.argv[1] if len(sys.argv) > 1 else ".."
plt.rcParams.update({"font.family": "serif", "font.serif": ["STIXGeneral", "DejaVu Serif"], "font.size": 7, "pdf.fonttype": 42,
                     "savefig.bbox": "tight", "savefig.pad_inches": 0.03})

def panel(fig_name, items, ncol, figsize, wspace=0.04, hspace=0.18):
    nrow = (len(items) + ncol - 1) // ncol
    fig, axs = plt.subplots(nrow, ncol, figsize=figsize)
    axs = axs.ravel() if hasattr(axs, "ravel") else [axs]
    for ax, (f, t) in zip(axs, items):
        im = Image.open(f).convert("RGB").resize((960, 540))
        ax.imshow(im); ax.set_xticks([]); ax.set_yticks([])
        for s in ax.spines.values():
            s.set_linewidth(0.6)
        ax.set_title(t, fontsize=6.8)
    fig.subplots_adjust(wspace=wspace, hspace=hspace)
    fig.savefig(f"{OUT}/{fig_name}.pdf", dpi=170); fig.savefig(f"/tmp/{fig_name}.png", dpi=120)
    plt.close(fig)

panel("fig_frames", [
    ("frames/v1abl_S2_ep4_look1.png", "(a) V1, look 1: remembered place empty (no)"),
    ("frames/v1abl_S2_ep4_look3.png", "(b) V1, look 3, 1.1 m from the postbox (yes)"),
    ("frames/standoff_S2_ep4_look3.png", "(c) standoff 2.5 m, look 3: scan, 5.3 m (yes)"),
    ("frames/standoff_S2_ep4_look4.png", "(d) standoff 2.5 m, look 4: 1.9 m (yes)")], ncol=2, figsize=(6.4, 3.9))
panel("fig_confusion", [
    ("frames/barrel_present.png", "(a) construction barrel"),
    ("frames/cone_present.png", "(b) traffic cone")], ncol=2, figsize=(3.45, 1.35), wspace=0.05)
print("saved")
