#!/usr/bin/env python3
"""Blockage-detection threshold sensitivity.

Two knobs decide when detour mode engages: the costmap cost at which a path
point counts as blocked (default 253 = inscribed; lower = more sensitive, so
inflation-band cells near walls trigger false positives), and how far ahead a
blockage must lie, as a multiple of the auto lookahead arc (default 1.0).
Panels: success and time-to-goal on gap / clutter (false-positive cost) and
blocked (detection), 15 seeds per point. The default point is taken from the
main campaign (same controller, seeds 0-14).

Inputs: figures/data/thresh_sweep.csv (+ campaign.csv for the default point)
"""
import csv
import math
import os
import statistics as st
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, "data")
plt.rcParams.update({
    "pdf.fonttype": 42, "ps.fonttype": 42,  # TrueType, not Type 3 (PaperPlaza requirement)
    "font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8,
    "legend.fontsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "axes.linewidth": 0.6, "figure.dpi": 200,
    "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
})
W = {"gap": ("#0072B2", "o", "narrow gap"), "clutter": ("#009E73", "s", "clutter"),
     "blocked": ("#D55E00", "^", "blocked path")}
COST = {"comppi_bc100": 100, "comppi_bc150": 150, "comppi_bc200": 200, "comppi": 253}
RANGE = {"comppi_rs05": 0.5, "comppi": 1.0, "comppi_rs15": 1.5, "comppi_rs20": 2.0}


def wilson(k, n, z=1.96):
    if n == 0:
        return (0, 0, 0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return p, max(0, c - h), min(1, c + h)


rows = list(csv.DictReader(open(os.path.join(D, "thresh_sweep.csv"))))
# default point from the main campaign, seeds 0-14 (paired with the sweep)
for r in csv.DictReader(open(os.path.join(D, "campaign.csv"))):
    if r["controller"] == "comppi" and int(r["seed"]) < 15 and r["world"] in W:
        rows.append(r)

succ = defaultdict(list)   # (ctrl, world) -> [bool]
ttg = defaultdict(list)
for r in rows:
    k = (r["controller"], r["world"])
    ok = r["success"] == "True"
    succ[k].append(ok)
    if ok and r.get("ttg"):
        ttg[k].append(float(r["ttg"]))

fig, axes = plt.subplots(1, 2, figsize=(3.5, 1.45), gridspec_kw={"wspace": 0.55})


def panel(ax_s, ax_t, mapping, xlabel, default_x):
    for w, (col, mk, lab) in W.items():
        xs = sorted(mapping.values())
        pts = sorted((x, c) for c, x in mapping.items())
        p, lo, hi, tm, ts = [], [], [], [], []
        for x, c in pts:
            v = succ[(c, w)]
            pp, l, h = wilson(sum(v), len(v))
            p.append(100 * pp); lo.append(100 * (pp - l)); hi.append(100 * (h - pp))
            t = ttg[(c, w)]
            tm.append(st.mean(t) if t else float("nan"))
            ts.append(st.pstdev(t) if len(t) > 1 else 0)
        ax_s.errorbar(xs, p, yerr=[lo, hi], color=col, marker=mk, markersize=3.2,
                      linewidth=1.1, capsize=2, label=lab)
        ax_t.errorbar(xs, tm, yerr=ts, color=col, marker=mk, markersize=3.2,
                      linewidth=1.1, capsize=2, label=lab)
    for ax in (ax_s, ax_t):
        ax.axvline(default_x, color="#999999", linestyle=":", linewidth=0.8)
        ax.set_xlabel(xlabel)
        ax.set_xticks(sorted(mapping.values()))
        ax.set_xticklabels([f"{v:g}" for v in sorted(mapping.values())])
        ax.spines[["top", "right"]].set_visible(False)
        ax.yaxis.grid(True, color="#EEEEEE", linewidth=0.5)
        ax.set_axisbelow(True)
    ax_s.set_ylim(-3, 105)
    ax_s.set_ylabel("Success (%)")
    ax_t.set_ylabel("Time-to-goal (s)")


panel(axes[0], axes[1], RANGE, "Range $r_{\\mathrm{det}}$ (×LA)", 1.0)
axes[0].set_title("(a) Success vs. detection range", fontsize=8)
axes[1].set_title("(b) Time-to-goal", fontsize=8)
axes[0].legend(frameon=False, fontsize=6, loc="lower right")
fig.savefig(os.path.join(HERE, "fig_thresh.pdf"))
plt.close(fig)

print(f"{'ctrl':14s} {'world':8s} {'succ':>6s} {'CI':>10s} {'ttg':>11s}")
for mapping in (COST, RANGE):
    for c in sorted(mapping, key=lambda c: mapping[c]):
        for w in W:
            v = succ[(c, w)]
            if not v:
                continue
            p, lo, hi = wilson(sum(v), len(v))
            t = ttg[(c, w)]
            ts = f"{st.mean(t):5.1f}±{(st.pstdev(t) if len(t) > 1 else 0):.1f}" if t else "   --"
            print(f"{c:14s} {w:8s} {sum(v):2d}/{len(v):<2d} [{100*lo:3.0f},{100*hi:3.0f}]% {ts:>11s}")
print("fig_thresh.pdf")
