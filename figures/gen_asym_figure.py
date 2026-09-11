#!/usr/bin/env python3
"""Asymmetry sweep figure: success rate (Wilson 95% CI) and time-to-goal vs the
lateral offset of a single unmapped 0.4 m box from the planned path, for full
CoMPPI, CoMPPI without blockage gating, and CoMPPI without temperature
sharpening. Answers "how often does symmetric mode averaging actually matter":
the failure is a finite basin around symmetry, not a point.

Input: figures/data/asym_sweep.csv (batch_run.py results.csv)
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
SRC = os.path.join(HERE, "data", "asym_sweep.csv")
plt.rcParams.update({
    "pdf.fonttype": 42, "ps.fonttype": 42,  # TrueType, not Type 3 (PaperPlaza requirement)
    "font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8,
    "legend.fontsize": 7, "xtick.labelsize": 7.5, "ytick.labelsize": 7.5,
    "axes.linewidth": 0.6, "figure.dpi": 200,
    "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
})
C = {"comppi": "#0072B2", "comppi_nodetour": "#D55E00", "comppi_noanneal": "#009E73"}
LAB = {"comppi": "CoMPPI (full)", "comppi_nodetour": "no blockage gating",
       "comppi_noanneal": "no temperature sharpening"}
MK = {"comppi": "o", "comppi_nodetour": "s", "comppi_noanneal": "^"}


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (p, max(0.0, c - h), min(1.0, c + h))


rows = list(csv.DictReader(open(SRC)))
succ = defaultdict(lambda: defaultdict(list))   # ctrl -> offset -> [bool]
ttg = defaultdict(lambda: defaultdict(list))
for r in rows:
    off = int(r["world"][len("blocked_off"):]) / 100.0
    ok = r["success"] == "True"
    succ[r["controller"]][off].append(ok)
    if ok and r.get("ttg"):
        ttg[r["controller"]][off].append(float(r["ttg"]))

fig, ax1 = plt.subplots(1, 1, figsize=(3.4, 1.85))
ax2 = fig.add_axes([0, 0, 0.001, 0.001]); ax2.set_visible(False)
for ctrl in ("comppi", "comppi_nodetour", "comppi_noanneal"):
    if ctrl not in succ:
        continue
    offs = sorted(succ[ctrl])
    p, lo, hi = zip(*[wilson(sum(succ[ctrl][o]), len(succ[ctrl][o])) for o in offs])
    ax1.errorbar(offs, [100 * v for v in p],
                 yerr=[[100 * (a - b) for a, b in zip(p, lo)],
                       [100 * (b - a) for a, b in zip(p, hi)]],
                 color=C[ctrl], marker=MK[ctrl], markersize=3.5, linewidth=1.2,
                 capsize=2, label=LAB[ctrl])
    to = [o for o in offs if ttg[ctrl][o]]
    ax2.errorbar(to, [st.mean(ttg[ctrl][o]) for o in to],
                 yerr=[st.pstdev(ttg[ctrl][o]) if len(ttg[ctrl][o]) > 1 else 0 for o in to],
                 color=C[ctrl], marker=MK[ctrl], markersize=3.5, linewidth=1.2,
                 capsize=2, label=LAB[ctrl])

# geometric landmarks: box half-width (0.2) and box edge clear of robot body
for ax in (ax1, ax2):
    ax.axvline(0.2, color="#999999", linestyle=":", linewidth=0.8)
    ax.set_xlabel("Lateral offset of blockage from path (m)")
    ax.spines[["top", "right"]].set_visible(False)
    ax.yaxis.grid(True, color="#EEEEEE", linewidth=0.5)
    ax.set_axisbelow(True)
    ax.set_xlim(-0.02, 0.52)
ax1.text(0.208, 52, "box edge\nleaves path", fontsize=6, color="#777777")
ax2.text(0.208, 33.5, "box edge\nleaves path", fontsize=6, color="#777777")
ax1.set_ylabel("Success (%)")
ax1.set_ylim(-3, 105)
ax1.set_title("Success vs. lateral offset (15 seeds per point, Wilson 95% CI)", fontsize=8)
ax1.legend(frameon=False, fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.28), ncol=3)
ax2.set_ylabel("Time-to-goal, successes (s)")
ax2.set_title("(b) Time-to-goal of successful trials", fontsize=8)

fig.savefig(os.path.join(HERE, "fig_asym.pdf"))
plt.close(fig)

# console summary for the paper text
print(f"{'ctrl':18s} {'off':>5s} {'succ':>6s} {'CI':>12s} {'ttg':>12s}")
for ctrl in sorted(succ):
    for o in sorted(succ[ctrl]):
        k, n = sum(succ[ctrl][o]), len(succ[ctrl][o])
        p, lo, hi = wilson(k, n)
        t = ttg[ctrl][o]
        ts = f"{st.mean(t):5.1f}±{(st.pstdev(t) if len(t) > 1 else 0):.1f}" if t else "   --"
        print(f"{ctrl:18s} {o:5.2f} {k:3d}/{n:<2d} [{100*lo:3.0f},{100*hi:3.0f}]% {ts:>12s}")
print("fig_asym.pdf")
