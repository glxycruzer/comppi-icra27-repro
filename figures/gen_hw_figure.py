#!/usr/bin/env python3
"""Hardware-campaign figure (90 trials, 3 courses x 3 controllers x 10).
Three panels: time-to-goal by course, command jitter by course, and the
surprise-course stall fraction. Run with the paper venv:
    ../figvenv/bin/python gen_hw_figure.py
"""
import csv
import os
import statistics as st
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))

plt.rcParams.update({
    "font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8,
    "legend.fontsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "axes.linewidth": 0.6, "figure.dpi": 200,
    "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
})

C = {"comppi": "#0072B2", "dwb": "#000000", "mppi": "#D55E00"}
LAB = {"comppi": "CoMPPI (ours)", "dwb": "DWB", "mppi": "nav2-MPPI"}
CTRLS = ["comppi", "mppi", "dwb"]
COURSES = ["corridor", "passage", "surprise"]
CLAB = {"corridor": "Corridor", "passage": "Passage", "surprise": "Surprise"}

# load per-trial rows, successful only for timing/quality aggregates
rows = defaultdict(list)
with open(os.path.join(HERE, "data", "hw_results.csv")) as f:
    for r in csv.DictReader(f):
        rows[(r["course"], r["controller"])].append(r)


def agg(course, ctrl, field, success_only=True):
    vals = [float(r[field]) for r in rows[(course, ctrl)]
            if (not success_only or r["success"] == "True") and r[field] not in ("", "None")]
    return (st.mean(vals) if vals else 0.0,
            st.stdev(vals) if len(vals) > 1 else 0.0)


fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.3))
w = 0.26

# panel (a): time-to-goal by course. The surprise-vs-corridor rise for DWB is
# the (robust) signature of its stall-recover behavior at the unmapped box.
ax = axes[0]
for i, c in enumerate(CTRLS):
    xs = [j + (i - 1) * w for j in range(len(COURSES))]
    ys = [agg(co, c, "ttg")[0] for co in COURSES]
    es = [agg(co, c, "ttg")[1] for co in COURSES]
    ax.bar(xs, ys, w, yerr=es, capsize=2, label=LAB[c], color=C[c],
           edgecolor="white", linewidth=0.3, error_kw={"linewidth": 0.6})
ax.set_xticks(range(len(COURSES)))
ax.set_xticklabels([CLAB[c] for c in COURSES])
ax.set_ylabel("Time to goal (s)")
ax.set_title("(a) Time to goal", fontsize=8)
ax.spines[["top", "right"]].set_visible(False)
ax.yaxis.grid(True, color="#DDDDDD", linewidth=0.5)
ax.set_axisbelow(True)

# panel (b): command jitter by course -- the clean smoothness signal.
ax = axes[1]
for i, c in enumerate(CTRLS):
    xs = [j + (i - 1) * w for j in range(len(COURSES))]
    ys = [agg(co, c, "cmd_jitter")[0] for co in COURSES]
    bars = ax.bar(xs, ys, w, label=LAB[c], color=C[c],
                  edgecolor="white", linewidth=0.3)
ax.set_xticks(range(len(COURSES)))
ax.set_xticklabels([CLAB[c] for c in COURSES])
ax.set_ylabel("Command jitter (m/s)")
ax.set_title("(b) Command smoothness", fontsize=8)
ax.spines[["top", "right"]].set_visible(False)
ax.yaxis.grid(True, color="#DDDDDD", linewidth=0.5)
ax.set_axisbelow(True)

axes[0].legend(frameon=False, ncol=3, loc="lower center",
               bbox_to_anchor=(1.1, -0.40), columnspacing=1.4, handlelength=1.2)
fig.savefig(os.path.join(HERE, "fig_hw_campaign.pdf"))
plt.close(fig)
print("fig_hw_campaign.pdf")
