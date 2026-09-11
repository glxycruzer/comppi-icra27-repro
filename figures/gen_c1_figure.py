#!/usr/bin/env python3
"""C1 figure: the critic-noise collapse, from real simulation runs.

Commanded forward velocity over time on the same open-world task, controller
identical except for the exploration noise. With i.i.d. per-step noise the
path-quality critics penalize the (necessarily wigglier) fast samples, so the
path-integral update collapses onto the unperturbed near-stationary sample and
the robot crawls. Block (piecewise-constant) noise makes fast samples coherent
maneuvers the critics can reward, and the controller cruises at the speed limit.
"""
import csv
import os

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
C = {"block": "#0072B2", "iid": "#D55E00"}


def load(fname):
    t, v = [], []
    for r in csv.DictReader(open(os.path.join(HERE, "data", fname))):
        t.append(float(r["t"])); v.append(float(r["v"]))
    return t, v


fig, ax = plt.subplots(figsize=(3.4, 2.2))
tb, vb = load("c1_block.csv")
ti, vi = load("c1_iid.csv")
ax.plot(tb, vb, color=C["block"], linewidth=1.3, label="block noise (CoMPPI)")
ax.plot(ti, vi, color=C["iid"], linewidth=1.3, label="i.i.d. noise")
ax.axhline(0.4, color="#999999", linewidth=0.8, linestyle="--")
ax.text(ax.get_xlim()[1], 0.405, "$v_{x,\\max}$", fontsize=6, color="#666666",
        ha="right", va="bottom")
ax.annotate("stall: no fast sample\nis ever proposed",
            xy=(max(ti) * 0.6, 0.03), xytext=(max(ti) * 0.30, 0.17),
            fontsize=6.2, color=C["iid"],
            arrowprops=dict(arrowstyle="->", color=C["iid"], lw=0.6))
ax.set_xlabel("Time (s)")
ax.set_ylabel("Commanded $v_x$ (m/s)")
ax.set_ylim(0, 0.44)
ax.legend(frameon=False, loc="center right")
ax.spines[["top", "right"]].set_visible(False)
ax.grid(True, color="#EEEEEE", linewidth=0.5)
ax.set_axisbelow(True)
fig.savefig(os.path.join(HERE, "fig_c1_cost.pdf"))
plt.close(fig)
print("fig_c1_cost.pdf")
