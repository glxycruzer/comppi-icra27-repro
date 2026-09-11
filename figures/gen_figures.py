#!/usr/bin/env python3
"""Generate all paper figures from the campaign / ablation / profiling /
hardware data in ./data. Run with the paper venv:
    ../figvenv/bin/python gen_figures.py
Outputs *.pdf (vector, for LaTeX \\includegraphics) into this directory.

Palette: Okabe-Ito (published colorblind-safe categorical set), assigned to
controllers in fixed order so a controller keeps its colour across figures.
"""
import csv
import math
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")

# --- paper style: serif to match IEEEtran, compact type -------------------
plt.rcParams.update({
    "pdf.fonttype": 42, "ps.fonttype": 42,  # TrueType, not Type 3 (PaperPlaza requirement)
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 8,
    "axes.titlesize": 8,
    "axes.labelsize": 8,
    "legend.fontsize": 7,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "axes.linewidth": 0.6,
    "lines.linewidth": 1.2,
    "figure.dpi": 200,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
})
COL1 = 3.4  # single IEEE column width (in)

# Okabe-Ito, fixed assignment per controller (identity, not rank)
C = {
    "comppi":          "#0072B2",  # blue  (our method)
    "dwb":             "#000000",  # black (production baseline)
    "mppi":            "#D55E00",  # vermillion (nav2-MPPI baseline)
    "comppi_iid":      "#E69F00",  # orange
    "comppi_nodetour": "#009E73",  # bluish green
    "comppi_noanneal": "#CC79A7",  # reddish purple
}
LABEL = {
    "comppi": "CoMPPI (ours)", "dwb": "DWB", "mppi": "Nav2-MPPI",
    "comppi_iid": "no block-noise (C1)", "comppi_nodetour": "no gating (C2)",
    "comppi_noanneal": "no anneal (C2)",
}
WORLDS = ["open", "clutter", "gap", "blocked"]
WORLD_LABEL = {"open": "Open", "clutter": "Clutter", "gap": "Gap",
               "blocked": "Blocked"}


def load(csvname):
    rows = []
    with open(os.path.join(DATA, csvname)) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def agg_success(rows):
    """(world, controller) -> success fraction."""
    tot = defaultdict(int)
    ok = defaultdict(int)
    for r in rows:
        k = (r["world"], r["controller"])
        tot[k] += 1
        if r["success"] == "True":
            ok[k] += 1
    return {k: ok[k] / tot[k] for k in tot}, tot


def agg_metric(rows, field, success_only=True):
    """(world, controller) -> list of float values."""
    out = defaultdict(list)
    for r in rows:
        if success_only and r["success"] != "True":
            continue
        v = r.get(field, "")
        if v not in ("", None):
            out[(r["world"], r["controller"])].append(float(v))
    return out


# --------------------------------------------------------------------------
def fig_success():
    rows = load("campaign.csv")
    succ, _ = agg_success(rows)
    controllers = ["comppi", "dwb", "mppi"]
    fig, ax = plt.subplots(figsize=(COL1, 2.1))
    n = len(controllers)
    w = 0.8 / n
    for i, c in enumerate(controllers):
        xs = [j + (i - (n - 1) / 2) * w for j in range(len(WORLDS))]
        ys = [100 * succ.get((wd, c), 0) for wd in WORLDS]
        bars = ax.bar(xs, ys, w, label=LABEL[c], color=C[c],
                      edgecolor="white", linewidth=0.4)
        for x, y in zip(xs, ys):
            if y < 25:  # direct-label the dramatic low bars
                ax.text(x, y + 2, f"{y:.0f}", ha="center", va="bottom",
                        fontsize=6, color=C[c])
    ax.set_xticks(range(len(WORLDS)))
    ax.set_xticklabels([WORLD_LABEL[w] for w in WORLDS])
    ax.set_ylabel("Success rate (%)")
    ax.set_ylim(0, 108)
    ax.legend(frameon=False, ncol=3, loc="lower center",
              bbox_to_anchor=(0.5, -0.42), columnspacing=1.0, handlelength=1.2)
    ax.spines[["top", "right"]].set_visible(False)
    ax.set_axisbelow(True)
    ax.yaxis.grid(True, color="#DDDDDD", linewidth=0.5)
    fig.savefig(os.path.join(HERE, "fig_success.pdf"))
    plt.close(fig)
    print("fig_success.pdf")


def fig_ablation():
    rows = load("campaign.csv")
    succ, _ = agg_success(rows)
    sym = load("sym_ablation.csv")
    ssucc, _ = agg_success(sym)

    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(COL1, 2.0), gridspec_kw={"width_ratios": [3, 1]})

    # left: full vs 3 ablations across the 4 worlds
    ctrls = ["comppi", "comppi_iid", "comppi_nodetour", "comppi_noanneal"]
    n = len(ctrls)
    w = 0.82 / n
    for i, c in enumerate(ctrls):
        xs = [j + (i - (n - 1) / 2) * w for j in range(len(WORLDS))]
        ys = [100 * succ.get((wd, c), 0) for wd in WORLDS]
        ax1.bar(xs, ys, w, label=LABEL[c], color=C[c],
                edgecolor="white", linewidth=0.3)
    ax1.set_xticks(range(len(WORLDS)))
    ax1.set_xticklabels([WORLD_LABEL[w] for w in WORLDS], rotation=20, ha="right")
    ax1.set_ylabel("Success rate (%)")
    ax1.set_ylim(0, 108)
    ax1.spines[["top", "right"]].set_visible(False)
    ax1.yaxis.grid(True, color="#DDDDDD", linewidth=0.5)
    ax1.set_axisbelow(True)
    ax1.legend(frameon=False, ncol=2, loc="lower center",
               bbox_to_anchor=(0.5, -0.72), columnspacing=0.8,
               handlelength=1.1, fontsize=6)

    # right: symmetric blockage — annealing on vs off (the decisive ablation)
    labs = ["with\nanneal", "no\nanneal"]
    ys = [100 * ssucc.get(("blocked_sym", "comppi"), 0),
          100 * ssucc.get(("blocked_sym", "comppi_noanneal"), 0)]
    ax2.bar([0, 1], ys, 0.6, color=[C["comppi"], C["comppi_noanneal"]],
            edgecolor="white", linewidth=0.4)
    for x, y in zip([0, 1], ys):
        ax2.text(x, y + 2, f"{y:.0f}", ha="center", va="bottom", fontsize=6)
    ax2.set_xticks([0, 1])
    ax2.set_xticklabels(labs, fontsize=6)
    ax2.set_ylim(0, 108)
    ax2.set_title("Symmetric\nblockage", fontsize=7)
    ax2.spines[["top", "right"]].set_visible(False)
    ax2.yaxis.grid(True, color="#DDDDDD", linewidth=0.5)
    ax2.set_axisbelow(True)
    fig.savefig(os.path.join(HERE, "fig_ablation.pdf"))
    plt.close(fig)
    print("fig_ablation.pdf")


def _pct(sorted_xs, q):
    if not sorted_xs:
        return 0.0
    return sorted_xs[min(len(sorted_xs) - 1, int(q * len(sorted_xs)))]


def fig_profile():
    batches = [256, 512, 1024, 2048]
    med, p95, mx = [], [], []
    for b in batches:
        xs = sorted(int(l) / 1000.0 for l in
                    open(os.path.join(DATA, f"comppi_profile_{b}.txt"))
                    if l.strip())
        med.append(_pct(xs, 0.5))
        p95.append(_pct(xs, 0.95))
        mx.append(xs[-1])
    fig, ax = plt.subplots(figsize=(COL1, 2.0))
    x = range(len(batches))
    ax.plot(x, med, "-o", color=C["comppi"], markersize=4, label="median")
    ax.plot(x, p95, "--s", color=C["comppi"], markersize=3.5,
            markerfacecolor="white", label="p95")
    ax.plot(x, mx, ":^", color=C["mppi"], markersize=3.5, label="max")
    ax.axhline(100, color="#999999", linewidth=0.8, linestyle="-")
    ax.text(1.5, 108, "10 Hz control budget (100 ms)", fontsize=6, va="bottom",
            ha="center", color="#666666")
    ax.set_yscale("log")
    ax.set_xticks(list(x))
    ax.set_xticklabels(batches)
    ax.set_xlabel("Batch size (sampled trajectories)")
    ax.set_ylabel("Solve time (ms)")
    ax.set_ylim(1, 260)
    ax.legend(frameon=False, loc="lower right", handlelength=1.6)
    ax.spines[["top", "right"]].set_visible(False)
    ax.yaxis.grid(True, which="both", color="#EEEEEE", linewidth=0.5)
    ax.set_axisbelow(True)
    # annotate production point
    prod = batches.index(1024)
    ax.annotate("production\n(6.2 ms median)", (prod, med[prod]),
                textcoords="offset points", xytext=(6, 14), fontsize=6,
                color=C["comppi"],
                arrowprops=dict(arrowstyle="->", color=C["comppi"], lw=0.6))
    fig.savefig(os.path.join(HERE, "fig_profile.pdf"))
    plt.close(fig)
    print("fig_profile.pdf")


def fig_smoothness():
    rows = load("campaign.csv")
    rms = agg_metric(rows, "rms_dv")
    controllers = ["comppi", "dwb", "mppi"]
    fig, ax = plt.subplots(figsize=(COL1, 1.9))
    for i, c in enumerate(controllers):
        vals = []
        for wd in WORLDS:
            vals += rms.get((wd, c), [])
        mean = sum(vals) / len(vals) if vals else 0
        ax.bar(i, mean, 0.6, color=C[c], edgecolor="white", linewidth=0.4)
        ax.text(i, mean + 0.0005, f"{mean:.4f}", ha="center", va="bottom",
                fontsize=6)
    ax.set_xticks(range(len(controllers)))
    ax.set_xticklabels([LABEL[c] for c in controllers])
    ax.set_ylabel("Velocity-jitter RMS (m/s)")
    ax.spines[["top", "right"]].set_visible(False)
    ax.yaxis.grid(True, color="#DDDDDD", linewidth=0.5)
    ax.set_axisbelow(True)
    fig.savefig(os.path.join(HERE, "fig_smoothness.pdf"))
    plt.close(fig)
    print("fig_smoothness.pdf")


def fig_hw_velocity():
    # Both traces are REAL recorded runs on the robot, same corridor goal,
    # reproduced by toggling use_odometry_velocity (the C3 anchor source):
    #   before = anchor to measured odom (zero on this base) -> 0.023 m/s crawl
    #   after  = anchor to previous command                  -> 0.388 m/s cruise
    # Data are the controller's commanded vx (cmd_vel_nav, zero-order-held onto
    # the odometry time base by extract_bag.py).
    def series(fname):
        t, v = [], []
        for r in csv.DictReader(open(os.path.join(DATA, fname))):
            t.append(float(r["t"])); v.append(float(r["cmd_vx"]))
        nz = [i for i, x in enumerate(v) if abs(x) > 0.005]
        if not nz:
            return t, v
        t0 = t[nz[0]]
        return [ti - t0 for ti in t[nz[0]:nz[-1] + 1]], v[nz[0]:nz[-1] + 1]
    tb, vb = series("hw_c3_before.csv")
    ta, va = series("hw_c3_after.csv")
    # show the before-crawl over the after-run's span (+ margin); it is flat
    # for its full 120 s and the caption states the duration
    span = (ta[-1] if ta else 40) + 6
    keep = [i for i, x in enumerate(tb) if x <= span]
    tb, vb = [tb[i] for i in keep], [vb[i] for i in keep]
    fig, ax = plt.subplots(figsize=(COL1, 2.0))
    ax.plot(tb, vb, color=C["dwb"], linewidth=1.1,
            label="before fix: odom anchor (0.023 m/s)")
    ax.plot(ta, va, color=C["comppi"], linewidth=1.2,
            label="after fix: command anchor (0.388 m/s)")
    ax.axhline(0.4, color="#999999", linewidth=0.8, linestyle="--")
    ax.text(0.2, 0.405, "$v_{x,\\max}=0.4$", fontsize=6, color="#666666")
    ax.set_xlabel("Time into drive (s)")
    ax.set_ylabel("Commanded $v_x$ (m/s)")
    ax.set_ylim(0, 0.44)
    ax.legend(frameon=False, loc="center right", handlelength=1.6)
    ax.spines[["top", "right"]].set_visible(False)
    ax.yaxis.grid(True, color="#EEEEEE", linewidth=0.5)
    ax.set_axisbelow(True)
    fig.savefig(os.path.join(HERE, "fig_hw_velocity.pdf"))
    plt.close(fig)
    print("fig_hw_velocity.pdf")


if __name__ == "__main__":
    fig_success()
    fig_ablation()
    fig_profile()
    fig_smoothness()
    fig_hw_velocity()
    print("all figures written to", HERE)
