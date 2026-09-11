#!/usr/bin/env python3
"""Dynamic-obstacle (crossing) figure.

(a) Outcome per controller over 15 seeds: reached goal / stalled-aborted /
    robot-at-fault collision, with the robot-at-fault criterion (contact while
    the robot is moving; a box walking into a stopped robot is not the robot's
    collision) and a half-cell (2.5 cm) quantization tolerance.
(b) One seed, CoMPPI vs Nav2-MPPI: commanded forward speed vs time with the
    box's lateral distance to the path overlaid -- shows the stop-and-resume
    (or stall) behaviour when the box steps into the corridor.

Inputs: figures/data/dyn/results.csv and figures/data/dyn/<rundir>/{traj,dyn}.csv
"""
import csv
import os
import statistics as st
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, "data", "dyn")
plt.rcParams.update({
    "font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8,
    "legend.fontsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "axes.linewidth": 0.6, "figure.dpi": 200,
    "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
})
ORDER_ALL = ["comppi", "comppi_noanneal", "comppi_nodetour", "mppi", "dwb"]
LAB = {"comppi": "CoMPPI", "comppi_noanneal": "CoMPPI\n–sharpen.",
       "comppi_nodetour": "CoMPPI\n–gating", "mppi": "Nav2-\nMPPI", "dwb": "DWB"}
C = {"ok": "#0072B2", "stall": "#E69F00", "coll": "#D55E00"}

rows = list(csv.DictReader(open(os.path.join(D, "results.csv"))))
present = {r["controller"] for r in rows}
ORDER = [c for c in ORDER_ALL if c in present]
# categories: reached goal without contact / reached goal but contacted the
# box while moving / never reached the goal (froze until the progress checker
# aborted), the last split into froze-without-contact and froze-after-contact.
out = defaultdict(lambda: {"ok": 0, "stall": 0, "coll": 0, "ttg": [], "clr": [], "stall_coll": 0})
for r in rows:
    c = r["controller"]
    reached = "status=4" in r.get("note", "")
    contact = "dyn_collision" in r.get("note", "")
    if reached and contact:
        out[c]["coll"] += 1
        out[c]["ttg"].append(float(r["ttg"]))
    elif reached:
        out[c]["ok"] += 1
        out[c]["ttg"].append(float(r["ttg"]))
    else:
        out[c]["stall"] += 1
        out[c]["stall_coll"] += int(contact)
    if r.get("min_dyn_clear"):
        out[c]["clr"].append(float(r["min_dyn_clear"]))

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(3.4, 3.1), gridspec_kw={"height_ratios": [1, 1.1], "hspace": 0.95})
xs = range(len(ORDER))
ok = [out[c]["ok"] for c in ORDER]
stl = [out[c]["stall"] for c in ORDER]
col = [out[c]["coll"] for c in ORDER]
ax1.bar(xs, ok, color=C["ok"], width=0.6, label="reached goal, no contact")
ax1.bar(xs, col, bottom=ok, color=C["coll"], width=0.6,
        label="reached goal, contact while moving")
stc = [out[c]["stall_coll"] for c in ORDER]           # froze AFTER striking the box
stn = [a - b for a, b in zip(stl, stc)]                 # froze without contact
ax1.bar(xs, stn, bottom=[a + b for a, b in zip(ok, col)], color=C["stall"], width=0.6,
        label="froze, no contact")
ax1.bar(xs, stc, bottom=[a + b + c for a, b, c in zip(ok, col, stn)], color=C["stall"], width=0.6,
        hatch="////", edgecolor=C["coll"], linewidth=0.4, label="froze after contact")
ax1.set_xticks(list(xs))
ax1.set_xticklabels([LAB[c] for c in ORDER], fontsize=7.5)
ax1.set_ylabel("Trials (of 15)")
ax1.set_ylim(0, 16)
ax1.set_yticks([0, 5, 10, 15])
ax1.set_title("(a) Outcome, box crossing the path", fontsize=8)
ax1.legend(frameon=False, fontsize=6.6, loc="upper center", bbox_to_anchor=(0.5, -0.36), ncol=2, columnspacing=1.0, handlelength=1.4)
ax1.spines[["top", "right"]].set_visible(False)
ax1.yaxis.grid(True, color="#EEEEEE", linewidth=0.5)
ax1.set_axisbelow(True)

# (b) speed traces for one seed
SEED = int(os.environ.get("DYN_SEED", "0"))
def load(ctrl):
    rd = os.path.join(D, f"crossing_s{SEED}_{ctrl}")
    tr = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(os.path.join(rd, "traj.csv")))]
    dn = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(os.path.join(rd, "dyn.csv")))]
    t0 = tr[0]["t_abs"]
    return tr, dn, t0

CC = {"comppi": "#0072B2", "mppi": "#009E73"}
for ctrl in ("comppi", "mppi"):
    try:
        tr, dn, t0 = load(ctrl)
    except FileNotFoundError:
        continue
    ax2.plot([r["t_abs"] - t0 for r in tr], [r["v"] for r in tr], color=CC[ctrl],
             linewidth=1.2, label=f"{LAB[ctrl].replace(chr(10), ' ')} $v_x$")
    # box lateral distance from the path (right axis), only while it moves
    mv = [r for r in dn if r["moving"] > 0]
    if mv and ctrl == "comppi":
        ax2b = ax2.twinx()
        ax2b.plot([r["t_abs"] - t0 for r in mv], [abs(r["by"] - 5.0) for r in mv],
                  color="#888888", linestyle="--", linewidth=1.0, label="box |y - path|")
        ax2b.axhline(0.25 + 0.19, color="#BBBBBB", linewidth=0.6, linestyle=":")
        ax2b.set_ylabel("Box distance to path (m)", color="#666666")
        ax2b.tick_params(axis="y", colors="#666666")
        ax2b.set_ylim(0, 1.6)
        ax2b.spines[["top"]].set_visible(False)
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Commanded $v_x$ (m/s)")
ax2.set_ylim(-0.02, 0.45)
ax2.set_title(f"(b) Speed trace, seed {SEED}", fontsize=8)
h1, l1 = ax2.get_legend_handles_labels()
try:
    h2, l2 = ax2b.get_legend_handles_labels()
except NameError:
    h2, l2 = [], []
ax2.legend(h1 + h2, l1 + l2, frameon=False, fontsize=6.6, loc="upper left", bbox_to_anchor=(0.0, 0.82))
ax2.spines[["top"]].set_visible(False)
ax2.yaxis.grid(True, color="#EEEEEE", linewidth=0.5)
ax2.set_axisbelow(True)

fig.savefig(os.path.join(HERE, "fig_dyn.pdf"))
plt.close(fig)

print(f"{'ctrl':18s} {'clean':>5s} {'contact':>7s} {'froze':>5s} {'(froze+contact)':>15s} {'ttg reached':>12s} {'min clear (moving)':>20s}")
for c in ORDER:
    o = out[c]
    t = f"{st.mean(o['ttg']):5.1f}±{(st.pstdev(o['ttg']) if len(o['ttg']) > 1 else 0):.1f}" if o["ttg"] else "    --"
    cl = f"{min(o['clr']):.3f} / med {st.median(o['clr']):.3f}" if o["clr"] else "--"
    print(f"{c:18s} {o['ok']:5d} {o['coll']:7d} {o['stall']:5d} {o['stall_coll']:15d} {t:>12s} {cl:>20s}")
print("fig_dyn.pdf")
