#!/usr/bin/env python3
"""C1 mechanism figure from the controller's own per-sample critic dump.

(a) Distribution of sampled horizon-mean forward speed under i.i.d. / block /
    colored noise: i.i.d. barely explores speed (its horizon-mean perturbation
    shrinks as sigma/sqrt(T)), block and colored span the speed range.
(b) Total critic cost vs sampled speed, per mode: all three reward speed
    identically -- the critics are NOT penalizing i.i.d. roughness. The stall is
    exploration starvation: the convex path-integral update cannot select a
    speed that was never proposed.
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
DEC = os.path.join(HERE, "data", "decomp")
plt.rcParams.update({
    "pdf.fonttype": 42, "ps.fonttype": 42,  # TrueType, not Type 3 (PaperPlaza requirement)
    "font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8,
    "legend.fontsize": 7, "xtick.labelsize": 6.5, "ytick.labelsize": 6.5,
    "axes.linewidth": 0.6, "figure.dpi": 200,
    "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
})
C = {"block": "#0072B2", "colored": "#009E73", "iid": "#D55E00"}
LAB = {"block": "block (CoMPPI)", "colored": "colored 1/f²", "iid": "i.i.d."}
CRITS = ["obstacle", "align", "follow", "angle", "fwd", "twirl"]

data = {}
for m in ("iid", "block", "colored"):
    rows = [{k: float(v) for k, v in r.items()}
            for r in csv.DictReader(open(os.path.join(DEC, f"decomp_{m}.csv")))]
    data[m] = rows

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(3.4, 1.36))
plt.subplots_adjust(wspace=0.42)

# (a) sampled-speed distributions
bins = [i * 0.02 for i in range(0, 14)]
for m in ("iid", "block", "colored"):
    vx = [r["vx"] for r in data[m]]
    ax1.hist(vx, bins=bins, histtype="step", linewidth=1.1, color=C[m],
             label=f"{LAB[m]} (max {max(vx):.3f})")
ax1.set_xlabel("Horizon-mean speed (m/s)", fontsize=7)
ax1.set_ylabel("Samples (of 1024)", fontsize=7)
ax1.set_title("(a) Speed exploration", fontsize=7.5)
ax1.legend(frameon=False, fontsize=5.6, handlelength=1.2, borderaxespad=0.2, loc="upper right")
ax1.spines[["top", "right"]].set_visible(False)
ax1.yaxis.grid(True, color="#EEEEEE", linewidth=0.5)
ax1.set_axisbelow(True)
# reference: one block's worth of rate-limited increments, a_max*B*dt = 0.2 m/s
blk = 0.25 * 8 * 0.1
ax1.axvline(blk, color=C["block"], linestyle=":", linewidth=0.9)
ax1.text(blk - 0.066, ax1.get_ylim()[1] * 0.30, "$a_{\\max}B\\Delta t$", fontsize=5.6,
         color=C["block"])

# (b) total cost vs speed, binned means per mode
for m in ("iid", "block", "colored"):
    b = defaultdict(list)
    for r in data[m]:
        b[min(int(r["vx"] / 0.05), 5)].append(sum(r[c] for c in CRITS))
    xs = [k * 0.05 + 0.025 for k in sorted(b)]
    ys = [st.mean(b[k]) for k in sorted(b)]
    ax2.plot(xs, ys, "-o", color=C[m], markersize=2.5, linewidth=1.0, label=LAB[m])
ax2.set_xlabel("Horizon-mean speed (m/s)", fontsize=7)
ax2.set_ylabel("Total critic cost", fontsize=7)
ax2.set_title("(b) Critic cost vs. speed", fontsize=7.5)
ax2.legend(frameon=False, fontsize=5.6, handlelength=1.2, borderaxespad=0.2, loc="lower left")
ax2.spines[["top", "right"]].set_visible(False)
ax2.yaxis.grid(True, color="#EEEEEE", linewidth=0.5)
ax2.set_axisbelow(True)

fig.savefig(os.path.join(HERE, "fig_c1_mechanism.pdf"))
plt.close(fig)
print("fig_c1_mechanism.pdf")
