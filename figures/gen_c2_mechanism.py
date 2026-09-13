#!/usr/bin/env python3
"""C2 mechanism figure: mode averaging vs. commitment over the first detour
cycles on the perfectly symmetric blocked world (seed 0).

Background: true occupancy grid. Thin lines: every 4th sampled rollout on the
FIRST detour cycle of the default (sharpened, 0.25 lambda) run, colored by
softmax weight. Thick lines: the weighted-mean sampled path on detour cycles
1..6 for the default run (blue, darker = later cycle) and for the run without
temperature sharpening (orange). Averaging keeps the plain-lambda mean on the
blocked path into the box cycle after cycle; sharpening lets the mean drift
to one side and the warm start compounds it.

Inputs: data/rollouts_blocked_sym_s0_{sharp,plain}.csv (controller dumps),
        data/traj_blocked_sym_s0_{sharp,plain}.csv (kinematic base logs)
"""
import csv
import io
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
import glob as _glob
# the controller package's simval test dir (name-agnostic so released copies carry no internal names)
for _d in _glob.glob(os.path.join(HERE, "..", "..", "*_controller", "test", "simval")):
    sys.path.insert(0, _d)
sys.path.insert(0, os.path.join(HERE, "..", "release", "comppi-icra27-repro", "sim"))
import worlds  # noqa: E402

plt.rcParams.update({
    "pdf.fonttype": 42, "ps.fonttype": 42,  # TrueType, not Type 3 (PaperPlaza requirement)
    "font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8,
    "legend.fontsize": 6.5, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "axes.linewidth": 0.6, "figure.dpi": 200,
    "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
})


def load(path):
    """-> list of cycles: dict(meta, X, Y, w_active, w_plain)"""
    cycles = []
    block = []
    meta = None
    for line in open(path):
        if line.startswith("#"):
            if block:
                cycles.append((meta, block))
            meta = dict(kv.split("=") for kv in line.strip("# \n").split())
            block = []
        else:
            block.append(line)
    if block:
        cycles.append((meta, block))
    out = []
    for meta, lines in cycles:
        rows = list(csv.DictReader(io.StringIO("".join(lines))))
        T = sum(1 for k in rows[0] if k.startswith("x"))
        X = np.array([[float(r[f"x{t}"]) for t in range(T)] for r in rows])
        Y = np.array([[float(r[f"y{t}"]) for t in range(T)] for r in rows])
        wa = np.array([float(r["w_sharp"]) for r in rows]); wa /= wa.sum()
        wp = np.array([float(r["w_plain"]) for r in rows]); wp /= wp.sum()
        out.append(dict(meta=meta, X=X, Y=Y, wa=wa, wp=wp))
    return out


sharp = load(os.path.join(HERE, "data", "rollouts_blocked_sym_s0_sharp.csv"))
plain = load(os.path.join(HERE, "data", "rollouts_blocked_sym_s0_plain.csv"))
grid = worlds.make_world("blocked_sym", 0)["grid_true"]

fig, ax = plt.subplots(figsize=(3.4, 2.05))
ax.imshow(~grid, cmap="gray", origin="lower",
          extent=[0, worlds.N * worlds.RES, 0, worlds.N * worlds.RES],
          interpolation="nearest", vmin=0, vmax=1)

# fan of first-cycle rollouts (sharpened run), colored by active weight
c0 = sharp[0]
cmap = plt.get_cmap("viridis")
norm = matplotlib.colors.LogNorm(vmin=max(c0["wa"].min(), 1e-6), vmax=c0["wa"].max())
for i in np.argsort(c0["wa"]):
    ax.plot(c0["X"][i], c0["Y"][i], color=cmap(norm(c0["wa"][i])), linewidth=0.4, alpha=0.6)

# executed trajectories of the two runs (kinematic base logs = ground truth in sim)
def traj(name):
    rows = list(csv.DictReader(open(os.path.join(HERE, "data", f"traj_blocked_sym_s0_{name}.csv"))))
    return [float(r["x"]) for r in rows], [float(r["y"]) for r in rows]
tx, ty = traj("plain")
ax.plot(tx, ty, color="#D55E00", linewidth=1.8, label="no sharpening: into the box, aborted")
tx, ty = traj("sharp")
ax.plot(tx, ty, color="#0072B2", linewidth=1.8, label=r"sharpened $0.25\lambda$: detours")

la = c0["meta"]
ax.plot([float(la["la_x"])], [float(la["la_y"])], marker="*", color="k", markersize=7,
        linestyle="none", label="follow target")
x0, y0 = c0["X"][:, 0].mean(), c0["Y"][:, 0].mean()
ax.plot([x0], [y0], marker="o", color="k", markersize=3.5, linestyle="none")
ax.set_xlim(x0 - 0.6, x0 + 2.6)
ax.set_ylim(5.0 - 0.9, 5.0 + 0.9)
ax.set_aspect("equal")
ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
ax.legend(frameon=False, fontsize=5.8, loc="upper center", bbox_to_anchor=(0.5, -0.28), ncol=3, handlelength=1.4, columnspacing=0.9)
sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm); sm.set_array([])
cb = fig.colorbar(sm, ax=ax, fraction=0.04, pad=0.02)
cb.set_label("softmax weight, cycle 1 ($0.25\\lambda$)", fontsize=6.2); cb.ax.tick_params(labelsize=6)
fig.savefig(os.path.join(HERE, "fig_c2_mechanism.pdf"))
plt.close(fig)

n = min(len(sharp), len(plain), 6)
for lab, cyc in (("sharp", sharp), ("plain", plain)):
    ends = []
    for c in cyc[:n]:
        my = (c["wa"][:, None] * c["Y"]).sum(0)
        ends.append(my[-1] - 5.0)
    ess = [1 / (c["wa"] ** 2).sum() for c in cyc[:n]]
    print(f"{lab:5s} mean-path end y-offset per cycle: {' '.join(f'{e:+.2f}' for e in ends)} | ESS/{len(cyc[0]['wa'])}: {' '.join(f'{e:.1f}' for e in ess)}")
print("fig_c2_mechanism.pdf")
