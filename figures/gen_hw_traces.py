#!/usr/bin/env python3
"""Hardware figure: course layout (+ robot photo if provided) and the three
controllers' commanded-speed traces on the surprise course.

(a) Robot photo, if data/robot_photo.jpg exists (otherwise the panel is
    omitted and the layout widens).
(b) Schematic of the surprise course: 2 m corridor, ~9.3 m run, unmapped
    0.6 x 0.5 m box on the planned path. Drawn to scale from the course
    definition; robot-side positions are NOT plotted (the platform has no
    map-frame pose stream).
(c) Commanded linear speed (/cmd_vel_nav) vs time for one representative
    (median time-to-goal) surprise trial per controller, aligned at the first
    forward command. DWB's stall-recover-replan cycle shows as a sawtooth,
    CoMPPI's detour as a continuous curve.

Inputs: data/hw_results.csv, robot_bags/bags/<bag>/extracted.csv
"""
import csv
import os
import statistics as st

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

HERE = os.path.dirname(os.path.abspath(__file__))
BAGS = os.path.join(HERE, "..", "..", "..", "robot_bags", "bags")
PHOTO = os.path.join(HERE, "data", "robot_photo.jpg")
plt.rcParams.update({
    "pdf.fonttype": 42, "ps.fonttype": 42,  # TrueType, not Type 3 (PaperPlaza requirement)
    "font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8,
    "legend.fontsize": 6.5, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "axes.linewidth": 0.6, "figure.dpi": 200,
    "savefig.bbox": "tight", "savefig.pad_inches": 0.02,
})
C = {"comppi": "#0072B2", "mppi": "#009E73", "dwb": "#D55E00"}
LAB = {"comppi": "CoMPPI", "mppi": "Nav2-MPPI", "dwb": "DWB"}

# representative trial = median time-to-goal among successes, per controller
rows = [r for r in csv.DictReader(open(os.path.join(HERE, "data", "hw_results.csv")))
        if r["course"] == "surprise" and r["success"] == "True"]
pick = {}
for c in ("comppi", "mppi", "dwb"):
    rs = sorted((float(r["ttg"]), r["bag"]) for r in rows if r["controller"] == c)
    pick[c] = rs[len(rs) // 2][1]


def trace(bag):
    b = os.path.join(BAGS, bag)
    rs = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(os.path.join(b, "extracted.csv")))]
    t = [r["t"] for r in rs]; v = [r["cmd_vx"] for r in rs]
    # first sustained forward command (5 samples > 0.05) = end of RotationShim alignment
    i0 = next(i for i in range(len(v) - 5) if all(v[i + j] > 0.05 for j in range(5)))
    last = max(i for i, x in enumerate(v) if abs(x) > 0.02)
    return [x - t[i0] for x in t[i0:last + 1]], v[i0:last + 1]


have_photo = os.path.exists(PHOTO)
fig = plt.figure(figsize=(3.4, 2.3))
if have_photo:
    gsa = fig.add_gridspec(1, 4, left=0.003, right=0.997, top=0.93, bottom=0.60, wspace=0.03)
    gs = fig.add_gridspec(1, 1, left=0.15, right=0.985, top=0.50, bottom=0.19)
    # (a) 2x2 mosaic: platform photo + the three course photos (cropped to the course)
    from PIL import Image, ImageOps
    tiles = [("robot_photo.jpg", "platform"), ("env_corridor.JPEG", "corridor"),
             ("env_passage.JPEG", "passage"), ("env_surprise.JPEG", "surprise")]
    for k, (f, lab) in enumerate(tiles):
        axp = fig.add_subplot(gsa[0, k])
        im = ImageOps.exif_transpose(Image.open(os.path.join(HERE, "data", f))).convert("RGB")
        w, h = im.size
        if f.startswith("env"):
            im = im.crop((0, int(0.30 * h), w, h)); w, h = im.size
        # centre-crop to a 4:3 tile so the four cells match
        tw, th = (w, int(w * 0.75)) if h >= w * 0.75 else (int(h / 0.75), h)
        im = im.crop(((w - tw) // 2, (h - th) // 2, (w - tw) // 2 + tw, (h - th) // 2 + th)).resize((400, 300), Image.LANCZOS)
        axp.imshow(im); axp.set_axis_off()
        axp.text(0.04, 0.05, lab, transform=axp.transAxes, fontsize=6, color="white",
                 bbox=dict(facecolor="black", alpha=0.55, pad=1.2, lw=0))
        if k == 0:
            axp.set_title("(a) Platform and the three courses", fontsize=7.5, loc="left", x=0.0)
    axt = fig.add_subplot(gs[0, 0])
else:
    axt = fig.add_subplot(111)

for c in ("dwb", "mppi", "comppi"):
    t, v = trace(pick[c])
    axt.plot(t, v, color=C[c], linewidth=1.1, label=f"{LAB[c]} (trial {pick[c].split('__t')[1]})")
axt.set_xlabel("Time since first forward command (s)")
axt.set_ylabel("Commanded $v_x$ (m/s)")
axt.set_ylim(-0.02, 0.45)
axt.set_title(f"({'b' if have_photo else 'a'}) Commanded speed, median-TTG surprise trial per controller", fontsize=7.5)
axt.legend(frameon=False, fontsize=6.3, loc="upper center", bbox_to_anchor=(0.5, -0.43), ncol=3, columnspacing=1.0, handlelength=1.4)
axt.spines[["top", "right"]].set_visible(False)
axt.yaxis.grid(True, color="#EEEEEE", linewidth=0.5); axt.set_axisbelow(True)

fig.savefig(os.path.join(HERE, "fig_hw_traces.pdf"))
plt.close(fig)
print("picked:", pick, "| photo:", have_photo)
print("fig_hw_traces.pdf")
