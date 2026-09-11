#!/usr/bin/env python3
"""Hardware course photos: corridor, passage, surprise (data/env_*.JPEG).
Each portrait photo is cropped to the lower 70 % (the course itself) and
downsampled so the PDF stays small."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image, ImageOps

HERE = os.path.dirname(os.path.abspath(__file__))
plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"],
                     "font.size": 7, "savefig.bbox": "tight", "savefig.pad_inches": 0.01})
courses = [("corridor", "(a) corridor"), ("passage", "(b) passage"), ("surprise", "(c) surprise")]
fig, axs = plt.subplots(1, 3, figsize=(3.4, 1.12))
fig.subplots_adjust(wspace=0.03, left=0, right=1, top=1, bottom=0)
for ax, (name, lab) in zip(axs, courses):
    im = ImageOps.exif_transpose(Image.open(os.path.join(HERE, "data", f"env_{name}.JPEG"))).convert("RGB")
    w, h = im.size
    im = im.crop((0, int(0.30 * h), w, h)).resize((520, int(520 * 0.70 * h / w)), Image.LANCZOS)
    ax.imshow(im, interpolation="lanczos")
    ax.set_axis_off()
    ax.text(0.03, 0.03, lab, transform=ax.transAxes, fontsize=7, color="white",
            va="bottom", ha="left", bbox=dict(facecolor="black", alpha=0.55, pad=1.5, lw=0))
fig.savefig(os.path.join(HERE, "fig_env.pdf"), dpi=200)
print("fig_env.pdf")
