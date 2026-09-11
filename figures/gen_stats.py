#!/usr/bin/env python3
"""Wilson 95% CIs on success rates + sample sizes for success-conditioned
metrics, for both the simulation campaign and the hardware campaign. Emits the
numbers to paste into the paper (and flags the n=4 survivorship case).
"""
import csv
import math
import os
import statistics as st
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
Z = 1.959963985  # 95%


def wilson(s, n):
    if n == 0:
        return (float("nan"), float("nan"), float("nan"))
    p = s / n
    d = 1 + Z * Z / n
    c = (p + Z * Z / (2 * n)) / d
    h = (Z / d) * math.sqrt(p * (1 - p) / n + Z * Z / (4 * n * n))
    return p, max(0.0, c - h), min(1.0, c + h)


def summarize(path, group_cols, worlds, ctrls, wlabel):
    rows = defaultdict(list)
    with open(path) as f:
        for r in csv.DictReader(f):
            rows[(r[group_cols[0]], r[group_cols[1]])].append(r)
    print(f"\n=== {os.path.basename(path)} ===")
    print(f"{'cell':<26}{'n':>4}{'succ':>5}  {'rate [Wilson 95% CI]':<26}"
          f"{'TTG mean±sd (n_succ)':<22}")
    for w in worlds:
        for c in ctrls:
            rs = rows.get((w, c), [])
            n = len(rs)
            if n == 0:
                continue
            s = sum(1 for r in rs if r["success"] == "True")
            p, lo, hi = wilson(s, n)
            succ_ttg = [float(r["ttg"]) for r in rs
                        if r["success"] == "True" and r.get("ttg") not in ("", "None")]
            ttg = (f"{st.mean(succ_ttg):.1f}±{(st.stdev(succ_ttg) if len(succ_ttg)>1 else 0):.1f}"
                   f" (n={len(succ_ttg)})") if succ_ttg else "-- (n=0)"
            flag = "  <-- SMALL n" if 0 < len(succ_ttg) <= 5 else ""
            print(f"{wlabel.get(w,w)+' / '+c:<26}{n:>4}{s:>5}  "
                  f"{100*p:5.0f}% [{100*lo:4.0f},{100*hi:4.0f}]        "
                  f"{ttg:<22}{flag}")


# simulation campaign (25 seeds/world)
summarize(os.path.join(HERE, "data", "campaign.csv"),
          ("world", "controller"),
          ["open", "clutter", "gap", "blocked"],
          ["comppi", "mppi", "dwb"],
          {"open": "Open", "clutter": "Clutter", "gap": "Gap", "blocked": "Blocked"})

# hardware campaign (10 trials/course)
summarize(os.path.join(HERE, "data", "hw_results.csv"),
          ("course", "controller"),
          ["corridor", "passage", "surprise"],
          ["comppi", "mppi", "dwb", "comppi_dos2"],
          {"corridor": "Corridor", "passage": "Passage", "surprise": "Surprise"})
