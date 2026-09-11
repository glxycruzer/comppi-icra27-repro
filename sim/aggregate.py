#!/usr/bin/env python3
"""Summarize results.csv per (world, controller)."""
import csv
import statistics as st
import sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else "results/results.csv"
groups = defaultdict(list)
with open(path) as f:
    for r in csv.DictReader(f):
        groups[(r["world"], r["controller"])].append(r)

print(f"{'world':<9} {'controller':<17} {'n':>3} {'succ%':>6} {'coll':>4} "
      f"{'ttg':>12} {'min_clear':>9} {'rms_dv':>7}")
for (world, ctrl), rows in sorted(groups.items()):
    n = len(rows)
    succ = [r for r in rows if r["success"] == "True"]
    coll = sum(1 for r in rows if r.get("collision") == "True")
    ttgs = [float(r["ttg"]) for r in succ if r.get("ttg")]
    clears = [float(r["min_clear"]) for r in rows if r.get("min_clear")]
    dvs = [float(r["rms_dv"]) for r in succ if r.get("rms_dv")]
    ttg = f"{st.mean(ttgs):5.1f}±{st.stdev(ttgs):4.1f}" if len(ttgs) > 1 else \
        (f"{ttgs[0]:5.1f}      " if ttgs else "     -      ")
    mc = f"{min(clears):9.2f}" if clears else "        -"
    dv = f"{st.mean(dvs):7.4f}" if dvs else "      -"
    print(f"{world:<9} {ctrl:<17} {n:>3} {100*len(succ)/n:>5.0f}% {coll:>4} "
          f"{ttg:>12} {mc} {dv}")
