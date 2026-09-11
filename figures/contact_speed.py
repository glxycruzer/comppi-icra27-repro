#!/usr/bin/env python3
"""Robot speed, box speed and closing speed at the moment of first contact in the
crossing-world runs (robot radius 0.19 m, box half-size 0.25 m; contact = centre
distance < 0.44 m while the robot moves > 0.05 m/s, as scored by the harness)."""
import csv, glob, os, sys, statistics as st, math
D = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__)) + "/data/dyn"
R, HALF = 0.19, 0.25
out = {}
for d in sorted(glob.glob(os.path.join(D, "crossing_s*_*"))):
    ctrl = os.path.basename(d).split("_", 2)[2]
    try:
        tr = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(d + "/traj.csv"))]
        bx = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(d + "/dyn.csv"))]
    except FileNotFoundError:
        continue
    # harness rule: box pose in force = LAST published pose; contact = clearance
    # to the square minus robot radius < -RES/2 (2.5 cm) while the robot moves > 0.05 m/s
    import bisect
    bt = [b["t_abs"] for b in bx]
    hit = None
    for r in tr:
        k = max(bisect.bisect_right(bt, r["t_abs"]) - 1, 0)
        x, y = bx[k]["bx"], bx[k]["by"]
        k0 = max(k - 1, 0); dt = bx[k]["t_abs"] - bx[k0]["t_abs"]
        vb = math.hypot(bx[k]["bx"] - bx[k0]["bx"], bx[k]["by"] - bx[k0]["by"]) / dt if dt > 0 else 0.0
        dx = max(abs(r["x"] - x) - HALF, 0.0); dy = max(abs(r["y"] - y) - HALF, 0.0)
        c = math.hypot(dx, dy) - R
        if c < -0.025 and r["v"] > 0.05:
            hit = (r["v"], vb, math.hypot(r["v"], vb)); break
    out.setdefault(ctrl, []).append(hit)
for c, hits in sorted(out.items()):
    h = [x for x in hits if x]
    if not h:
        print(f"{c:18s} n={len(hits):2d} contacts=0"); continue
    print(f"{c:18s} n={len(hits):2d} contacts={len(h):2d}  robot v at contact: median {st.median(v for v,_,_ in h):.2f} (min {min(v for v,_,_ in h):.2f}, max {max(v for v,_,_ in h):.2f}) m/s | "
          f"box v {st.median(b for _,b,_ in h):.2f} m/s | closing {st.median(cs for *_,cs in h):.2f} m/s")
