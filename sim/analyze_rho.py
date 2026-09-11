#!/usr/bin/env python3
"""rho sweep + flicker: per controller, success, TTG, per-cycle ESS in detour mode, detour-flag toggles."""
import csv, glob, os, statistics as st, sys
out = sys.argv[1] if len(sys.argv) > 1 else "results_rho"
rows = list(csv.DictReader(open(os.path.join(out, "results.csv"))))
by = {}
for r in rows:
    by.setdefault(r["controller"], []).append(r)
for c, rs in sorted(by.items()):
    succ = [r for r in rs if r["success"] == "True"]
    ess_det, ess_nom, toggles, det_frac = [], [], [], []
    for r in rs:
        f = os.path.join(out, f"{r['world']}_s{r['seed']}_{c}", "ess.csv")
        if not os.path.exists(f):
            continue
        cyc = [(float(a), int(b)) for a, b in (l.strip().split(",") for l in open(f) if l.strip())]
        flags = [b for _, b in cyc]
        toggles.append(sum(1 for i in range(1, len(flags)) if flags[i] != flags[i-1]))
        det_frac.append(sum(flags) / len(flags))
        ess_det += [e for e, b in cyc if b == 1]
        ess_nom += [e for e, b in cyc if b == 0]
    print(f"{c:18s} n={len(rs):2d} succ={len(succ):2d} ttg={st.median([float(r['ttg']) for r in succ]) if succ else float('nan'):5.1f}"
          f" | ESS detour med={st.median(ess_det) if ess_det else float('nan'):6.1f} nominal med={st.median(ess_nom) if ess_nom else float('nan'):6.1f}"
          f" | detour toggles/run med={st.median(toggles) if toggles else float('nan'):4.1f} max={max(toggles) if toggles else 0:3d} | detour frac={st.mean(det_frac) if det_frac else 0:.2f}")
