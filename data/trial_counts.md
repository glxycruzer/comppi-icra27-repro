# Simulation trial accounting (4,475 trials)

| study | file | cells | trials |
|---|---|---|---|
| main comparison, 3 controllers x 4 worlds x 100 seeds | campaign.csv | 12 | 1,200 |
| ablation variants (iid, nodetour, noanneal) x 4 worlds x 25 seeds | campaign.csv | 12 | 300 |
| perfectly symmetric blockage (default vs no-sharpening, 25 seeds) | sym_ablation.csv | 2 | 50 |
| noise comparison block / colored / iid x 4 worlds x 15 | noise_comparison.csv | 12 | 180 |
| block-size sweep B in {2,4,16,32} x 2 worlds x 15 | bsweep.csv | 8 | 120 |
| retuned Nav2-MPPI x 4 worlds x 100 (15 in the first draft, extended to 100 for the final version) | mppi_tuned.csv | 4 | 400 |
| asymmetry sweep 7 offsets x 3 variants x 15 | asym_sweep.csv | 21 | 315 |
| detection thresholds 6 settings x 3 worlds x 15 | thresh_sweep.csv | 18 | 270 |
| dynamic crossing 5 controllers x 15 | dyn/results.csv | 5 | 75 |
| C1 conjunction 5 variants x 3 worlds x 15 | c1_condition.csv | 15 | 225 |
| detour obstacle scale (2x, 4x) crossing + blocked, 15 / 25 | detour_obstacle_scale.csv | 4 | 80 |
| **subtotal, first submission draft (retune at 15 seeds)** | | | **2,875** |
| sigma sweep 5 sigma x 3 worlds x 15 | sigma_sweep.csv | 15 | 225 |
| rho sweep 4 rho x 15 (blocked_sym) | rho_sweep.csv | 4 | 60 |
| crossing flicker run, default controller x 15 | dyn_flicker.csv | 1 | 15 |
| patched Nav2-MPPI, limit variants x 15 (open) | nav2_patch_limit.csv | 4 | 60 |
| patched Nav2-MPPI, temperature variants x 15 (open) | nav2_patch_temperature.csv | 5 | 75 |
| lambda sweep, i.i.d., lambda in {0.10,0.15,0.20,0.25,0.30,0.35} x 15 (open) | lambda_sweep.csv | 6 | 90 |
| adaptive temperature: ESS target 5/10/20 % + cost normalization, i.i.d., 3 worlds x 15 | adaptive_temperature.csv | 12 | 180 |
| headroom: K=2048, T=48 (and default) x clutter/blocked/crossing x 15 | headroom_kt.csv | 9 | 135 |
| retuned Nav2-MPPI on the crossing world x 15 | dyn_mppi_tuned.csv | 1 | 15 |
| crossing repeats: Nav2-MPPI + retune, 2 further runs x 15 | dyn_mppi_rep2.csv, dyn_mppi_rep3.csv | 4 | 60 |
| crossing repeats: 2x, 4x, no-sharpening, no-gating variants, 2 further runs x 15 | dyn_variants_rep2.csv, dyn_variants_rep3.csv | 8 | 120 |
| C1 scope: a_max 0.5 / 1.0 at 4:1 asymmetry (kappa 0.25 / 0.5), i.i.d., open x 15 | kappa_scope.csv | 2 | 30 |
| Nav2-MPPI own gate at ratio 0.01, temperature 0.1, both x blocked/clutter/gap x 15 | nav2_gate.csv | 9 | 135 |
| Nav2-MPPI temperature 0.1, open world x 15 (Table II row) | nav2_gate_open.csv | 1 | 15 |
| C1 asymmetry ratio 2 (a_min = -0.5), i.i.d., open/gap/blocked x 15 | asymmetry_ratio2.csv | 3 | 45 |
| retune extension: seeds 15-99 x 4 worlds (already counted in the 400 above) | mppi_tuned.csv | 4 | 340 |
| **total** | | | **4,475** |

Rows can be re-derived with `wc -l` on each file (minus the header); `gen_stats.py` prints the same totals.

# Hardware trial accounting (240 trials, `hw_results.csv`)

| cell | trials |
|---|---|
| corridor x {comppi, mppi, dwb} | 3 x 30 |
| passage x {comppi, mppi, dwb} | 3 x 10 |
| surprise x {comppi, mppi, dwb} | 3 x 30 |
| surprise x comppi_dos2 (detour_obstacle_scale 2.0 validation) | 30 |
| **total** | **240** |

The two single-run C3 recordings (`hw_c3_before.csv`, `hw_c3_after.csv`) are not campaign trials and are not counted.
