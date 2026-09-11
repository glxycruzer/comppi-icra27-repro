# CoMPPI (ICRA 2027) — reproducibility bundle

Companion material for *Exploration Starvation and Mode Averaging in
Acceleration-Limited Critic-Shaped MPPI: Lightweight Remedies Validated on an Embedded AMR*
(CoMPPI, "Coherent MPPI").

Repository: https://github.com/glxycruzer/comppi-icra27-repro (this bundle, versioned; the
`v1.0-submission` tag is the state cited in the submitted manuscript).

**What is and is not here.** The CoMPPI controller source is proprietary
(SK intellix) and is **not** included. Everything else needed to reproduce the
baselines and the experimental protocol, and to audit every number in the
paper, is: the simulation harness and world generator, every CoMPPI / DWB /
Nav2-MPPI parameter file used in simulation and on the robot, the
field-campaign recording and analysis scripts, and the raw per-trial results of
all 4,475 simulation and 240 hardware trials.

With this bundle a reader can (a) re-run every DWB and Nav2-MPPI simulation
trial in the paper on identical worlds and seeds, (b) recompute every table,
figure, and confidence interval from the raw CSVs, and (c) run the full
protocol against their own controller by dropping a Nav2 controller plugin into
`sim/sim_params.yaml`. Re-running the CoMPPI trials requires the CoMPPI plugin
binary, which is not distributed.

**Redactions.** In the robot configuration files, the names of internal
(non-Nav2) plugins, topics, and packages of the production navigation stack
are replaced by `REDACTED_*` or `<...>` placeholders. Only the
`controller_server` and `local_costmap` sections of the robot configuration
are included; they are identical across the three controllers except for the
`FollowPath` block, which is the property the comparison rests on. The CoMPPI
plugin identifier appears as `comppi_controller::CoMPPIController`.
Controller ids in the data are `comppi`, `comppi_iid`, `comppi_colored`,
`comppi_b<B>`, `comppi_nodetour`, `comppi_noanneal`, `comppi_bc<cost>`,
`comppi_rs<range>`, `comppi_iid_{nolimit,amax1,symacc,l010,l003}` (C1
conjunction test), `comppi_dos{2,4}` (detour obstacle scale), `dwb`, `mppi`,
`mppi_tuned`.

## Layout

```
sim/            Simulation harness (ROS 2 Humble + Nav2 controller_server)
  worlds.py       world generator: open, clutter, gap, blocked, blocked_sym,
                  blocked_off<cm> (asymmetry sweep), crossing (dynamic box)
  batch_run.py    Monte-Carlo driver: worlds x seeds x controllers -> results.csv (--ess-log: per-cycle ESS/detour log)
  analyze_rho.py  success / ESS-in-detour / detour-flag toggles per controller from a results dir
  run_trial.py    sends the planned path as a FollowPath action goal
  kin_sim.py      kinematic base stand-in (unicycle, publishes TF/odom, logs)
  dyn_obstacle.py live-updating map publisher for the crossing world
  aggregate.py    per-(world,controller) summary of a results.csv
  decomp_run.py   per-sample critic-cost dump run (C1 mechanism figure)
  sim_params.yaml CoMPPI configuration (all parameters, including ablation knobs)
  params_dwb.yaml, params_mppi.yaml, params_mppi_tuned.yaml   baselines
  profile_assets/ on-target timing run (params per batch size, script, raw results)
robot/          Hardware campaign tooling (runs on the robot; ROS 2 Humble)
  record_trial.sh, swap_controller.sh, extract_bag.py, analyze_trials.py
  course_goals.yaml, courses.md, README_field_campaign.md
  controllers/    controller_server + local_costmap sections of the three
                  MATCHED robot configs (comppi, mppi, dwb) plus the C3
                  "before" variant (comppi_c3before) and the 2x detour-weight
                  validation variant (comppi_dos2, one added line). The local_costmap
                  sections are identical across controllers; only the
                  FollowPath block differs. Internal plugin and topic names
                  are redacted (see below).
data/           Raw per-trial results (CSV) behind every number in the paper
figures/        Scripts that turn data/ into the paper's figures and statistics
```

## Data dictionary

Simulation results (`campaign.csv`, `sym_ablation.csv`, `noise_comparison.csv`,
`bsweep.csv`, `mppi_tuned.csv`, `asym_sweep.csv`, `thresh_sweep.csv`,
`dyn/results.csv`), one row per trial:

| column | meaning |
|---|---|
| world, seed, controller | scenario id; `controller` names a `CONTROLLERS` entry in `batch_run.py` |
| success | reached goal (FollowPath status 4) and no collision |
| collision | min clearance below the robot radius (static), or contact while moving (dynamic) |
| ttg | time-to-goal (s), first to last motion of the kinematic base |
| goal_err, path_len, min_clear | m |
| rms_dv | commanded-velocity jitter RMS (m/s) |
| vmax | peak forward speed (m/s) |
| min_dyn_clear, min_dyn_clear_any | crossing world only: min clearance to the moving box while the robot moves / at any time (m; negative = overlap) |
| note | driver result string, e.g. `RESULT: SUCCESS (status=4)`, `dyn_collision` |

Which file supports which paper result:

| file | paper |
|---|---|
| campaign.csv | main study: CoMPPI / Nav2-MPPI / DWB at 100 seeds per world (1,200 trials) plus the ablation variants at 25 seeds (300) — success figure, sim TTG table, ablation numbers |
| campaign_25seed.csv | the original 25-seed campaign (600 trials) before the extension; its worlds were generated before `worlds.py` became deterministic and cannot be regenerated bit-for-bit (statistics unaffected: every controller saw the same worlds within the run) |
| sym_ablation.csv | perfectly symmetric blockage (ablation figure right) |
| noise_comparison.csv | block vs colored vs i.i.d. noise (C1). The colored variant replaces the block draw by IDFT(DFT(i.i.d.) * f^(-beta/2)) with the f=0 bin clamped to f=1, rescaled to RMS sigma, mean kept (beta = 2) |
| bsweep.csv | block-size sweep (C1) |
| decomp/*.csv | per-sample critic decomposition (C1 mechanism figure) |
| c1_block.csv, c1_iid.csv | velocity traces for the C1 collapse figure |
| mppi_tuned.csv | Nav2-MPPI retuned for the blocked world (100 seeds per world; wider noise, stronger obstacle repulsion, weaker path critics, temperature unchanged) |
| asym_sweep.csv | asymmetry sweep (C2) |
| thresh_sweep.csv | blockage-detection threshold sensitivity |
| dyn/ | dynamic crossing scenario, incl. per-run robot and box trajectories |
| c1_condition.csv | C1 conjunction test: i.i.d. noise with rollout accel limit removed / symmetric / loosened, and sharper lambda (Table: breaking the C1 conjunction) |
| rollouts_blocked_sym_s0_{sharp,plain}.csv, traj_blocked_sym_s0_{sharp,plain}.csv | C2 mechanism figure: every 4th sampled rollout with softmax weights on the first six detour cycles (sharpened vs plain temperature runs) and the executed paths |
| ess/ | per-cycle effective sample size 1/sum(w^2) of the softmax weights, one open-world run each for block and i.i.d. noise (columns: ess, detour_flag) |
| detour_obstacle_scale.csv | detour-mode obstacle-weight remedy (x2, x4) on crossing and blocked worlds |
| sigma_sweep.csv | C1 sigma sweep: i.i.d. noise at sigma_v in {0.03, 0.05, 0.10, 0.20, 0.30}, 15 seeds x open/gap/blocked (225 trials, all stall) |
| rho_sweep.csv, rho_sweep_summary.txt | C2 sharpening-ratio sweep rho in {0.10, 0.25, 0.50, 1.0} on blocked_sym, 15 seeds; summary adds per-run ESS in detour mode and detour-flag toggle counts |
| dyn_flicker.csv | 15 further crossing-world runs of the default controller recorded with the per-cycle log (detour-flag toggles) |
| ess_logs/ | per-cycle `ess,detour_flag` for every rho-sweep and dyn_flicker trial (`batch_run.py --ess-log`) |
| nav2_patch_limit.csv, nav2_patch_temperature.csv | C1 external-validity test: Nav2-MPPI Humble 1.1.20 built from source with a 34-line patch adding the paper's Eq. (1) rate limit inside its rollouts (+ optional block noise, + ESS log); open world, 15 seeds per cell. `mppi_patch` = patch compiled in but disabled (control), `_lim` = our asymmetric limits, `_sym` = symmetric, `_block` = limits + block noise B=8, `_t10`/`_t30` = temperature 1.0/3.0. The patch is `sim/nav2_mppi_humble_c1_patch.diff` (see below) |
| ess_logs_nav2/ | per-cycle ESS of the patched Nav2-MPPI runs (nav2_patch_temperature campaign) |
| lambda_sweep.csv, ess_logs_lambda/ | C1 temperature leg inside CoMPPI: i.i.d. noise at lambda in {0.10, 0.15, 0.20, 0.25, 0.30, 0.35}, open world, 15 seeds, with per-cycle ESS (boundary between lambda 0.15 and 0.20) |
| adaptive_temperature.csv, adaptive_temperature_summary.txt, ess_logs_adaptive/ | rebuttal Q1: i.i.d. noise with an ESS-targeting adaptive lambda (5/10/20 % of K; controller options `ess_target_frac`) and with cost normalization (`cost_normalize`), open/gap/blocked x 15 seeds; per-cycle ESS logs |
| headroom_kt.csv | what the latency margin buys: K=2048 and T=48 vs the default on clutter/blocked/crossing, 15 seeds |
| dyn_mppi_tuned.csv | the blocked-world retune of Nav2-MPPI on the crossing world, 15 seeds |
| dyn_mppi_rep2.csv, dyn_mppi_rep3.csv | two further 15-seed crossing runs of Nav2-MPPI and its retune (the crossing is real-time, so runs differ; pooled contact counts in the paper are over dyn/, these two files and, for CoMPPI, dyn_flicker.csv and headroom_kt.csv) |
| dyn_variants_rep2.csv, dyn_variants_rep3.csv | two further 15-seed crossing runs of the 2x / 4x detour-obstacle-weight, no-sharpening and no-gating variants (pooled with dyn/ and detour_obstacle_scale.csv in the paper: 17/45, 18/45, 20/45, 15/45 contacts) |
| kappa_scope.csv | C1 scope: i.i.d. noise with a_max raised to 0.5 and 1.0 m/s^2 at the same 4:1 asymmetry (kappa = a_max*dt/sigma_v = 0.25, 0.5), open world, 15 seeds |
| nav2_gate.csv, nav2_gate_open.csv | reviewer experiment: stock Nav2-MPPI with its path-align gate at occupancy ratio 0.01 (`mppi_align001`), temperature 0.1 (`mppi_t01`), or both, on blocked/clutter/gap, 15 seeds |
| asymmetry_ratio2.csv | C1 asymmetry ratio 2 (a_max 0.25, a_min -0.5), i.i.d. noise, open/gap/blocked x 15 seeds (all stall) |
| (figures/contact_speed.py) | robot / box / closing speed at first contact for every crossing-world run, harness contact rule |
| cpu_profile/ | same-board compute comparison Nav2-MPPI vs CoMPPI (pinned 2.3 GHz, stack off): per-second CPU csv, summaries, CoMPPI internal timer; run1_lowclock/ = discarded idle-clock attempt |
| comppi_profile_*.txt | on-target per-cycle solve times per batch size |

Hardware results (`hw_results.csv`), one row per trial (240 clean trials: the 210-trial three-controller campaign plus the 30-trial `comppi_dos2` validation cell, surprise course, detour_obstacle_scale 2.0):

| column | meaning |
|---|---|
| bag, course, controller | `<course>__<controller>__t<n>` |
| success | closest map-frame approach to the goal within the course tolerance |
| ttg | command-derived time-to-goal: first forward command (after the RotationShim's in-place heading alignment, ~6 s in every cell) to last command |
| rot_s | duration of that initial in-place alignment (s) |
| cruise, cruise_p90 | median / 90th-pct pose-derived speed while moving (m/s) |
| min_clear | min lidar range while moving (m) |
| cmd_jitter | RMS step change of the controller's commanded linear velocity, native `/cmd_vel_nav` (m/s) |
| accel_rms, rms_dv, stall_frac, goal_err, path_len, duration | as named |

`hw_c3_before.csv` / `hw_c3_after.csv` are the recorded runs behind the
velocity-feedback (C3) figure. `hardware_quarantined_trials.txt` lists 13
surprise-course bags excluded from the campaign and re-recorded: 11 ran under
the platform's own floor-sensor speed governor (realized speed clamped to
0.30 m/s against a 0.40 m/s command, detected by comparing `/cmd_vel` with
`/cmd_vel_nav`), one was empty, one was borderline. Raw rosbags (≈0.9 GB) and the raw surprise-course video clips are not in
this bundle and are available on request; the accompanying video submitted
with the paper (`comppi_icra2027_video.mp4`, 149 s) contains the real-time
three-way side-by-side of those clips plus simulation animations rendered from
the data in this bundle (`video/make_video.py` in the paper repository). `robot_photo.jpg` is the
platform photo used in the hardware figure; `env_{corridor,passage,surprise}.JPEG`
are the course photos (`figures/gen_env_figure.py`).

## Reproducing the simulation baselines

Requirements: ROS 2 Humble with Nav2 (`nav2_controller`, `nav2_map_server`,
`nav2_lifecycle_manager`, `nav2_dwb_controller`, `nav2_mppi_controller`),
Python 3 with `numpy`, `scipy`, `pyyaml`. Use an isolated `ROS_DOMAIN_ID`.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
cd sim
python3 batch_run.py --controllers dwb,mppi,mppi_tuned \
    --worlds open,clutter,gap,blocked --seeds 25 --outdir results/
python3 aggregate.py results/results.csv
```

Worlds are deterministic given (kind, seed): the per-kind RNG offset is a CRC of the kind name (earlier runs used Python's salted `hash()`; see campaign_25seed.csv). The asymmetry and crossing worlds draw from a seed-only RNG so paired comparisons hold. `batch_run.py`
launches map_server (or `dyn_obstacle.py` for `crossing`), `controller_server`
with the chosen parameter file, the kinematic base, and a lifecycle manager,
then sends the A*-planned path as a `FollowPath` goal and scores the logged
trajectory. Each trial takes ~35 s.

### Reproducing C1 inside Nav2-MPPI (the patched baseline)

`sim/nav2_mppi_humble_c1_patch.diff` is a 34-line, off-by-default patch to
`nav2_mppi_controller` (Humble, tag 1.1.20) that adds three parameters to the
`FollowPath` block: `rollout_ax_max` / `rollout_ax_min` (m/s^2; when
`rollout_ax_max > 0` the sampled forward speed is box-limited to
[vx_min, vx_max] and then rate-limited step by step inside the rollouts,
i.e. the paper's Eq. (1)) and `noise_block_steps` (B > 1 holds each noise draw
for B steps). It also logs the per-cycle effective sample size to the file
named by `MPPI_ESS_LOG` when that variable is set (the harness sets it with
`--ess-log`).

```bash
git clone --depth 1 --branch 1.1.20 https://github.com/ros-navigation/navigation2.git
cd navigation2 && git apply /path/to/nav2_mppi_humble_c1_patch.diff && cd ..
colcon build --packages-select nav2_mppi_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash        # overlay: replaces the apt plugin for this shell only
cd sim && python3 batch_run.py --ess-log \
    --controllers mppi_patch,mppi_patch_lim,mppi_patch_lim_t10,mppi_patch_lim_t30,mppi_patch_t30 \
    --worlds open --seeds 15 --outdir results_nav2_patch/
```

`mppi_patch` (patch compiled in, disabled) must reproduce the stock `mppi`
rows of `campaign.csv` bit-for-bit for the same seeds; `mppi_patch_lim_t30`
reproduces the C1 stall (0.025 m/s).

To run a different controller, add an entry to `CONTROLLERS` in
`batch_run.py` pointing at a parameter file whose `FollowPath` block loads your
plugin.

## Reproducing the figures and statistics

```bash
python3 -m venv figvenv && . figvenv/bin/activate && pip install "numpy<2" matplotlib
cd figures && for f in gen_*.py; do python3 $f; done
```

`gen_stats.py` prints every success rate with its Wilson 95% CI and every
time-to-goal with its n, exactly as quoted in the paper.

## Hardware protocol

See `robot/README_field_campaign.md` and `robot/courses.md`. The three robot
configurations differ only in the `FollowPath` block (verified by md5 on the
`local_costmap` sections). Recording, extraction and analysis are scripted;
`analyze_trials.py` documents the success and time-to-goal definitions in
its comments, including the two measurement corrections made during the
campaign (command-gated motion window; command-derived time-to-goal).

## License and contact

License: to be set by SK intellix before release (data and scripts only; the
controller remains proprietary). Corresponding author: Jinseop Lee, SK intellix.
