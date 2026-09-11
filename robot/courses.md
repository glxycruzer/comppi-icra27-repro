# Field-campaign courses

Three courses, chosen to exercise the same behaviors the simulation study
isolated, so the hardware results map onto Fig. success / smoothness / ablation.
Keep start/goal poses fixed per course across all controllers and trials (mark
them on the floor) — the comparison is only fair if the task is identical.

| Course | What it tests | Layout | Success criterion |
|---|---|---|---|
| **corridor** | cruise tracking + smoothness | ~6–8 m straight/gently curved clear run | reaches goal; compare cruise speed + jitter |
| **passage** | clearance shaping (C1 quality) | one narrow gap ~0.7–1.0 m between obstacles | passes the gap without contact |
| **surprise** | unmapped-obstacle detour (C2) | box placed on the planned path after the map was built | detours and reaches goal; DWB expected to stall/recover |

Per course: **≥10 trials × 3 controllers** (comppi / dwb / mppi) = 90 trials
minimum. If robot time is tight, protect **surprise** (the C2 story) and
**corridor** (speed/smoothness) first; `passage` can drop to 5 trials.

## Protocol notes

- Same start/goal for every trial of a course. Undock to the same spot.
- The **surprise** course is the key differentiator: place the obstacle only
  after the global map/plan exists, so it is genuinely unmapped. Record whether
  the robot detours (comppi), stalls then triggers BT recovery/replan (DWB), or
  clips the obstacle.
- Note anything unusual in the per-trial operator note — it ends up in the
  manifest and is searchable during analysis.
- For the paper video: keep one clean `surprise` run per controller with the
  local-plan topic recorded (it's in the bag).
