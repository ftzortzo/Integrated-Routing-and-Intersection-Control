# Isolated intersection controller

Single-intersection harness for the lower layer of
*Integrated Routing and Intersection Control for Mixed Traffic* (arXiv 2604.13424).
State: validated (Sept 17, 2026) — 300 s runs: kinematics ratio 1.000, 0 collisions,
0 SUMO warnings, every stop-line crossing on green, rear-end barrier h >= 0 throughout.

## Provenance

| File | Source | Status |
|---|---|---|
| `cav_control_policy.m` | `FINAL\Final_play - Copy (5)` | modified — see "Changes" (crossing-time selection, profile, OC cases untouched) |
| `optimize_signal_plan.m` | `FINAL\Final_play - Copy (5)` | modified — clearance lost time, v_max, phase model |
| `run_from_matlab.m` | `FINAL\Final_play - Copy (5)` | reduced to one intersection; routing removed; APPLY fixed; logging; downstream policy |
| `presort.m` | `FINAL\Final_play - Copy (5)` | verbatim |
| `preliminaries.m` | `Simple example matlab - Copy - Copy` | step 0.1, ballistic, collision + message logging |
| `quickstart.net.xml` | `Simple example matlab - Copy - Copy` | internal lanes at 13.89 m/s |
| `quickstart.rou.xml` | `Simple example matlab - Copy - Copy` | CAV vType consistent with controller |
| `quickstart.sumocfg` | `Simple example matlab - Copy - Copy` | step 0.1 |
| `plot_trajectories.m` | `Simple example matlab - Copy - Copy` | verbatim |
| `check_kinematics.m`, `analyze_braking.m`, `audit_controller.m`, `report_collisions.m` | new | validation tools, run automatically |

## Run

Open a NEW MATLAB instance, `cd` here, `run_from_matlab`. Set `SIM_DURATION_OVERRIDE = 120`
in the workspace first for a short run (default 300 s; paper runs 2100 s).
If MATLAB was not started from a shell with SUMO_HOME set: `setenv('SUMO_HOME','C:\Program Files (x86)\Eclipse\Sumo')`
and `addpath(genpath('C:\Program Files\MATLAB\traci4matlab'))` before running.

At the end: `sim_trajectory_data_<APPLY_MODE>.mat`, time-space diagrams, and four console
reports — kinematic check, collisions (from `collisions.xml`), SUMO warnings (from
`sumo_log.txt`), speed-drop attribution, controller audit.

## Changes with respect to the code that produced the paper's results

### 1. Execution of CAV commands (run_from_matlab.m, preliminaries.m, quickstart.rou.xml)
- `moveTo` removed (it moved every CAV twice per step; `check_kinematics` showed ratio 2.0).
  `APPLY_MODE = 'fixed'` (default): `setSpeed` only; CAVs in the zone get `speedMode 96`
  (all SUMO speed/right-of-way/red-light/speed-limit checks off) and `laneChangeMode 512`.
  `'legacy_moveTo'` reproduces the original for comparison.
- `--step-method.ballistic` so the position update equals `v*dt + 0.5*u*dt^2` (Eq. 10).
- CAV vType: `accel = u_max (3.459)`, `minGap = s0 (2.0)`, `tau = T_safe (0.5)`,
  `speedFactor = 1, speedDev = 0`. `cav.v_max = idm.v0 = 13.89` (paper Sec. V; code had 15;
  the optimizer's private copy of `cav` updated accordingly).
- Junction-internal lanes set to 13.89 m/s (the model has no turning dynamics).
- Collisions logged (`--collision.action warn`), SUMO messages logged to `sumo_log.txt`.

### 2. Rear-end safety constraint (cav_control_policy.m, `cbf_project`)
Paper Eqs. (13),(18) implemented literally: `h = gap - s0 - v*T`, `h_dot + kappa*h >= 0`,
solved as the QP with box bounds (closed form). `cav.T_safe = 0.5`, `cav.kappa = 0.3`.
Original: `h = gap - L - s0`, second-order barrier with gains 0.01/0.05, no clipping.
`u_nom` is clipped to `[u_min, u_max]` before the projection.

### 3. Downstream policy (run_from_matlab.m, APPLY block)
After the stop line the CAV stays under the controller until it leaves the zone radius:
nominal `u = u_max` toward `v_max`, subject to the same Eq. (18) constraint against the
leader found in the per-step vehicle data (and SUMO `getLeader` across lane boundaries).
Then hand-back to SUMO (`speedMode 31`, `laneChangeMode 1621`, `setSpeed(-1)`).
Reason: the original handed the CAV to SUMO's IDM at the stop line, which produced
emergency braking (IDM disagreed with the controller's headway) or, with a blind
acceleration inside the junction, rear-end collisions.

### 4. Stop-line handling on red (cav_control_policy.m)
- Original: instantaneous stop (`v_next = 0`), executed literally once SUMO's checks are off.
  Now: brake with `u = -v^2/(2 d)` bounded by `u_min` toward a standoff point
  `cav.line_standoff = 1.0 m` before the line (prevents overshoot onto the internal lane,
  which used to launch the CAV through the red). If even `u_min` cannot stop the vehicle,
  a `cannot stop before stop line on red` message is printed.
- Commit rule (dilemma zone): the one-step stoppability constraint is enforced unless the
  vehicle reaches the line at its CURRENT speed at least `cav.green_margin = 1.0 s` before
  the end of the contiguous green (and its planned crossing does too). Crossing-time
  targets are admissible only if the green persists `green_margin` after them.

### 5. Signal horizon / clearance interval (run_from_matlab.m, both controller files)
- Clearance `h_c` (`yellowDuration`) = 3.0 s (original 0.5 s, which relied on SUMO's
  right-of-way logic to hold vehicles; a vehicle entering the 20 m junction at the end of
  green needs 1.5-3 s to clear). Shown after every non-zero green followed by a different
  phase (original: only when the green was longer than the yellow).
- The horizon handed to CAVs carries a clearance tail only where the TLS will show one;
  zero-duration (skipped) phases carry none and are skipped in the phase model (original:
  a tail on every slot and 0.1 s clamps, which made CAVs believe a continuous green was
  ending every 3 s and coast at reduced speed on green).
- Beyond the horizon, the phase model extrapolates historical green durations plus a
  clearance each (original subtracted it, so the lane was "never green again" and
  `find_next_green_start` hit its 500 s limit).
- The optimizer evaluates schedules with the clearance lost time included (Eq. 8's
  epsilon), optional 11th argument `yellowDurIn`.

### 6. Profile fallbacks (cav_control_policy.m)
When no closed-form case of `cav_control_profile` applies (57% of steps in the original):
cruise (`v*T == l`), smooth deceleration toward the standoff point when the target is far
(`v*T > l`), bounded linear profile near cruise. Effort (mean u^2) fell from 1.27 to 0.90.

## Parameters worth knowing
- `controlZoneRadius = 200` m (paper text; current network script has 150 m).
- `vehicleHeadway = 3.1`, `cavHeadway = 0.5`, `dt = 0.1` (values of `Final_play - Copy (5)`).
- Paper Sec. V states `u_min = -6, u_max = 5`; the code (unchanged) uses `-3, 3.459`.
- The static `tlLogic` in `quickstart.net.xml` is inert (overridden via TraCI).

## Known, accepted behaviours
- A CAV whose lane's phase has no history and no vehicles in the horizon cannot forecast a
  green (target = search limit); it decelerates smoothly toward the line and waits — the
  optimizer grants the phase once the vehicle is counted.
- Behind an HDV that brakes for yellow, the Eq. (18) barrier brakes at `u_min` for ~2 s
  (h stays >= 0). A larger `T_safe` would make this gentler (parameter choice).
