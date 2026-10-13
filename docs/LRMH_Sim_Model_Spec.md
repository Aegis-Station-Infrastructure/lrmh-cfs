# LRMH Simulation Model Specification

**Document:** LRMH-SM-001
**Version:** 1.0
**Date:** October 2026

*Scope: every model the sim HAL backend uses to synthesize the signals the flight software would receive from a real actuator in real regolith, and where each number comes from.*

---

The simulation lives in `sim/` (the physics models) and `hal/sim/lrmh_hal_sim.c` (the UDP server that drives them at 100 Hz). Per ADR-0002 the regolith is opaque to the flight software: the mission apps see force, depth, stroke, current, bus voltage and temperature, never anything in this document. Everything here is a modelling choice; nothing is fitted to bench hardware, and every envelope number it shares with the flight software is a design assumption for a simulated instrument, not a measurement.

The model exists for synthesis: many sites that differ the way real ones do, lunar and Earth, so that nominal operation can be exercised across them and a fault is recognisable as a departure from all of them rather than from one fixed curve.

## 1. Conventions

- **Depth and stroke.** The sim reports `depth_m` equal to `stroke_m` (no tool offset). The tool-tip area used for the regolith reaction is a fixed 1 cm^2 (`LRMH_TOOL_TIP_AREA_M2`).
- **Timebase.** `CLOCK_MONOTONIC`. The physics tick is 10 ms (`LRMH_HAL_SIM_TICK_NSEC`), driven on the wall clock inside the UDP loop, independent of request arrival. `time_us` in every read is the sim clock at the read.
- **Determinism.** A site is a pure function of (preset, seed). Noise on the sensor channels comes from the sensor model's own fixed-seed generator, which is seeded once at process start and never reseeded, so two runs from a fresh simulator with the same site and command sequence reproduce each other.
- **Gravity** is a site parameter (1.62 m/s^2 lunar presets, 9.81 m/s^2 Earth presets), because the cone index gradient at one sixth g is about a quarter of its 1 g value for the same material.

## 2. Regolith model (`sim/regolith_model.c`)

### 2.1 What a site is

A site is **drawn** from a preset and a seed (`Regolith_DrawSite`, `Regolith_SetSite`). The same preset and seed draw the same site. The default site at init is `lunar_highland` drawn from site id 0.

| Element | What it is | Lunar highland draw |
|---|---|---|
| Density profile | Carrier's law rho(z) = rho_inf (z+A)/(z+B), z in cm | 1.92 (z+12.2)/(z+18) g/cm^3, scaled 0.95 to 1.05 |
| Layers | 1 to 3; each has a top depth, cohesion c, friction angle phi, a density scale | c 500 to 1200 Pa, phi 40 to 45 deg; each deeper layer x1.4 in c and +1.5 deg |
| Loose top | A crust factor fading quadratically over a thickness | x0.2 to 0.4 over the top 4 cm |
| Variability | A log-normal field along depth: value noise with a correlation length | sigma 0.25, length 1.5 cm |
| Clasts | Buried fragments in the top 35 cm: a bump in resistance over their size; those marked immovable add their full strength over 5 mm of contact compression (2 MPa is 200 N on the 1 cm^2 tip, so a force clamp below that stops the tool and one above pushes through) | 2.5 per metre, 0.5 to 3 cm, 2 to 8 MPa, 25% of those over 2 cm immovable |
| Shaft friction | Fraction of resistance felt inside an existing hole, reached over 8 mm of rebound as the tip lifts | 0.15 |
| Compressibility | Density growth per load cycle | 0.10 |
| Bekker kc, kphi, n | Plate-load sinkage parameters | 1.4 kN/m^2, 820 kN/m^3, 1.0 |

The variability field is a function of depth alone, so a second pass down the same hole meets the same soil.

### 2.2 Presets

| Preset | g (m/s^2) | What it stands for | q at 10 cm, 1.29 cm^2 cone (12 seeds) |
|---|---|---|---|
| `lunar_highland` (0) | 1.62 | Apollo 16 Descartes: dense, cohesive, rock fragments | 260 to 1800 kPa, median 525 |
| `lunar_mare` (1) | 1.62 | Apollo 12/15 mare: lighter, fewer fragments | 100 to 850, median 220 |
| `lunar_loose` (2) | 1.62 | Fresh ejecta, a crater rim | 13 to 74, median 42 |
| `earth_dry_sand` (3) | 9.81 | Cohesionless, uniform density | about 200 |
| `earth_sandy_loam` (4) | 9.81 | Some cohesion, a crust, rocks | about 300 to 700 |
| `earth_firm_clay` (5) | 9.81 | Cohesion carries it, phi near 0 | about 300 to 1000; a buried clast can take a seed to several MPa |
| `bench_simulant` (6) | 9.81 | A bin of JSC-1A/LHS-1-like simulant, one layer, a floor at 30 cm | about 400 to 1700; the floor stops the tool |

The Earth presets exist because a bench backend will push the tool into one of them at 1 g. The sim already produces those curves so the flight software's behaviour on them can be rehearsed before any bench exists.

### 2.3 How resistance is computed

Cone resistance by bearing-capacity theory, the method the Apollo 16 Preliminary Science Report used in reverse to get c and phi from penetration records:

```
q = F * ( c Nc sc dc  +  sigma_v Nq sq dq  +  0.5 gamma B Ngamma sgamma )
```

- `Nq = e^(pi tan phi) tan^2(45 deg + phi/2)`, `Nc = (Nq - 1)/tan phi`, `Ngamma = 2 (Nq + 1) tan phi` (Vesic); for phi = 0, `Nc = 5.14`.
- Shape factors for a circular tip: `sc = 1 + Nq/Nc`, `sq = 1 + tan phi`, `sgamma = 0.6`. Depth factors after Hansen with `k = D/B` for `D <= B` and `arctan(D/B)` beyond.
- `sigma_v` is the overburden from the density profile under the site's gravity, integrated layer by layer (the integral of (z+A)/(z+B) is z + (A-B) ln(z+B)).
- The loose surface: phi is reduced by `SurfaceSoftening` (8 deg lunar) scaled by how far the density ratio is from its deep value, and c by the ratio squared. This, with the density gradient, is the Moon's steep near-surface rise.
- `F` is one calibration factor per preset: **4.5 for the lunar presets, fitted to the Apollo 16 records** (below), 1.5 for dry sand, 1.0 otherwise. The gap between shallow-footing theory and a cone's deep failure mechanism is real; the factor stands for it.

Then, in order: the tip's engagement (the bearing area grows as the square of depth over the cone's 10 mm height, so the surface bears nothing at zero penetration), the crust, the variability field, the clasts, the hole (shaft friction only above the deepest point reached so far, with the 8 mm rebound ramp), the compaction multiplier below the hole, and a 5% per 5 mm/s rate effect. Force is q times the tip area. The local stiffness proxy is dq/dz over one millimetre.

### 2.4 Disturbance

The hole: `Regolith_GetPenetrationResponse` records the deepest point reached. On a later pass (POST_PEN) the tip feels only shaft friction above that depth and the compacted soil below it.

Compaction: `Regolith_RecordLoadCycle(peak_force, peak_sinkage)` increments the density multiplier by `Compressibility * peak_force / 500 N`, capped at 2.0. The HAL sim does not currently call this from its physics tick; load phases run through the penetration resistance path.

Plate load: `Regolith_GetLoadResponse` implements Bekker sinkage `p = (kc/b + kphi) z^n` and is kept for the API; the HAL sim's physics tick does not use it.

### 2.5 Against Apollo 16

The Apollo 16 self-recording penetrometer records (ten penetrations, stations 4 and 10, PDS bundle `urn-nasa-pds-a16lsrp_soil_mechanics`) are digitized as load and depth per row. The resistance envelope of each record is the largest load seen at each depth, made monotone, over the cone's area. Nine records used the 0.2 in^2 (1.29 cm^2) cone. `sim/tools/apollo16_compare.py` fetches the records from the PDS Geosciences Node (once, into a cache) and runs `regolith_probe` against them.

| Depth | Apollo 16, 1.29 cm^2 cone (9 records) | `lunar_highland`, 12 seeds |
|---|---|---|
| 10 cm | 142 / 577 / 997 kPa (min / median / max) | 264 / 525 / 1812 |
| 15 cm | 160 / 823 / 1135 | 324 / 866 / 6695 |
| 20 cm | 160 / 966 / 1204 | 580 / 1216 / 7339 |
| 30 cm | 427 / 1281 / 1660 | 831 / 1728 / 8175 |

Two things to read in that table. The Apollo values flatten toward 1650 kPa because the instrument's force limit was 215 N: 215 N over 1.29 cm^2 is 1667 kPa. Beyond 15 cm the Apollo envelope is the penetrometer's limit, not the soil's, so the model is fitted at 10 and 15 cm and allowed above at depth. The model's maxima above 5 MPa are seeds that hit an immovable clast; the Apollo crews stopped at the force limit in two records at 21 cm, which may have been the same thing.

The spread between sites, a factor of seven at 10 cm, is the point. The simulated tool (1 cm^2, 650 N) reads to 6.5 MPa and will see past where Apollo stopped.

### 2.6 Probing a site from the command line

```
./build/sim/regolith_probe --list
./build/sim/regolith_probe lunar_highland 3            # 1.29 cm^2 cone, 0 to 35 cm, 0.5 cm steps
./build/sim/regolith_probe bench_simulant 7 1.0 35 0.5  # 1 cm^2 tip
python3 sim/tools/apollo16_compare.py                   # lunar presets vs the PDS records
```

`regolith_probe <preset> <seed> [area_cm2] [max_depth_cm] [step_cm] [gravity]` prints one header line describing the drawn site (layers, crust, clasts) and then `depth_cm q_kPa force_N layer clast hole` per step.

## 3. Actuator model (`sim/actuator_model.c`)

| Parameter | Default | Note |
|---|---|---|
| Max stroke | 0.35 m | Hard stop; matches `LRMH_MAX_STROKE_M` |
| Max velocity | 0.01 m/s | 10 mm/s |
| Max force | 650 N | Matches `LRMH_MAX_FORCE_N` |
| Max current | 5.0 A | Current is clamped here |
| Backlash | 0.2 mm | |
| Response lag | 50 ms | First-order lag on velocity |
| Stall current threshold | 4.0 A | Stall declared when current exceeds this at zero velocity |
| Hard-stop stiffness | 50 kN/m | Spring at the ends of travel |

Control modes: POSITION (P-gain 2.0 on position error, 0.5 mm deadband, rate-limited), FORCE (drive toward the target force, slowing inside 10 N), RETRACT (to home, 1 mm deadband), HOLD, ESTOP.

Motor current: `I = |F| / 140 N/A + |v| * 0.02 A per m/s + 0.1 A` quiescent, clamped at 5 A. The 140 N/A constant puts 650 N continuous at about 5 A.

Temperature: `dT = (I^2 * 0.5 W/A^2 * dt - (T - 20 C) / 0.5 K/W * dt) / 200 J/K`, starting at 20 C ambient.

Command timeout: an active command that exceeds its timeout is marked complete without changing mode; the FSW decides what to do. The HAL sim passes a 60 s move timeout (plus the ramp time for a force command) and a 30 s retract timeout into the model.

### 3.1 Clamps and flags (ADR-0010)

The HAL sim enforces the clamps in `physics_tick`:

- A position move holds (does not retract) when `AppliedForce >= max_force_n` and sets `STOPPED_AT_FORCE_CLAMP`. A command with `max_force_n = 0` gets the 650 N envelope, never no clamp at all.
- A force command holds when `Position >= max_stroke_m` and sets `STOPPED_AT_STROKE_CLAMP`.
- A retract holds when `AppliedForce >= 1.5 x 650 N` and sets `RETRACT_BLOCKED`.
- On natural completion the sim sets `POSITION_REACHED` or `FORCE_REACHED`.

A force command's target is ramped at `ramp_rate_nps` inside the physics tick and the actuator tracks the moving target at its top speed; the command is not complete while the target is still ramping.

### 3.2 State mapping

`IsEStoped` maps to ESTOP; `IsStalled` to FAULT (with `RETRACT_BLOCKED` set); `CommandComplete` in HOLD mode to HOLDING, in any other mode to IDLE; otherwise MOVING. `AtHardStopMin/Max` set `HARD_STOP_HIT`.

## 4. Sensor model (`sim/sensor_model.c`)

| Channel | Noise (1 sigma) | Bias / drift | Other |
|---|---|---|---|
| Force | 1.5 N Gaussian | 0.3 N bias, 0.001 N/s drift | Saturates at 650 N; 0.2% dropout clears the valid bit |
| Depth | 100 um Gaussian | none | 50 um encoder quantisation; 0.1% dropout |
| Stroke | same as depth | | Valid when depth is valid |
| Current | 0.05 A | 0.02 A bias | |
| Bus voltage | none | | Constant 28.0 V; always valid |
| Temperature | 0.2 C | none | |
| Timestamp | 1 ms jitter | | |

A dropout leaves the last value in place and clears that channel's bit in `channel_valid`. `lrmh_sense` tolerates single-cycle blips; three consecutive stale cycles on any channel raise `SENSOR_TIMEOUT`.

## 5. Fault hooks (sim only)

Three UDP request types exist only in the sim backend. The bench template acknowledges `INJECT_FAULT` as a no-op; a bench or flight backend has none of these.

| Request | Payload | Effect |
|---|---|---|
| `INJECT_FAULT` (9) | `kind, override_value, duration_ms` | Overrides one sensor channel on every following read so the matching LC watchpoint trips: 1 force 700 N, 2 current 5.5 A, 3 stroke 0.34 m, 4 temperature 60 C, 5 bus 21 V, 6 force ramping from 50 to 600 N at 1500 N/s and wrapping (a sustained dF/dt above the watchpoint). `override_value` replaces the default; `duration_ms` 0 persists until kind 0. |
| `SET_SITE` (10) | `uint32 preset, uint32 seed` | Draws and loads a site; the hole and compaction reset. The site survives a later `INIT` so a run does not revert to the default draw. |
| `ACTUATOR_FAULT` (11) | `uint32 kind, uint32 active` | A fault in the actuator physics: 0 stall (velocity zero, current pinned at 4.5 A, command never completes), 1 overcurrent (6 A), 2 encoder loss (depth freezes and its channel goes invalid; the stroke encoder is unaffected). |
| `SIM_FAULT` (12) | `kind, value, duration_ms` | A fault in the sensor or the ground: 0 force sensor frozen at its last reading (still reported valid), 1 a sudden hard layer below the current depth resisting `value` times more, 2 a bad zero (force offset by `value` N), 3 tool not present (no reaction at all). Cleared with kind 0xFFFF. |

`INJECT_FAULT` overwrites one channel with a constant; `ACTUATOR_FAULT` and `SIM_FAULT` change what the physics does, so every channel moves the way it would. The stall is the one that matters for fault work. The bridge exposes these as `inject_fault`, `set_site` and `actuator_fault`; `scripts/hal_bench_test.py --inject <kind>` sends `INJECT_FAULT` directly.

## 6. Where the numbers come from

- Carrier, Olhoeft and Mendell, *Lunar Sourcebook* ch. 9 (1991): the density-depth law and the ranges of c (0.1 to 1 kPa) and phi (30 to 50 deg).
- Apollo 16 Preliminary Science Report, Soil Mechanics Experiment: the bearing-capacity method and the self-recording penetrometer records (PDS).
- Gasteiner, Murdoch and D'Angelo, *An Open Database of Lunar Regolith and Simulants Properties* (2026): the per-site and per-simulant parameter pool the presets are set from.
- Wong, *Theory of Ground Vehicles*: the Bekker parameter tables for dry sand, sandy loam, clayey soil and the lunar estimate. To be checked against the edition in hand.
- JSC-1A strength properties (ASCE JGGE, 2010) and LHS-1 direct shear: c about 1 kPa, phi 40 to 46 deg depending on density, for the bench preset.
- The cone index gradient at 1/6 g is about a quarter of its 1 g value for the same material (GRC-1 work): the reason gravity is a site parameter and not a constant.

## 7. Known fidelity gaps

1. The `ConeFactor` of 4.5 is one number standing for the cone's failure mechanism; a Durgunoglu-Mitchell cone solution would replace it.
2. The plate-load path (`Regolith_GetLoadResponse`) is not used by the HAL sim's physics tick; load phases run through the penetration resistance, and `Regolith_RecordLoadCycle` is never called from the tick, so the compaction multiplier stays at 1.0 in a normal run.
3. No sidewall friction on the shaft during the first penetration, and no pore or thermal effects.
4. Depth equals stroke; there is no tool-offset model and no depth-versus-stroke disagreement path for the `DEPTH_DISAGREE` fault to detect.
5. Bus voltage is a constant 28.0 V; the `BUS_VOLTAGE` fault is reachable only through `INJECT_FAULT`.
6. The actuator thermal model (I^2 times 0.5 ohm into 0.5 K/W) rises at most about 6 C above ambient at 5 A, so the `THERMAL` fault is also reachable only through `INJECT_FAULT`.
7. `HAL_REQ_INIT` returns the simulated tool to home. The HAL contract says Init does not move the tool, and a hardware backend must honour that; in the sim, a tip left parked at the bottom of its hole makes the loading/unloading branch of the regolith model flip every sample.
8. Sensor noise is Gaussian white with no spectral shaping; the force-rate watchpoint margin (800 N/s against 1.5 N noise at 100 Hz, about 150 N/s per sample pair) was set against this model.
9. Apollo 15 records (`urn-nasa-pds-a15lsrp_soil_mechanics`) are not yet in the comparison.
