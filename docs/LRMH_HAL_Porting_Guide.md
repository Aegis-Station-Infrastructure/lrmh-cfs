# LRMH HAL Porting Guide

**Document:** LRMH-HP-001
**Version:** 1.0
**Date:** October 2026

*Scope: what a bench or flight HAL backend has to provide so the eight mission apps run unchanged against real hardware.*

---

## Overview

The LRMH Hardware Abstraction Layer is the single portability boundary between the flight software and the actuator and sensor stack. Every actuator command and every sensor read goes through the nine functions declared in `hal/include/lrmh_hal.h`. The mission apps never touch hardware directly, and the header is identical in sim, bench and flight builds (ADR-0001).

The HAL is deliberately placed at the setpoint level (ADR-0003): the flight software issues position and force setpoints with safety clamps, and the position and force loops close below the HAL line in the motor controller. The apps never see duty cycles, encoder counts or ADC codes.

Three backends are defined:

| Backend | Directory | Status |
|---------|-----------|--------|
| `sim` | `hal/sim/lrmh_hal_sim.c` | Complete. Standalone physics process, UDP port 17011, 100 Hz tick. |
| `bench` | `hal/bench/lrmh_hal_bench.c` | Template. UDP envelope and dispatch complete; every handler body is a stub marked `TODO(bench)`. |
| `flight` | `hal/flight/` | Not implemented. `cmake -DTARGET=flight` warns and builds nothing. |

The cFS image links `hal/client/lrmh_hal_client.c`, which implements the nine `LRMH_HAL_*` functions as UDP requests to whichever backend process is listening. Nothing in the image changes when you swap sim for bench; only the process on the other end of the socket changes.

## Architecture

```
   LRMH apps (lrmh_cmd, lrmh_ctrl, lrmh_sense, ...)
                        |
             hal/include/lrmh_hal.h       <-- THE API CONTRACT
                        |
             hal/client/lrmh_hal_client.c  (UDP client, in the cFS image)
                        |
                  UDP 127.0.0.1:17011
                        |
        +---------------+---------------+
        |               |               |
   hal/sim/        hal/bench/       hal/flight/
   physics         your controller  (not implemented)
```

Only `lrmh_cmd` (Init, Retract on STOW, EStop on ABORT), `lrmh_ctrl` (all commands, GetHealth) and `lrmh_sense` (ReadSensors at 100 Hz) call the HAL. The other five apps work entirely from Software Bus telemetry.

## Status codes

All functions return `int32`:

| Code | Value | Meaning |
|------|-------|---------|
| `LRMH_HAL_OK` | 0 | Accepted (for commands: accepted, not completed) |
| `LRMH_HAL_ERROR` | -1 | Generic failure |
| `LRMH_HAL_NOT_INITIALIZED` | -2 | `Init` has not succeeded |
| `LRMH_HAL_REJECTED_ESTOP` | -3 | Actuator is latched in ESTOP |
| `LRMH_HAL_REJECTED_FAULT` | -4 | Actuator is in FAULT |
| `LRMH_HAL_REJECTED_BAD_PARAM` | -5 | Out-of-range argument |
| `LRMH_HAL_REJECTED_BUSY` | -6 | A command is already in progress and cannot be preempted |
| `LRMH_HAL_SELFTEST_FAIL` | -7 | `Init` self-test failed |
| `LRMH_HAL_BACKEND_TIMEOUT` | -8 | Client got no response within 100 ms |

## The nine request handlers

The UDP envelope and the C API map one to one. Request types are the `HalReqType_t` enum shared by `hal/client/`, `hal/sim/` and `hal/bench/`.

| Req | C function | What the backend must do |
|-----|------------|--------------------------|
| 0 `INIT` | `LRMH_HAL_Init()` | Power on, zero offsets, run the self-test, clear any ESTOP or FAULT latch. On success the actuator is IDLE at its current stroke; it is not moved to home. Return `LRMH_HAL_SELFTEST_FAIL` if any channel fails. Called at startup and on every recovery from ESTOP or FAULT. |
| 1 `READ_SENSORS` | `LRMH_HAL_ReadSensors(out)` | Fill `LRMH_HAL_SensorData_t`: `force_n`, `depth_m`, `stroke_m`, `current_a`, `bus_v`, `temp_c`, plus the freshness fields below. Called at 100 Hz. |
| 2 `GET_HEALTH` | `LRMH_HAL_GetHealth(out)` | Fill `LRMH_HAL_Health_t`: `state` (`LRMH_HAL_State_t`), `flags` (`LRMH_HAL_FLAG_*`), `stroke_m`, `last_force_n`. Polled at 10 Hz by `lrmh_ctrl` to detect command completion. |
| 3 `CMD_POSITION` | `LRMH_HAL_CommandPosition(cmd)` | Drive toward `target_stroke_m` at no more than `max_speed_mps`. If force exceeds `max_force_n`, stop where you are (do not retract) and set `STOPPED_AT_FORCE_CLAMP`. Return on accept; completion is observed through `GetHealth`. |
| 4 `CMD_FORCE` | `LRMH_HAL_CommandForce(cmd)` | Apply `target_force_n` at the tool tip, ramping at `ramp_rate_nps` (0 means step). If stroke exceeds `max_stroke_m`, hold the last force and set `STOPPED_AT_STROKE_CLAMP`. On completion the state becomes HOLDING. |
| 5 `RETRACT` | `LRMH_HAL_Retract()` | Retract to stroke 0 at maximum safe speed with an implicit force clamp at 1.5 x `LRMH_MAX_FORCE_N`. On clamp, stop and set `RETRACT_BLOCKED`. Permitted from every state except ESTOP. Clears a FAULT latch. |
| 6 `HOLD` | `LRMH_HAL_Hold()` | Stop the active move and hold the current stroke and force. Completes immediately; state becomes HOLDING. |
| 7 `ESTOP` | `LRMH_HAL_EStop()` | Drop motor enable now and latch. Only `Init` clears the latch. Must be immediate. |
| 8 `STEP_SIM` | `LRMH_HAL_StepSim(dt)` | Sim only. A bench or flight backend acknowledges with `LRMH_HAL_OK` and does nothing. |

Request types 9 through 12 (`INJECT_FAULT`, `SET_SITE`, `ACTUATOR_FAULT`, `SIM_FAULT`) are sim-only test hooks. The bench template acknowledges `INJECT_FAULT` as a no-op; a bench backend that wants fault stimulus needs a controller-specific hook of its own.

## Actuator state machine (ADR-0010)

```
          Init                 CmdPosition / CmdForce
  COLD ---------> IDLE ----------------------------------> MOVING
                   ^  ^                                       |
                   |  |  position reached / Retract done      |
                   |  +---------------------------------------+
                   |                                          |
                   |  Retract         force reached / Hold    v
                   +-------------- HOLDING <------------------+

  any state except ESTOP --EStop--> ESTOP --Init--> IDLE
  MOVING --stall / hard stop--> FAULT --Init or Retract--> IDLE
```

`LRMH_HAL_State_t` values: `COLD` 0, `IDLE` 1, `MOVING` 2, `HOLDING` 3, `FAULT` 4, `ESTOP` 5.

`lrmh_ctrl` treats IDLE and HOLDING as "command complete"; a backend that reports MOVING forever holds the sequencer until the command's deadline (`LRMH_ACTUATOR_TIMEOUT_S` 30 s for retracts, contact and the self-test; 120 s for a penetration; ramp time plus 30 s for a load ramp), after which the site is skipped as T2.

## Health flag bits

| Bit | Name | Set when |
|-----|------|----------|
| 0x0001 | `POSITION_REACHED` | A position command reached its target |
| 0x0002 | `STOPPED_AT_FORCE_CLAMP` | A position command stopped at `max_force_n` |
| 0x0004 | `FORCE_REACHED` | A force command reached its target |
| 0x0008 | `STOPPED_AT_STROKE_CLAMP` | A force command stopped at `max_stroke_m` |
| 0x0010 | `RETRACT_BLOCKED` | A retract stopped at 1.5 x max force |
| 0x0020 | `HARD_STOP_HIT` | Mechanical end of travel reached |
| 0x0040 | `ENCODER_LOSS` | Depth encoder channel lost |
| 0x0080 | `OVERCURRENT` | Motor current above the hardware limit |
| 0x0100 | `THERMAL` | Winding or controller over temperature |
| 0x0200 | `BUS_VOLTAGE` | Bus outside the operating window |
| 0x0400 | `BACKEND_LOST` | Set by the client, not the backend, when requests time out |

The lower four bits are status and clear on the next command; the upper bits are faults. The ground UI renders them as a labelled bit grid in the Engineer view.

## Freshness contract (ADR-0009)

Every `READ_SENSORS` and `GET_HEALTH` response carries three freshness fields, and a backend that fakes them will trip the fault path:

- `valid` (bool): false means discard the whole read. Set it false when the controller or sensor bus is down, never to mean "no new sample".
- `time_us` (uint64): the monotonic time at **sample acquisition**, not at UDP receipt. `lrmh_sense` rebases this into cFE time at the read boundary, so the absolute epoch does not matter, but the value must advance with real samples.
- `channel_valid` (uint16): one bit per channel (`LRMH_HAL_CH_FORCE` 0, `DEPTH` 1, `STROKE` 2, `CURRENT` 3, `BUS_V` 4, `TEMP` 5), set when that channel's value in this read is fresh. A channel whose transducer has dropped out must have its bit cleared, with the last value left in place.

On the FSW side `lrmh_sense` counts consecutive stale cycles per channel; three in a row are counted and evented as `SENSOR_TIMEOUT` by `lrmh_sense`; that path dispatches no recovery on its own. `lrmh_fault` separately raises `SENSOR_TIMEOUT` if no sample telemetry arrives for `LRMH_SENSOR_TIMEOUT_MS` (200 ms) during an acquiring phase. `lrmh_proc` rejects any sample whose timestamp predates the current phase start.

## Wire envelope

`HalRequest_t` and `HalResponse_t` are fixed-size structs with the magic `0x4C524D48` ("LRMH"):

```
HalRequest_t   { uint32 magic; uint32 seq_no; uint32 req_type;
                 uint32 payload_len; uint8 payload[256]; }
HalResponse_t  { uint32 magic; uint32 seq_no; uint32 req_type; int32 status;
                 uint32 payload_len; uint8 payload[256]; }
```

Payloads are the C structs from `lrmh_hal.h` copied byte for byte (`LRMH_HAL_SensorData_t`, `LRMH_HAL_Health_t`, `LRMH_HAL_PositionCmd_t`, `LRMH_HAL_ForceCmd_t`, a `double` for `STEP_SIM`). Host byte order; both ends are assumed to be the same little-endian machine or the equivalent.

The client (`hal/client/lrmh_hal_client.c`) does four things a backend author should know about:

1. Bounded receive: each request waits at most 100 ms (`LRMH_HAL_REQ_TIMEOUT_MS`) and returns `LRMH_HAL_BACKEND_TIMEOUT` otherwise. Do not block in a handler; start the controller operation and return.
2. Sequence numbers: a response whose `seq_no` does not match the request is drained and ignored, so a late reply to an earlier request never masquerades as the current one. Echo `seq_no` exactly.
3. Per-task sockets: each cFS task has its own socket and sequence counter, so `lrmh_sense` at 100 Hz and `lrmh_ctrl` at 10 Hz do not cross-talk. The backend will see requests from several source ports.
4. The sim backend runs physics on its own clock, not on request arrival, so a slow client does not slow the sim.

The same envelope is reproduced at the top of `scripts/hal_bench_test.py`. If you change it in any of the four places, change all four.

## Porting walkthrough: bench

The bench template is `hal/bench/lrmh_hal_bench.c`. It is structurally identical to the sim backend (same `main`, same UDP loop, same dispatch switch) with every physics call replaced by a stub that returns zeroed-but-valid data for sensors and `LRMH_HAL_OK` for commands. Each stub is marked `TODO(bench):`.

Replace the stubs in this order:

1. `hal_init`: power on, zero offsets, self-test sweep. Return `LRMH_HAL_ERROR` or `LRMH_HAL_SELFTEST_FAIL` when a channel fails.
2. `hal_read_sensors`: populate the six channels, `channel_valid` and `time_us` at acquisition.
3. `hal_get_health`: controller state, flag word, current stroke, last commanded force.
4. `hal_cmd_position`: drive to `target_stroke_m` at `max_speed_mps` with the `max_force_n` clamp.
5. `hal_cmd_force`: ramp to `target_force_n` at `ramp_rate_nps` with the `max_stroke_m` clamp.
6. `hal_retract`: unconditional retract to home; clears the FAULT latch.
7. `hal_hold`: stop and hold.
8. `hal_estop`: drop motor enable, latch until `Init`.
9. `hal_step_sim`: leave as a no-op.

Extend the `g_bench` struct with whatever long-lived handles the controller library needs (bus descriptors, calibration vectors). Then:

```bash
cmake -S . -B build-bench -DTARGET=bench
cmake --build build-bench
./build-bench/hal/bench/lrmh_hal_bench
# In another shell, launch the cFS image as usual; it will talk UDP to this
# process instead of the sim.
```

The cFS image does not need to be rebuilt to switch backends. Stop `lrmh_hal_sim`, start `lrmh_hal_bench` on the same port, and the next request from the image goes to the bench.

### The bench integration loop

`scripts/hal_bench_test.py` talks UDP directly to a HAL backend (no cFS image, no bridge) and walks each primitive through a canonical scenario:

```
1. Replace one TODO(bench) block in lrmh_hal_bench.c
2. cmake --build build-bench
3. ./build-bench/hal/bench/lrmh_hal_bench &
4. python3 scripts/hal_bench_test.py --scenario <relevant>
5. Watch the matching scenario flip from FAIL to OK; iterate
```

The eight scenarios:

| Scenario | What it checks |
|----------|----------------|
| `init` | `Init` returns OK |
| `sensors` | 100 reads at 100 Hz; `valid` and `channel_valid` behave per the freshness contract |
| `position` | Move 0 to 0.05 m at 0.01 m/s; final stroke within 5 mm of target |
| `force` | A force command is accepted and the state machine responds (convergence is regolith-dependent on the sim) |
| `retract` | From a non-zero stroke, `Retract` ends at IDLE near 0 |
| `hold` | `Hold` mid-motion freezes the actuator at HOLDING |
| `estop` | `EStop` latches ESTOP; subsequent motion is rejected; `Init` + `Retract` clears |
| `state-machine` | Lifecycle walk: Init, CmdPos, Retract, CmdForce, Hold, Retract, EStop, Init, Retract |

Each scenario is independent and begins with Init + Retract. Run one with `--scenario <name>` or all eight (the default). Output is `RUN <name>` / `OK <name>` / `FAIL <name>: <reason>`. Options: `--host`, `--port` (default 127.0.0.1:17011), `--timeout-s` (default 0.2), `--verbose`. The `--inject <kind>` option sends a sim-only `INJECT_FAULT` request and is what `scripts/lrmh_fault_test.py` uses; a bench backend acknowledges it and does nothing.

## Porting walkthrough: flight

There is no flight backend in this repository. `hal/flight/` is reserved and `TARGET=flight` only prints a warning. A flight backend has two architectural choices open:

- Keep the UDP process model (a separate HAL server process on the flight computer, the same `hal/client/`), which keeps the mission apps byte-identical to the sim and bench builds.
- Replace `hal/client/lrmh_hal_client.c` with a direct in-process implementation of the nine functions against the motor controller and sensor interfaces. The apps still see only `lrmh_hal.h`.

Either way the contract above (state machine, clamps, flags, freshness) is what the apps, the LC tables and the recovery tiers were written against.

## What the FSW assumes about the hardware

These are the envelope values in `cfs_defs/lrmh_platform_cfg.h` (ADR-0012). Every one is a design assumption for a simulated instrument, not a measurement; a real actuator will have different numbers and ADR-0012 states explicitly that none of them is canonical.

| Quantity | Envelope | LC threshold |
|----------|----------|--------------|
| Force | 650 N | 600 N |
| Force rate | 1000 N/s | 800 N/s |
| Motor current | 5.0 A | 4.5 A |
| Stroke | 0.35 m | 0.33 m |
| Temperature | 60 C | 55 C |
| Bus voltage | 22 to 34 V | 23 V low, 33 V high |

The HAL backend is expected to enforce the hardware limits itself (the inner loops below the line); the LC thresholds sit inside them so the FSW reacts before the hardware limit is reached.

## Validation checklist

For a new backend, verify:

- [ ] `Init` returns `LRMH_HAL_OK` and leaves the state IDLE
- [ ] All six sensor channels report plausible values with their `channel_valid` bits set
- [ ] `time_us` advances by about 10 ms between consecutive 100 Hz reads
- [ ] Non-blocking commands return within a few milliseconds (well inside the 100 ms client timeout)
- [ ] `GetHealth` state goes MOVING then IDLE (position) or HOLDING (force)
- [ ] The force clamp on a position move stops the actuator and sets `STOPPED_AT_FORCE_CLAMP` without retracting
- [ ] `EStop` latches within one control cycle and rejects further motion until `Init`
- [ ] `Retract` works from FAULT and clears it
- [ ] `scripts/hal_bench_test.py` reports OK for all eight scenarios
- [ ] The full stack (`scripts/lrmh_launch.sh` with the bench process in place of the sim) runs a site test to COMPLETE

## Sim backend reference

| Element | Simulation model |
|---------|------------------|
| Actuator | Rate-limited first-order response (50 ms lag), 10 mm/s top speed, 0.35 m stroke, hard-stop spring at the ends, 0.2 mm backlash. Current = force term + velocity term + 0.1 A quiescent; stall when current exceeds 4 A at zero velocity. Temperature from an I^2 R thermal model with 20 C ambient. |
| Regolith | Drawn site (density profile, layers, crust, variability field, clasts) with bearing-capacity cone resistance. See the Sim Model Spec. |
| Sensors | Gaussian noise, bias and slow drift on force; 50 um encoder quantisation on depth; random dropouts clear the channel's valid bit (0.2% force, 0.1% depth); 1 ms timestamp jitter. Bus voltage is a constant 28.0 V. Depth equals stroke. |
| Clamps | Position move holds at `max_force_n`; force command holds at `max_stroke_m`; retract holds at 1.5 x 650 N. |
| Fault hooks | `INJECT_FAULT` overrides one channel past an LC threshold; `ACTUATOR_FAULT` stalls, over-currents or blinds the encoder in the physics; `SIM_FAULT` freezes the force sensor, inserts a hard layer, offsets the zero or removes the tool. |

Full detail, including every constant, is in the [Sim Model Spec](LRMH_Sim_Model_Spec.md).
