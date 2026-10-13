# LRMH Operator Guide

**Document:** LRMH-OG-001
**Version:** 1.0
**Date:** October 2026
**Audience:** the person running site tests from the ground tools, watching a run, reading the result and recovering from a fault.

*Scope: the browser UI, the bridge, the terminal tools and the scripted drivers; what each command does and what the telemetry means.*

---

This guide is procedures and interpretation. For the other material see:

| Question | Document |
|----------|----------|
| How do I build and launch the stack? | [Build and Install Guide](LRMH_Build_Install_Guide.md) |
| How does the flight software work inside? | [Architecture Guide](LRMH_Architecture_Guide.md) |
| What physics does the sim implement? | [Sim Model Spec](LRMH_Sim_Model_Spec.md) |
| Why was it built this way? | [Design Decisions](LRMH_Design_Decisions.md) |

## 1. The instrument in one page

LRMH runs an instrumented penetration and load test at one site at a time. The host vehicle parks over the site and issues `START_SITE_TEST`; the instrument walks nine phases and publishes a site result. You own three things: getting the instrument to READY, choosing the test parameters, and deciding what to do when a fault takes the instrument to SAFE.

```
OFF --INIT--> STANDBY --DEPLOY--> READY --START_SITE_TEST--> TESTING
                 ^                                              |
                 +------------- test complete ------------------+
                 ^
                 +--INIT-- SAFE  <-- T3 fault or ABORT   (FAULT <-- repeated SAFE; RESET_FAULT -> SAFE)
```

Within TESTING the sequencer runs:

```
PRECHECK -> POSITIONING -> BASELINE_PEN -> LOAD_RAMP -> [REPEAT_LOAD x N] -> [POST_PEN]
         -> (TRACTION, stub) -> PROCESSING -> REPORTING -> IDLE
```

Two outcomes are not faults: a hard site that saturates force early, and a soft site that reaches the depth target on little force. Both produce a VALID result. Only a hardware condition (overcurrent, thermal, bus voltage) or an escalated cluster of site problems takes the instrument to SAFE.

## 2. Ground tools

All in `ground/` and `scripts/`:

| Tool | Purpose |
|------|---------|
| `ground/lrmh_ui.html` | The browser UI: Operator view (default) and Engineer view. Plain HTML, CSS and JS; no build step; no external resources. |
| `ground/lrmh_bridge.py` | WebSocket and HTTP bridge to cFS (CI_LAB UDP 1234 in, TO_LAB UDP 2234 out). Serves the UI with `--serve-ui`. Requires the `websockets` Python package. |
| `ground/lrmh_console.py` | Interactive terminal CLI with tab completion. Talks UDP straight to CI_LAB; no bridge needed. |
| `ground/lrmh_ground_test.py` | Non-interactive scripted driver (`--test full\|abort\|noop`). Same direct-UDP path. |
| `scripts/lrmh_smoke_test.py` | NOOP, INIT, DEPLOY, START_SITE_TEST against CI_LAB and a tail of the cFS log. |
| `scripts/lrmh_fault_test.py` | Injects each of the six LC-detected faults during a test and checks the dispatch chain (`--kind force-sat\|overcurrent\|hard-stop\|thermal\|bus-voltage\|force-rate\|all`). |
| `scripts/lrmh_inspect_record.py` | Listens on UDP 2234 for the next binary site record and prints it. |
| `scripts/lrmh_read_record.py` | Decodes a DS-captured site record file after the fact (`--latest`, `--json`). |
| `scripts/hal_bench_test.py` | Drives a HAL backend directly (no cFS); also the `--inject` fault stimulus. |

## 3. Security posture

The bridge is a development and bench tool, not a flight ground station.

- The WebSocket and HTTP listener binds to `127.0.0.1` by default. `--ws-host 0.0.0.0` opts into exposure on the local network.
- **There is no authentication on the WebSocket.** Anyone who can reach the port can command the cFS image, inject sim faults and read files under the cFS `cf/lrmh/` tree through the file browser route.
- The bridge talks UDP to CI_LAB (default `127.0.0.1:1234`) and receives TO_LAB telemetry on UDP 2234, bound on all interfaces regardless of `--ws-host`. UDP is unauthenticated; treat the cFS host and the network as trusted.
- The `raw_cmd` route forwards any MID and command code to CI_LAB without validation.

The intended deployment is one developer machine running the HAL sim, the cFS image, the bridge and a browser. Do not expose the bridge on a network you do not control. A real ground station would need an authentication layer above the WebSocket and a firewall around the UDP ports.

## 4. Bring-up

```bash
scripts/lrmh_launch.sh            # HAL sim, core-cpu1, bridge with UI on http://127.0.0.1:8080
scripts/lrmh_launch.sh stop       # tear down
```

The launcher finds the cFS tree through `LRMH_CFS_ROOT` (default `~/lrmh-cfs-build`), binds the bridge to `LRMH_WS_HOST` (default `127.0.0.1`) on `LRMH_UI_PORT` (default 8080), pre-creates the DS output directories, enables TO_LAB output to the local host, and enables SC RTS 1 through 9 (upstream SC boots with every RTS disabled; without this step every LC actionpoint trip is rejected with "RTS Disabled" and no recovery is dispatched). It also checks `fs.mqueue.msg_max` and warns if it is below 1024; see the Build and Install Guide for why that matters.

Open the printed URL. The `Connected` dot in the header turns green within a second. Logs land in `build/logs/` (override with `LRMH_LOG_DIR`), `/tmp/lrmh_cfs.log` and `/tmp/lrmh_bridge.log`.

To run the bridge by hand against an already running image:

```bash
cd ground
python3 lrmh_bridge.py --serve-ui --ws-port 8080
```

The WebSocket endpoint is `/ws`; everything else is served as a static file from the directory containing `lrmh_bridge.py` (override with `--ui-dir`). Without `--serve-ui` the bridge listens on port 8765 and exposes only the WebSocket. Other options: `--cfs-host`, `--ci-port`, `--tlm-port`, `--hal-host`, `--hal-port` (sim fault hooks, default 17011), `--cf-dir` (root of the cFS `cf/lrmh/` tree for the file browser), `-v`.

Run one telemetry consumer at a time: the bridge and `lrmh_inspect_record.py` both bind UDP 2234.

## 5. Operator flow (browser)

1. **Pick.** Choose Cone penetration, Plate load or Full characterization. Clicking sends `{cmd:"ensure_ready"}`: if the latest HK shows READY nothing happens; if it shows OFF, STANDBY or SAFE the bridge sends `INIT_INSTRUMENT` then `DEPLOY` (from FAULT it sends `RESET_FAULT` first, since INIT is rejected there) and waits up to 5 s for `mode == READY`; if it shows TESTING the request is refused.
2. **Params.** Set the site id, maximum depth (default 0.30 m) and maximum load (default 500 N), then *Start test*. The bridge sends `START_SITE_TEST` with `DispatchFlags = WITNESS_TARGET` (0x08) by default, which bypasses the host-vehicle pose check so the test runs with no rover telemetry present. A real deployment overrides this to 0 (require a fresh host pose) or `STRICT_NAV` (require the host stopped).
3. **Run.** The phase bar shows the current sequencer phase with friendly labels. Force, depth and motor current cards update from the 100 Hz `sense` stream. The plot draws the baseline force-vs-depth trace; on entering POST_PEN it switches to the dashed post-disturbance trace.
4. **Done.** A quality pill (Valid, Partial, Fault) and the result stats from the `site_result` packet. The site id auto-increments for the next test.

If the latest HK reports SAFE or FAULT, the pick screen shows a banner with *Clear and continue*, which issues `ensure_ready`.

Test types map to the FSW as follows. The FSW does not currently branch on `TestProfileId`; the enabled phases are what differ.

| UI test | Profile / tool | Repeat load | Post pen |
|---------|----------------|-------------|----------|
| Cone penetration (`cpt`) | 1 / 1 | no | no |
| Plate load (`plate`) | 1 / 2 | yes | yes (runs as full characterization with a warning event; a plate-only sequence is not in the FSW yet) |
| Full characterization (`full`) | 1 / 1 | yes | yes |

## 6. Engineer view

Toggle top-right. The choice persists in `localStorage`; toggling never disconnects or interrupts a test.

- **Status bar**: mode badge, site id, UDP endpoint, global Abort.
- **Sequencer pipeline**: pills for the phases from the `seq_status` packet (past green, current blue, future muted). TRACTION is hidden because it is a stub.
- **Action row**: Init, Deploy, Stow, and three presets (Standard baseline, Quick survey, Deep characterization) that reuse `ensure_ready` then `start_test`.
- **Manual actuator control** (collapsible): position move, force push, retract, hold, e-stop, clear e-stop (which re-runs the HAL init, the only thing that clears the latch). These route through `lrmh_seq` (`LRMH_SEQ_CMD_MID` function code 30, ADR-0018) and are accepted only when the sequencer phase is IDLE; the buttons are greyed out during a test. Force push and e-stop ask for confirmation.
- **HK packet inspector**: every field of the latest HK packet as a key/value grid.
- **cFS file browser**: read-only listing of `cf/lrmh/data/` (binary records) and `cf/lrmh/reports/` (human text); view text inline or download. Path traversal is rejected and a fetch is capped at 2 MiB.
- **Raw command sender**: MID, command code and hex payload, forwarded unchecked. Use only when a typed route does not cover the case.
- **Connection diagnostics**: WebSocket state, round-trip latency (Ping sends NOOP and times the ack), per-type telemetry rates over 5 s, staleness per type, reconnect count.
- **Bitmask decoders**: `sensorChannelValid` (FORCE, DEPTH, STROKE, CURRENT, BUS_V, TEMP) and the eleven `halFlags` bits, status bits in green, fault bits in amber.
- **Fault injection** (sim only): six buttons for the HAL sim's inject kinds plus *Clear injection*. The bridge sends `INJECT_FAULT` on UDP 17011; a bench or flight backend acknowledges and does nothing. Site selection (`set_site` with preset and seed) and actuator faults (`actuator_fault`: stall, overcurrent, encoder loss) are the other sim-only routes.
- **Metrics and plot**: force, depth, motor current, temperature; full-width force-vs-depth with the same baseline and post traces.
- **Event log**: info, warn and error events from the bridge, the last 100 or so.

## 7. Terminal tools

### 7.1 Console

```bash
python3 ground/lrmh_console.py [--host 127.0.0.1] [--port 1234]
```

Type `help`. Instrument commands: `noop`, `reset`, `init`, `deploy`, `start [args]`, `abort`, `stow`, `quickstart`. Sensor commands: `sense.start`, `sense.stop`, `zero.force`, `zero.depth`. Fault commands: `fault.reset`, `fault.enable`, `fault.disable`. History is kept in `~/.lrmh_history`.

The console's actuator commands (`move`, `force`, `retract`, `hold`, `estop`, `clearestop`) go through `lrmh_seq` manual control (function code 30) like the Engineer view, so they are accepted only while the sequencer is IDLE.

### 7.2 Scripted drivers

```bash
python3 ground/lrmh_ground_test.py --test full     # NOOP, INIT, DEPLOY, START, watch
python3 ground/lrmh_ground_test.py --test abort    # start then ABORT; expect SAFE
python3 ground/lrmh_ground_test.py --test noop

python3 scripts/lrmh_smoke_test.py                 # 2 cm, 50 N: a short test
python3 scripts/lrmh_smoke_test.py --full          # enable repeat, post and (stub) traction
python3 scripts/lrmh_smoke_test.py --noop | --init # stop after that step
```

The smoke test sends `DispatchFlags = OVERRIDE_HOST` and tails `build/logs/cfs.log` (override with `--log` or `LRMH_LOG_DIR`).

### 7.3 Reading results

Live, before the test ends:

```bash
python3 scripts/lrmh_inspect_record.py             # waits up to 120 s on UDP 2234 for 0x08AA
```

After the fact, from the DS files under `<cfs>/build/exe/cpu1/cf/lrmh/`:

```bash
python3 scripts/lrmh_read_record.py --latest                    # newest .bin or .txt
python3 scripts/lrmh_read_record.py --latest --prefer txt       # the human summary
python3 scripts/lrmh_read_record.py --json path/to/lrmh_site_NNN.bin > site.json
```

The reader strips the cFE file header (64 bytes) and the DS file header (76 bytes), walks the CCSDS packets and decodes the last one by MID. DS appends every captured record to the open file until it rotates (1 MB or 24 h), so a file usually holds several sites; the reader prints how many.

## 8. Monitoring a run

The decisive event stream in the cFS terminal is compact. Phase numbers follow `LRMH_Phase_t`: 1 PRECHECK, 2 POSITIONING, 3 BASELINE_PEN, 4 LOAD_RAMP, 5 REPEAT_LOAD, 6 POST_PEN, 7 TRACTION, 8 PROCESSING, 9 REPORTING, 0 IDLE.

| Event | Meaning |
|-------|---------|
| `LRMH_CMD: START_SITE_TEST Session=... Seq=... Site=... Profile=... Tool=...` | Dispatch accepted; mode READY to TESTING |
| `LRMH_SEQ: New test Session=... Seq=... Site=...` | Sequencer latched the parameters and bumped the token epoch |
| `LRMH_SEQ: Phase 2 → 3` | Phase advance (here POSITIONING to BASELINE_PEN); a phase advances only after its last actuator command has completed, so the label names the motion in progress |
| `LRMH_CTRL: mode=0 accepted (Pos=0.300 Force=500.0)` | Actuator command accepted (mode 0 position, 1 force, 2 retract, 3 hold, 4 e-stop) |
| `LRMH_CTRL: command complete (mode=0 stroke=0.2998)` | Actuator reached IDLE or HOLDING |
| `LRMH_SEQ: actuator command timed out in Phase=3 step 1` | The command's deadline passed; the site is skipped as T2 |
| `LRMH_SENSE: acquisition started` / `acquisition stopped (N samples)` | Sensor stream on and off |
| `LRMH_PROC: layer break at 0.0412 m` | Largest second difference of force over depth in a 5-sample window; noise-sensitive at the simulator's sample spacing, so treat it as indicative |
| `LRMH_PROC: phase 3 done MaxF=... Stiff=...` | Per-phase result published |
| `LRMH_PROC: phase extended → STREAMING at 10000 raw samples` | Long phase; results from the reservoir accumulator (not a fault) |
| `LRMH_SEQ: COMPLETE Site=N Quality=0` | Test done; quality 0 VALID, 1 PARTIAL, 2 FAULT |
| `LRMH_REPORT: site N published (quality=0, 3 phases)` | Site record and human text on the bus; DS writes the files |
| `LRMH_CMD: Mode 3 → 1 (trigger 5)` | TESTING back to STANDBY |
| `LRMH force saturation` (or `overcurrent`, `stroke hard stop`, `thermal`, `bus voltage`, `force rate`) | An LC actionpoint fired |
| `LRMH_FAULT: T1 RESUME (fault=0x0020, attempt 1)` | Tier 1 recovery: re-enter the phase |
| `LRMH_FAULT: T2 SKIP_SITE (fault=0x0004)` | Tier 2: finish the site early, tag PARTIAL |
| `LRMH_FAULT: T3 SAFE (fault=0x0002)` and `LRMH_CMD: SAFE entry, token epoch bumped` | Tier 3: retract, SAFE, wait for ground |
| `LRMH_CTRL: stale cmd dropped (rule=...)` | A command with a stale sequence token was discarded (normal after a recovery or re-init) |

Modes in HK and events: 0 OFF, 1 STANDBY, 2 READY, 3 TESTING, 4 SAFE, 5 FAULT.

## 9. Contingencies

### 9.1 SAFE entry

Any T3 fault (overcurrent, thermal, bus voltage, or T2 skips on three of the last five sites) or ground `ABORT` stops acquisition, retracts the actuator (a ground `ABORT` e-stops it first), bumps the token epoch and parks the instrument in SAFE. The site, if one was running, is reported with quality FAULT and whatever phases had completed.

1. Read the fault flag in the dispatch event or in HK `FaultFlags`: 0x0001 STALL, 0x0002 OVERCURRENT, 0x0004 FORCE_SATURATION, 0x0008 ENCODER_LOSS, 0x0010 THERMAL, 0x0020 FORCE_RATE, 0x0040 HARD_STOP, 0x0080 SENSOR_TIMEOUT, 0x0100 DEPTH_DISAGREE, 0x0200 BUS_VOLTAGE, 0x0400 HOST_NAV_TIMEOUT.
2. Clear the cause. In the sim, clear any injection (*Clear injection*, or `hal_bench_test.py --inject none`).
3. `INIT_INSTRUMENT` (console `init`, or *Clear and continue*). This re-initialises the HAL and takes SAFE to STANDBY. Then `DEPLOY` to READY.

### 9.2 FAULT mode

A T3 arriving while already in SAFE drives SAFE to FAULT (a fault outside TESTING is dispatched as T3, so from STANDBY or READY it goes to SAFE, and from SAFE to FAULT). From FAULT the only legal commands are `RESET_FAULT` (back to SAFE) and `POWER_OFF`. `INIT_INSTRUMENT` is rejected until `RESET_FAULT` has been sent. `lrmh_fault_test.py` sends `RESET_FAULT` first for exactly this reason.

### 9.3 Test will not start

`LRMH_CMD: START_SITE_TEST rejected, Mode=N (need READY)`: the instrument is not READY; `ensure_ready` or `init` then `deploy`.

`LRMH_CMD: START_SITE_TEST rejected, host nav stale or invalid`: no fresh host pose on the bus and the dispatch flags did not bypass the check. Send with `WITNESS_TARGET` (the UI default) or `OVERRIDE_HOST` (the smoke test default) when no host vehicle is present.

`LRMH_SEQ: START rejected, Phase=N (need IDLE)`: a previous test has not finished. Wait for COMPLETE or `ABORT`.

### 9.4 Result never arrives in the UI

The record is published once per site. If the cFS pipe depth is clamped by the kernel (`fs.mqueue.msg_max` below 1024) the single-shot 0x08AA and 0x08AB packets are the ones that get dropped, even though the DS file under `cf/lrmh/data/` is written. Fix the sysctl and restart the stack; see the Build and Install Guide.

### 9.5 Manual actuator moves

Only when the sequencer phase is IDLE. During a test the stop options are the global Abort (T3) or waiting for the test to finish. A manual command sent during a test produces `LRMH_SEQ: manual ctrl rejected, Phase=N (need IDLE)` and does nothing.

## 10. Wire schema

Client to bridge:

```jsonc
{"cmd": "noop"}
{"cmd": "init"}
{"cmd": "deploy"}
{"cmd": "stow"}
{"cmd": "abort"}
{"cmd": "ensure_ready"}
{"cmd": "start_test", "site_id": 17, "test_type": "cpt", "params": {"max_depth_m": 0.3, "max_load_n": 500}}
{"cmd": "ctrl_move" | "ctrl_force" | "ctrl_retract" | "ctrl_hold" | "ctrl_estop" | "ctrl_clear_estop", ...}
{"cmd": "inject_fault", "params": {"kind": "force-sat"}}           // sim only
{"cmd": "set_site", "preset": "lunar_mare", "seed": 7}             // sim only
{"cmd": "actuator_fault", "kind": "stall", "active": true}         // sim only
{"cmd": "presets_load" | "presets_save" | "presets_delete" | "history_load"}
{"cmd": "fs_list"} / {"cmd": "fs_fetch", "path": "reports/lrmh_human_....txt"}
{"cmd": "raw_cmd", "mid": "0x18A0", "cc": 0, "payload_hex": ""}
```

Bridge to client:

```jsonc
{"tlm": "hk", "mode": 2, "modeName": "READY", "seqState": 0, ...}
{"tlm": "sense", "force": 123.4, "depth": 0.148, "current": 0.42, "temp": 24.1, "t": 12.38}
{"tlm": "seq_status", "seqState": 3, "seqStateName": "BASELINE_PEN", ...}
{"tlm": "site_result", "siteId": 17, "qualityName": "VALID", ...}
{"tlm": "fault", "errorFlags": 1, ...}
{"event": "info|warn|error", "text": "..."}
{"ack": "<cmd>", "ok": true|false, ...}
```

Telemetry messages also carry a matching legacy `"type"` key; nothing in the tree consumes it and it can be removed.

## 11. Persistent bridge state

The bridge keeps a small JSON state directory at `ground/state/` (created on first launch, ignored by git):

| File | Purpose |
|------|---------|
| `presets.json` | Named parameter presets per test type, saved from the params screen. |
| `site_history.json` | Slim record per `site_result` (site id, quality, fault flags, tier, duration, max force, timestamp). The pick screen shows the last 8; truncated at 200 entries. The record of reference is always the DS file under `cf/lrmh/`. |

Both are server-side so they survive a browser swap.

## 12. Known gaps

- Plate-only test profile: the UI's Plate load runs as full characterization with a warning event; `TestProfileId` is carried into the record but does not change the phase sequence.
- Abort path: the UI moves to the Done screen with a Fault pill when Abort is pressed; the FSW publishes a FAULT-quality `site_result` with the phases that completed, which fills the cards.
- The site record's `fault_flags` and `recovery_tier` fields are written as 0 by `lrmh_report`; the dispatch events and HK are where fault state is visible.
- Phase skip, drive-to-phase, live LC threshold edits and a tool-table viewer have no FSW route yet and are not in the UI.
