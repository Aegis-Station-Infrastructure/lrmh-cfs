# LRMH cFS Flight Software

**Lunar Regolith Mechanics Head: a simulated geotechnical instrument on NASA cFS**

LRMH is a simulated lunar regolith instrument you can build and run on any
Linux box. It is a simulation and educational project: nothing here is
flight software or flight-qualified, and every number in it is a design
assumption for a simulated instrument, not a measurement. The flight
software is eight apps on NASA's core Flight System (cFS v7.0.0) that run
an instrumented penetration and load test at a site: push a cone into the
ground while sampling force, depth, stroke, current, bus voltage and
temperature at 100 Hz, load it to a target force, run a second
penetration, and reduce the samples onboard to penetration resistance, stiffness,
bearing and compaction quantities in a site record. Fault detection and
recovery run through NASA's LC, SC and HS apps as tables, with a thin
orchestrator app on top. All hardware access goes through a setpoint-level
Hardware Abstraction Layer: the included `sim` backend is a physics
simulation (a drawn regolith site with layers, a crust and buried clasts,
fitted to the Apollo 16 penetrometer records), and the `bench` backend is
the template for porting to real hardware. A browser UI, a terminal
console and record readers complete the loop from command to site record.

## Quick Start

Full detail and troubleshooting in the
[Build and Install Guide](docs/LRMH_Build_Install_Guide.md); the steps
below are the whole path.

```bash
# 0. One-time host setup (cFS pipes are POSIX mqueues; the default depth
#    drops the end-of-test record)
sudo sysctl fs.mqueue.msg_max=1024

# 1. Clone cFS v7.0.0 (pin the tag; the default branch is a different bundle)
git clone --branch v7.0.0 --recurse-submodules https://github.com/nasa/cFS.git ~/lrmh-cfs-build

# 2. Clone this repo WITH submodules (DS, FM, LC, SC, HS are pinned
#    submodules under vendor/) and install LRMH into the cFS tree
git clone --recurse-submodules https://github.com/Aegis-Station-Infrastructure/lrmh-cfs.git ~/lrmh-cfs
cd ~/lrmh-cfs
./install.sh ~/lrmh-cfs-build

# 3. Build the cFS image (MISSIONCONFIG is needed on prep: the tree now
#    has both sample_defs/ and lrmh_defs/)
cd ~/lrmh-cfs-build
make SIMULATION=native MISSIONCONFIG=sample prep
make -j$(nproc)
make install

# 4. Build the HAL simulator and the unit tests (no cFS needed for this part)
cd ~/lrmh-cfs
cmake -B build -S . -DTARGET=sim && cmake --build build && (cd build && ctest)

# 5. Check that all eight apps come up, then run the stack
scripts/lrmh_boot_check.sh ~/lrmh-cfs-build/build/exe/cpu1
scripts/lrmh_launch.sh          # HAL sim + core-cpu1 + bridge; UI on http://127.0.0.1:8080
```

`scripts/lrmh_launch.sh stop` tears it down. The launcher finds the cFS
tree through `LRMH_CFS_ROOT` (default `~/lrmh-cfs-build`).

## Your first site test

With the stack up, in a second terminal:

```bash
python3 scripts/lrmh_smoke_test.py --full
```

This sends NOOP, INIT_INSTRUMENT, DEPLOY and a START_SITE_TEST with the
repeat-load, post-penetration and (stub) traction phases enabled (2 cm, 50 N; the UI's
defaults are 0.30 m and 500 N). Watch the cFS log
(`tail -f build/logs/cfs.log`); the instrument narrates itself through
events. Phase numbers follow `LRMH_Phase_t`: 1 PRECHECK, 2 POSITIONING,
3 BASELINE_PEN, 4 LOAD_RAMP, 5 REPEAT_LOAD, 6 POST_PEN, 7 TRACTION,
8 PROCESSING, 9 REPORTING, 0 IDLE.

```
LRMH_CMD: START_SITE_TEST Session=43690 Seq=1 Site=1 Profile=0 Tool=0
LRMH_SEQ: Phase 0 → 1                                 IDLE to PRECHECK (actuator self-test)
LRMH_CTRL: mode=5 accepted (Pos=0.000 Force=0.0)      self-test
LRMH_CTRL: command complete (mode=5 stroke=0.0000)
LRMH_SEQ: Phase 1 → 2                                 POSITIONING
LRMH_SENSE: acquisition started
LRMH_CTRL: mode=1 accepted (Pos=0.000 Force=5.0)      5 N contact
LRMH_CTRL: command complete (mode=1 stroke=0.0083)
LRMH_SEQ: Phase 2 → 3                                 BASELINE_PEN
LRMH_CTRL: mode=0 accepted (Pos=0.020 Force=50.0)     penetration to 2 cm
LRMH_CTRL: command complete (mode=0 stroke=0.0195)
LRMH_SEQ: Phase 3 → 4                                 LOAD_RAMP: retract, then load
LRMH_PROC: layer break at 0.0106 m
LRMH_PROC: phase 3 done MaxF=17.6 Stiff=688.6
LRMH_REPORT: ingested phase 3 (1 so far)
LRMH_CTRL: mode=2 accepted (Pos=0.000 Force=0.0)      retract to home
LRMH_CTRL: command complete (mode=2 stroke=0.0007)
LRMH_CTRL: mode=1 accepted (Pos=0.000 Force=50.0)
LRMH_CTRL: command complete (mode=1 stroke=0.0501)
LRMH_SEQ: Phase 4 → 5                                 REPEAT_LOAD: 3 more cycles
...
LRMH_CTRL: mode=0 accepted (Pos=0.020 Force=50.0)     penetration to 2 cm
LRMH_CTRL: command complete (mode=0 stroke=0.0195)
LRMH_SEQ: Phase 6 → 7                                 TRACTION is a stub
LRMH_PROC: layer break at 0.0193 m
LRMH_PROC: phase 6 done MaxF=6.4 Stiff=155.3
LRMH_SEQ: Phase 7 → 8
LRMH_REPORT: ingested phase 6 (3 so far)
LRMH_SENSE: acquisition stopped (10000 samples)
LRMH_CTRL: mode=2 accepted (Pos=0.000 Force=0.0)      retract to home
LRMH_CTRL: command complete (mode=2 stroke=0.0006)
LRMH_SEQ: Phase 8 → 9                                 all results in
LRMH_SEQ: COMPLETE Site=1 Quality=0
LRMH_SEQ: Phase 9 → 0
LRMH_CMD: Mode 3 → 1 (trigger 5)                      TESTING back to STANDBY
```

The numeric values will differ run to run (the site is drawn from a seed
and the sensors are noisy); the sequence of events is what to look for.

**Success looks like:** `LRMH_SEQ: COMPLETE Site=1 Quality=0` (0 is VALID,
1 PARTIAL, 2 FAULT) followed by `LRMH_REPORT: site 1 published`, and the
record appended to the DS file under
`~/lrmh-cfs-build/build/exe/cpu1/cf/lrmh/data/` (`lrmh_site_<time>.bin`;
DS keeps appending to the open file until it rotates) with its
human-readable twin under `cf/lrmh/reports/`. Read it back:

```bash
python3 scripts/lrmh_read_record.py --latest --prefer txt
```

which prints the DS wrapper and then the summary:

```
=== LRMH Site Result ===
Session: 0xAAAA  Site: 1 (paired with site 0)
Profile: 0  Tool: 0
Outcome: VALID
Storage: RAW
...
Per-phase (3):
  phase=3 ok      n=1012  maxF=48.2 N depth=0.0200 m stiff=2410.5 N/m
  phase=5 ok      n=611   maxF=50.3 N depth=0.0214 m stiff=1890.2 N/m
  phase=6 ok      n=220   maxF=50.1 N depth=0.0216 m stiff=3107.9 N/m

Derived:
  ...
  Compaction delta:              0.289
```

`--latest` alone decodes the binary record instead; `--json` emits it as
JSON. The derived block is the onboard reduction of one simulated run;
the [Architecture Guide](docs/LRMH_Architecture_Guide.md) says what each
quantity is and where its current limits are. The same test from the browser: open the UI, pick *Full
characterization*, start it, and watch the force-versus-depth plot draw
the baseline and then the dashed post-disturbance trace.

## Architecture

```
            cFS / cFE  (ES, SB, EVS, TBL, TIME)   +  DS  FM  LC  SC  HS
                                |
                         (Software Bus)
                                |
   CMD ---- SEQ ---- CTRL ---- SENSE ---- PROC ---- REPORT ---- HK ---- FAULT
  (mode)  (phases) (actuator) (100 Hz)  (reduce)  (record)   (agg)  (FDIR glue)
                      |          |
                   LRMH HAL (setpoint level, hal/include/lrmh_hal.h)
                      |          |
                     UDP 127.0.0.1:17011
                                |
                  +-------------+-------------+
                  |                           |
            hal/sim (physics)          hal/bench (template)
```

**Mission:** `lrmh_cmd` owns the instrument mode (OFF, STANDBY, READY,
TESTING, SAFE, FAULT) and the dispatch protocol. `START_SITE_TEST` hands
a site to `lrmh_seq`, which walks PRECHECK (actuator self-test),
POSITIONING (contact), BASELINE_PEN (penetration to depth), LOAD_RAMP
(unload, then one load ramp), REPEAT_LOAD (optional, default three more
unload and reload cycles), POST_PEN (optional: retract, then a second
penetration down the same hole), TRACTION (deferred stub), PROCESSING and
REPORTING, advancing each step only when the actuator reports the move
complete or its deadline passes. Every
command the sequencer issues carries a sequence token (epoch, phase,
count) that the receiving app validates, so a stale or reordered command
can never act on the wrong phase or the wrong test. `lrmh_proc` reduces
each phase to maxima, a bearing proxy, a stiffness slope, a layer-break
depth and (for the second penetration) a compaction delta; `lrmh_report`
publishes a binary site record and a text summary that the DS app writes
to rotated files.

**FDIR:** six fault categories are LC watchpoints on the 100 Hz sample
packet (force, current, stroke, temperature, bus voltage low and high,
force rate). Each actionpoint fires an SC sequence that notifies
`lrmh_fault`, which looks up the recovery tier: T1 re-enters the phase,
T2 skips the rest of the site and tags it PARTIAL, T3 retracts and
safes the instrument. A hard site that saturates force is T2 data, not a
hardware fault. Sensor timeout is caught by a polled path in
`lrmh_fault`; HS restarts a hung app, and a restarted sequencer bumps the
token epoch so its predecessor's commands are discarded.

**HAL:** the apps issue position and force setpoints with clamps; the
servo loops, the clamps and the e-stop latch live below the HAL in the
backend. The sim backend is a separate process: a rate-limited actuator
model pushing a 1 cm^2 tip into a drawn regolith site (Carrier density
law, bearing-capacity cone resistance, layers, crust, clasts, a hole that
remembers the first pass), with Gaussian sensor noise, dropouts and three
kinds of fault injection. The bench backend has the same UDP envelope
with every handler marked `TODO(bench)`. There is no flight backend.

## Apps

| App | cFS name | Rate | Role |
|-----|----------|------|------|
| `lrmh_cmd` | LRMH_CMD | event | Mode FSM, START_SITE_TEST dispatch and idempotency cache, token epoch on INIT and SAFE |
| `lrmh_seq` | LRMH_SEQ | 10 Hz | Nine-phase site test, token stamping, T1/T2/T3 execution, manual actuator route |
| `lrmh_ctrl` | LRMH_CTRL | 10 Hz | Sequencer commands to HAL calls, completion polling, token validation |
| `lrmh_sense` | LRMH_SENSE | 100 Hz | Sensor read, per-channel freshness, force rate, host pose alignment |
| `lrmh_proc` | LRMH_PROC | event | Per-phase engineering quantities; RAW to STREAMING accumulator on long phases |
| `lrmh_report` | LRMH_REPORT | event | Site record and human text on two MIDs for DS |
| `lrmh_hk` | LRMH_HK | 1 Hz | Aggregated housekeeping |
| `lrmh_fault` | LRMH_FAULT | 10 Hz | Tier lookup, escalation, cross-app recovery dispatch, polled sensor timeout |

## Ground tools

- `ground/lrmh_ui.html`: operator and engineer views in one page, served by the bridge. See the [Operator Guide](docs/LRMH_Operator_Guide.md).
- `ground/lrmh_bridge.py`: WebSocket and HTTP bridge to CI_LAB and TO_LAB. Binds to `127.0.0.1` by default; `--ws-host 0.0.0.0` exposes it to the local network. **There is no authentication on the WebSocket.**
- `ground/lrmh_console.py`: terminal CLI straight to CI_LAB; `ground/lrmh_ground_test.py --test full|abort|noop`: scripted driver.
- `scripts/lrmh_smoke_test.py`: NOOP, INIT, DEPLOY, START against CI_LAB; `scripts/lrmh_fault_test.py --kind force-sat|overcurrent|hard-stop|thermal|bus-voltage|force-rate|all`: injects each fault and checks the LC, SC, `lrmh_fault` chain.
- `scripts/lrmh_read_record.py`, `scripts/lrmh_inspect_record.py`: decode the site record from a DS file or from the live downlink.
- `scripts/hal_bench_test.py`: eight scenarios against a HAL backend with no cFS in the loop; the bench porting loop.
- `scripts/lrmh_boot_check.sh`: boot a built tree and confirm the eight apps initialise (what CI runs).
- `sim/tools/regolith_probe.c`, `sim/tools/apollo16_compare.py`: push a cone through a drawn site; lay the lunar presets against the Apollo 16 records.

## Documentation

- [Architecture Guide](docs/LRMH_Architecture_Guide.md): apps, MIDs, command codes, telemetry, FDIR tables, token protocol, configuration
- [Build and Install Guide](docs/LRMH_Build_Install_Guide.md): step-by-step setup, what `install.sh` patches, troubleshooting
- [Operator Guide](docs/LRMH_Operator_Guide.md): UI, bridge, console, scripted drivers, event stream, contingencies, wire schema
- [HAL Porting Guide](docs/LRMH_HAL_Porting_Guide.md): the contract a bench or flight backend must meet
- [Sim Model Spec](docs/LRMH_Sim_Model_Spec.md): the regolith, actuator and sensor models and their sources
- [Design Decisions](docs/LRMH_Design_Decisions.md): what ADR-0001 through ADR-0018 in the code comments resolve to

## Message IDs

Commands `0x18A0` to `0x18A7` (one per app) | Internal `0x18B0` to `0x18B8` | Wakeups `0x18C0` to `0x18C2` | Telemetry `0x08A0` to `0x08AC` (HK, per-app status, 100 Hz samples, phase results, the site record on `0x08AA` and its text on `0x08AB`) | Inbound host pose and velocity `0x08C5`, `0x08C6`.

`install.sh` relocates LC's command topics (upstream 0xA4 to 0xA6) to 0xC3 to 0xC5 because they collide with the LRMH block. Authoritative list: `msg/lrmh_msgids.h`.

## Linux host notes

- `fs.mqueue.msg_max` must be at least 1024. The kernel default of 10 silently clamps every cFS pipe; the packets that then get dropped are the single-shot end-of-test record and summary, so the DS file appears but the UI never shows a result. The launcher checks and tries `sudo sysctl` for you.
- SC boots with every RTS disabled. The launcher enables RTS 1 to 9; without that, LC actionpoints fire but nothing is dispatched.
- Site records accumulate under `<cfs>/build/exe/cpu1/cf/lrmh/`; a clean rebuild wipes the directory and the launcher recreates it. Run one telemetry consumer at a time: the bridge and `lrmh_inspect_record.py` both bind UDP 2234.

## Status

Verified on NASA cFS v7.0.0 (the `v7.0.0` bundle tag) on Ubuntu-family Linux with the `sim` HAL backend. A full site test with the repeat-load and post-penetration phases runs to COMPLETE against the simulator, writes the DS site record, and `scripts/lrmh_fault_test.py` drives each of the six LC-detected fault categories through LC, SC and `lrmh_fault` to its expected tier. The eight per-app unit tests pass under ctest. The processing quantities are consistent with the simulator by construction (the signal and the ground truth come from the same model), which is a self-consistency check, not a fidelity claim. Known limits, stated so nobody has to discover them: PRECHECK runs the actuator self-test but not a witness-target check, and TRACTION passes straight through; the layer-break detector is noise-sensitive at the simulator's sample spacing and the compaction delta compares regression slopes of the two penetrations, so both are indicative rather than measured; stall, encoder loss, depth disagreement and host-nav timeout have tier entries but no detector yet; the record's fault and host-pose fields are not populated; the bench backend is a template and there is no flight backend. The [HAL Porting Guide](docs/LRMH_HAL_Porting_Guide.md) describes what real hardware has to provide.

## License

Apache License 2.0; see [LICENSE](LICENSE) and [NOTICE](NOTICE).
Copyright 2026 Aegis Station Infrastructure LLC. NASA cFS and the five
NASA support apps are not part of this repository; the support apps are
pulled in as git submodules from NASA's repositories under their own
Apache 2.0 terms.

This is a simulation and educational project. Nothing here is
flight-qualified.
