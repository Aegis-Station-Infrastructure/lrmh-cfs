# LRMH cFS Flight Software Architecture Guide

**Document:** LRMH-AD-001
**Version:** 1.0
**Date:** October 2026
**Status:** As built, verified on NASA cFS v7.0.0 with the sim HAL backend

*Scope: the full technical reference for the eight mission apps, the message set, the FDIR tables and the HAL boundary. Design rationale lives in the Design Decisions index; this document says what the code does.*

---

## 1. Mission overview

The Lunar Regolith Mechanics Head (LRMH) is a simulated geotechnical instrument: a linear actuator driving a penetrator tool through a commanded stroke profile while force, depth, stroke, motor current, bus voltage and temperature are sampled at 100 Hz. It is mounted on a host vehicle that positions it over a site and provides power, data and a pose. A single **site test** walks nine phases and produces a **site result**: penetration resistance versus depth, a load-sinkage stiffness, a bearing proxy, a layer-break depth and a compaction delta between the undisturbed and disturbed passes.

The instrument implements five functional requirements from the RMH-class capability definition published by Aegis Station Infrastructure LLC: normal load testing (RMH-F-001), penetration profiling (RMH-F-002), disturbance sensitivity (RMH-F-003), traction correlation (RMH-F-004, deferred) and time synchronisation to the host vehicle (RMH-F-005). The capability definition contains no numbers; every envelope value in this repository is a project-level engineering choice for a simulated instrument, not a measurement (ADR-0012).

**Site test phases** (`LRMH_Phase_t`, `msg/lrmh_msg.h`):

| # | Phase | Source | What happens |
|---|-------|--------|--------------|
| 1 | `PRECHECK` | bookend | Actuator self-test (`LRMH_HAL_Init` through `lrmh_ctrl`); the witness-target check is not implemented |
| 2 | `POSITIONING` | bookend | Force-mode contact at 5 N with a 3 N/s ramp; sensor acquisition starts |
| 3 | `BASELINE_PEN` | RMH-F-002 | Position-mode penetration to `MaxDepth_m` at 5 mm/s with `MaxLoad_N` as the force clamp |
| 4 | `LOAD_RAMP` | RMH-F-001 | Unload (retract to home), then ramp force to `MaxLoad_N` at 3 N/s: one load-sinkage cycle |
| 5 | `REPEAT_LOAD` | RMH-F-001/003 | `RepeatLoadCount` (default 3) more cycles of unload and reload to `MaxLoad_N`; optional. The load samples from LOAD_RAMP on are processed as one dataset |
| 6 | `POST_PEN` | RMH-F-003 | Retract to home, then a second position-mode penetration to `MaxDepth_m` down the same hole; optional |
| 7 | `TRACTION` | RMH-F-004 | Deferred stub; passes straight through |
| 8 | `PROCESSING` | bookend | Acquisition stops, actuator retracts, the sequencer waits until every processing trigger has been answered |
| 9 | `REPORTING` | bookend | Triggers `lrmh_report`, notifies `lrmh_cmd`, returns to IDLE |

A move ends at its depth target, its force clamp, or its deadline: every actuator command carries a timeout, and a command that has not completed by then is abandoned and the site is skipped as T2 (during the final retract it is simply abandoned so the site is still reported). Hard regolith that saturates force early and soft regolith that reaches depth on little force are both valid results, not faults.

### 1.1 Terminology

- **Mode**: the instrument lifecycle state owned by `lrmh_cmd` (OFF, STANDBY, READY, TESTING, SAFE, FAULT).
- **Phase**: the site-test step owned by `lrmh_seq`, meaningful only while the mode is TESTING.
- **Sequence token**: the 32-bit stamp on every sequencer-issued command (ADR-0007).
- **Recovery tier**: T1 resume the phase, T2 skip the rest of the site, T3 safe the instrument (ADR-0008).
- **Envelope**: a hardware, operational or science limit from `cfs_defs/lrmh_platform_cfg.h`; the LC threshold sits inside the envelope so the FSW reacts before the limit is crossed.

## 2. Software architecture

### 2.1 Framework

LRMH runs on NASA's core Flight System v7.0.0. The flight software is eight mission apps on the Software Bus plus five NASA support apps (DS, FM, LC, SC, HS) that carry the storage and FDIR policy as tables. All hardware access goes through one HAL header and a UDP client to a separate backend process.

### 2.2 Application set

| App | cFS name | Rate | Role |
|-----|----------|------|------|
| `lrmh_cmd` | LRMH_CMD | event driven | Front door: mode FSM, dispatch cache, token epoch bumps on INIT and SAFE |
| `lrmh_seq` | LRMH_SEQ | 10 Hz | Site-test phase machine, token stamping, recovery-tier execution |
| `lrmh_ctrl` | LRMH_CTRL | 10 Hz | Translates sequencer commands into HAL calls, polls for completion, validates tokens |
| `lrmh_sense` | LRMH_SENSE | 100 Hz | Sensor acquisition, per-channel freshness, force-rate, host pose alignment |
| `lrmh_proc` | LRMH_PROC | event driven | Per-phase engineering quantities from the sample stream |
| `lrmh_report` | LRMH_REPORT | event driven | Assembles the site record and human text; publishes for DS |
| `lrmh_hk` | LRMH_HK | 1 Hz | Aggregates the other apps' status into one HK packet |
| `lrmh_fault` | LRMH_FAULT | 10 Hz | Thin orchestrator: LC notifications in, recovery-tier dispatch out, polled sensor timeout |

All eight load at priority 100 with 16 KB stacks; the support apps load first at 95 to 99.

### 2.3 Software bus architecture

```
                    cFS / cFE  (ES, SB, EVS, TBL, TIME)
                                  |
                           (Software Bus)
                                  |
   ground --> CMD --> SEQ --+--> CTRL --> HAL client --UDP--> backend
                       |    +--> SENSE <-- HAL client <--UDP-- (sim | bench)
                       |    +--> PROC --> REPORT --> DS files
                       |
         LC --> SC --> FAULT --> SEQ (T1/T2) / CMD (T3)
         HS (app restarts)            HK (aggregate)
```

### 2.4 Message flow

```
CMD  --START_SITE_TEST (0x18A2)--> SEQ
SEQ  --CtrlCmd (0x18B0)----------> CTRL --CTRL_STATUS (0x08A2)--> SEQ, HK
SEQ  --start/stop (0x18B1)-------> SENSE --SAMPLE (0x08A3)------> PROC, FAULT, HK, LC
SEQ  --trigger (0x18B2)----------> PROC --PROC_TLM (0x08A4)-----> SEQ, REPORT
SEQ  --trigger (0x18B3)----------> REPORT --0x08AA, 0x08AB------> DS, ground
SEQ  --notify (0x18B8)-----------> CMD   (TEST_COMPLETE: TESTING -> STANDBY)
SEQ  --phase advance-------------> FAULT (resets T1 counter)
LC   --RTS n----------------------> SC --LC_NOTIFY (0x18A7 cc 10)--> FAULT
FAULT --recovery (0x18B4)---------> SEQ   (T1 RESUME, T2 SKIP, T3 SAFE)
CMD/SEQ --TOKEN_RESET (0x18B7)----> CTRL, SENSE, PROC, REPORT
SEQ  --SAFE broadcast (0x18B6)----> CTRL (defensive retract)
```

SCH_LAB drives the wakeups: `LRMH_SENSE_WAKEUP_MID` at 10 Hz (the sense app paces its 100 Hz acquisition on its own 10 ms receive timeout), `LRMH_SEQ_WAKEUP_MID` and `LRMH_FAULT_WAKEUP_MID` at 1 Hz (both apps pace themselves at 10 Hz on their receive timeouts), `LRMH_SEND_HK_MID` at 1 Hz, plus LC's sample-actionpoints message and SC's wakeup at 10 Hz. The sequencer also runs its phase machine after every received message, with a 100 ms receive timeout, so it advances at 10 Hz regardless of the wakeup period.

## 3. Message ID allocation

LRMH owns topic block 0xA0 to 0xC2 (ADR-0005). The stock apps sit at 0x80 (TO_LAB), 0x82 (SAMPLE_APP) and 0x84 (CI_LAB); `install.sh` relocates LC's command topics from 0xA4 to 0xA6 to 0xC3 to 0xC5 because they collide. The authoritative list is `msg/lrmh_msgids.h`.

### 3.1 Command MIDs (ground or host to instrument)

| MID | Name | Description |
|-----|------|-------------|
| 0x18A0 | `LRMH_CMD_MID` | Instrument-level commands (`lrmh_cmd`) |
| 0x18A1 | `LRMH_SEND_HK_MID` | HK request (`lrmh_hk`) |
| 0x18A2 | `LRMH_SEQ_CMD_MID` | Sequencer commands, including START forwarded by `lrmh_cmd` and manual actuator control |
| 0x18A3 | `LRMH_CTRL_CMD_MID` | Controller ground commands (NOOP, RESET only) |
| 0x18A4 | `LRMH_SENSE_CMD_MID` | Sensor commands |
| 0x18A5 | `LRMH_PROC_CMD_MID` | Processing commands |
| 0x18A6 | `LRMH_REPORT_CMD_MID` | Report commands |
| 0x18A7 | `LRMH_FAULT_CMD_MID` | Fault commands, including the LC notification from SC |

### 3.2 Internal MIDs (app to app)

| MID | Name | Flow |
|-----|------|------|
| 0x18B0 | `LRMH_SEQ_TO_CTRL_MID` | Sequencer to controller (tokened) |
| 0x18B1 | `LRMH_SEQ_TO_SENSE_MID` | Sequencer to sensor start/stop (tokened) |
| 0x18B2 | `LRMH_SEQ_TO_PROC_MID` | Sequencer to processing trigger (tokened) |
| 0x18B3 | `LRMH_SEQ_TO_REPORT_MID` | Sequencer to report trigger (tokened) |
| 0x18B4 | `LRMH_FAULT_TO_SEQ_MID` | Fault orchestrator to sequencer (recovery tier) |
| 0x18B5 | `LRMH_FAULT_TO_CTRL_MID` | Fault orchestrator to controller (retract) |
| 0x18B6 | `LRMH_SEQ_SAFE_MID` | SAFE broadcast |
| 0x18B7 | `LRMH_TOKEN_RESET_MID` | Token epoch reset broadcast |
| 0x18B8 | `LRMH_SEQ_TO_CMD_MID` | Sequencer to mode FSM (TEST_COMPLETE) |
| 0x18C3 | `LRMH_LC_CMD_MID` | NASA LC's command MID after install.sh relocates it; `lrmh_fault` re-arms actionpoints on it |
| 0x18C0 | `LRMH_SEQ_WAKEUP_MID` | Scheduler wakeup, `lrmh_seq` |
| 0x18C1 | `LRMH_SENSE_WAKEUP_MID` | Scheduler wakeup, `lrmh_sense` (100 Hz) |
| 0x18C2 | `LRMH_FAULT_WAKEUP_MID` | Scheduler wakeup, `lrmh_fault` |

### 3.3 Telemetry MIDs (instrument to ground)

| MID | Name | Source | Contents |
|-----|------|--------|----------|
| 0x08A0 | `LRMH_HK_TLM_MID` | HK | Aggregated instrument HK (mode, phase, HAL state and flags, sensor snapshot, fault state, counters) |
| 0x08A1 | `LRMH_SEQ_STATUS_MID` | SEQ | Phase, quality, site/session ids, token epoch, repeat cycles, elapsed time |
| 0x08A2 | `LRMH_CTRL_STATUS_MID` | CTRL | Active mode, pending/complete, HAL state and flags, accepted token, stroke |
| 0x08A3 | `LRMH_SENSE_TLM_MID` | SENSE | One sample per publication at 100 Hz: six channels, `channel_valid`, phase, force rate, host pose and velocity with ages |
| 0x08A4 | `LRMH_PROC_TLM_MID` | PROC | Per-phase result |
| 0x08A5 | reserved | | |
| 0x08A6 | `LRMH_FAULT_TLM_MID` | FAULT | Fault flags, active, tier, attempt counters |
| 0x08A7, 0x08A8 | reserved | | Superseded host-state and compact site-result MIDs |
| 0x08A9 | `LRMH_PROC_PHASE_EXTENDED_TLM_MID` | PROC | One-shot notice that a phase switched from RAW to STREAMING storage |
| 0x08AA | `LRMH_REPORT_SITE_RECORD_MID` | REPORT | Binary `LRMH_SiteResultRecord_t`; captured by DS |
| 0x08AB | `LRMH_REPORT_SITE_HUMAN_MID` | REPORT | ASCII site summary; captured by DS |
| 0x08AC | `LRMH_CMD_STATUS_MID` | CMD | Mode, counters, dispatch cache hits, current session |

Inbound subscriptions (published by a host vehicle, not owned by LRMH): `0x08C5` host pose, `0x08C6` host velocity (ADR-0011). `0x08D0` to `0x08D7` are reserved for host vehicles whose own MIDs collide.

### 3.4 CCSDS packet format

v7.0.0 default configuration, as deployed. Commands: 6-byte primary header, 2-byte command secondary header (function code, checksum), payload from byte 8. Telemetry: 6-byte primary header, 6-byte time secondary header, 4 bytes of alignment padding, payload from byte 16. The ground tools build and parse exactly these offsets; CI_LAB does not validate the command checksum.

## 4. Application design

### 4.1 LRMH_CMD (front door and mode FSM)

**Purpose:** owns the six-mode instrument FSM (ADR-0004), the START_SITE_TEST dispatch protocol (ADR-0014) and the token epoch bumps for INIT and SAFE (ADR-0007). It is deliberately separate from the sequencer so that a sequencer crash never takes down the command surface; HS takes no restart action on it.

**Command codes (`LRMH_CMD_MID`):**

| CC | Name | Payload | Description |
|----|------|---------|-------------|
| 0 | NOOP | | Connectivity test; events `LRMH_CMD: NOOP Cmd=n Err=n Mode=n` |
| 1 | RESET_COUNTERS | | Zero counters |
| 2 | INIT_INSTRUMENT | | `LRMH_HAL_Init`, then OFF to STANDBY or SAFE to STANDBY; rejected from FAULT until RESET_FAULT has taken the mode back to SAFE. Bumps the token epoch and broadcasts the reset. No-op on mode in STANDBY, READY or TESTING apart from the epoch bump. A failed HAL init leaves the mode unchanged. |
| 3 | DEPLOY | | STANDBY to READY (from OFF: OFF to STANDBY) |
| 4 | STOW | | `LRMH_HAL_Retract`, READY to STANDBY. Rejected from STANDBY (use POWER_OFF). |
| 5 | ABORT | | Unconditional T3: TESTING/READY/STANDBY to SAFE |
| 6 | START_SITE_TEST | `LRMH_StartSiteTestCmd_t` | Dispatch a site test (below) |
| 7 | REQUEST_REPORT | | Reserved |
| 8 | SET_MODE | `uint8 NewMode` | Rejected: not implemented |
| 9 | RESET_FAULT | | FAULT to SAFE |
| 10 | SAFE | | Mode to SAFE without an e-stop; used by the T3 fallback RTS |
| 11 | POWER_OFF | | STANDBY to OFF |

**Mode transition matrix** (from, trigger) as built:

| From | Legal transitions |
|------|-------------------|
| OFF | POWER_ON or DEPLOY to STANDBY; EMERGENCY_OFF stays OFF |
| STANDBY | READY_FOR_TEST to READY; FAULT_T1_T2, FAULT_T3 or ABORT to SAFE; INIT stays STANDBY; EMERGENCY_OFF or POWER_OFF to OFF |
| READY | START_SITE_TEST to TESTING; STOW or INIT to STANDBY; any fault or ABORT to SAFE; EMERGENCY_OFF to OFF |
| TESTING | TEST_COMPLETE to STANDBY; FAULT_T1_T2 stays TESTING (recovery is the sequencer's job); FAULT_T3 or ABORT to SAFE; EMERGENCY_OFF to OFF |
| SAFE | INIT to STANDBY; FAULT_T3 to FAULT; EMERGENCY_OFF to OFF |
| FAULT | RESET_FAULT to SAFE; EMERGENCY_OFF to OFF |

Illegal transitions event `LRMH_CMD: illegal transition Mode=n Trigger=n` and increment `RejectedCount`. SAFE entry events at CRITICAL level and bumps the token epoch.

**START_SITE_TEST payload** (`LRMH_StartSiteTestCmd_t`, 36 bytes after the header): `SiteId` (u16), `TestProfileId`, `ToolId`, `MaxDepth_m` (float), `MaxLoad_N` (float), `EnableRepeatLoad`, `EnablePostProfile`, `EnableTraction`, `DispatchFlags`, `SessionId`, `SequenceNumber`, `PairedSiteId`, `MicroSiteOffsetX_mm`, `MicroSiteOffsetY_mm`, `RepeatLoadCount` (0 means 3). `TestProfileId` and `ToolId` are carried into the site record but do not change the phase sequence.

**Dispatch flags** (ADR-0014): bit 0 `STRICT_NAV` (refuse unless the cached host pose is fresh and the host nav state is IDLE), bit 1 `IDEMPOTENT` (retry; the 8-entry cache keyed on session and sequence number absorbs a repeat without starting a second test), bit 2 `OVERRIDE_HOST` (accept regardless of host motion), bit 3 `WITNESS_TARGET` (test against the instrument's witness target; bypasses the host check entirely). With no flags the command is accepted if a host pose no older than `LRMH_HOST_NAV_MAX_AGE_MS` (500 ms) is cached.

Preconditions in order: idempotency cache, mode is READY, host nav acceptable. Only after both pass is the token epoch bumped and the command forwarded to `lrmh_seq` on `LRMH_SEQ_CMD_MID`.

### 4.2 LRMH_SEQ (sequencer)

**Purpose:** owns the phase machine, stamps a sequence token on every outbound command, and executes recovery tiers. It never touches the HAL.

**Command codes (`LRMH_SEQ_CMD_MID`):**

| CC | Name | Payload | Description |
|----|------|---------|-------------|
| 0 | NOOP | | |
| 1 | RESET_COUNTERS | | |
| 5 | ABORT | | T3: stop acquisition, retract, SAFE broadcast, phase to IDLE |
| 6 | START_SITE_TEST | as above | Forwarded by `lrmh_cmd`; rejected unless phase is IDLE |
| 20 | RESUME | | T1: re-enter the current phase (from `lrmh_fault` or the fallback RTS) |
| 21 | SKIP | | T2: quality PARTIAL, jump to PROCESSING |
| 30 | MANUAL_CTRL | `LRMH_SeqManualCtrlCmd_t` (mode, position, force, rate, timeout) | Ground actuator command; accepted only in phase IDLE, then stamped and forwarded to `lrmh_ctrl` (ADR-0018) |

**Phase machine:** runs every receive-loop pass. A phase is a short step sequence (`PhaseStep`): each step issues at most one actuator command, sets `WaitingForCtrl` and arms the command's deadline, and the machine then does nothing until `lrmh_ctrl` publishes `CommandComplete = 1` or the deadline passes. The phase advances only after its last command has completed, so the phase label in telemetry names the motion in progress. Before each measured segment (the contact move, each penetration, the first load ramp) the sequencer sends `lrmh_proc` a segment-start command so a preceding retract is not part of the dataset and the phase start time is right; after each measured segment it sends the processing trigger. Every transition events `LRMH_SEQ: Phase a → b`, updates the token's phase byte, and notifies `lrmh_fault` so the T1 attempt counter resets.

Actuator parameters issued by the sequencer: self-test, 30 s; POSITIONING force 5 N at 3 N/s, 30 s; penetration position `MaxDepth_m` at 5 mm/s with `MaxLoad_N` clamp, 120 s; load ramps to `MaxLoad_N` at 3 N/s with a deadline of the ramp time plus 30 s; retracts at 8 mm/s, 30 s. The sequencer enforces these deadlines against the test's elapsed wall-clock time; the HAL sim separately applies its own 60 s and 30 s model timeouts and marks a timed-out command complete.

**Recovery (ADR-0008):** T1 increments `T1AttemptsCurrentPhase` and re-enters the phase from its first step (an in-flight command is superseded); the fourth attempt in one phase escalates to T2. T1 is rejected in IDLE and in PROCESSING or later. T2 sets quality PARTIAL, drops any in-flight wait and transitions to PROCESSING so the site is still reported with whatever was collected. T3 sets quality FAULT, stops acquisition, retracts, broadcasts SAFE and returns to IDLE; `lrmh_cmd` moves the mode to SAFE in parallel.

**Status telemetry** (`LRMH_SEQ_STATUS_MID`): phase, previous phase, quality, `WaitingForCtrl`, site/session/sequence ids, token epoch, sample count, counters, T1 attempts, repeat cycles done/target, elapsed and phase-start time.

### 4.3 LRMH_CTRL (actuator controller)

**Purpose:** the only app that commands the HAL. Receives `LRMH_CtrlCmd_t` on `LRMH_SEQ_TO_CTRL_MID` (token validated), on `LRMH_FAULT_TO_CTRL_MID` (reserved; nothing sends it today) and on its own ground MID (not validated; ground is limited to NOOP and RESET).

**Modes** (`Mode` byte in the command): 0 POSITION (`LRMH_HAL_CommandPosition`), 1 FORCE (`CommandForce`), 2 RETRACT, 3 HOLD, 4 ESTOP, 5 PRECHECK (`LRMH_HAL_Init` self-test). `TargetPosition_m` is the tool-tip depth; `RateLimit` is m/s for position and N/s for force.

**Completion:** on every 10 Hz wakeup it polls `LRMH_HAL_GetHealth`; a pending command completes when the HAL state is IDLE or HOLDING. The status packet carries `CommandComplete = 1` for exactly one publication, which is what the sequencer gates on. On `LRMH_SEQ_SAFE_MID` it calls `LRMH_HAL_Retract` directly as a defensive measure before the SAFE RTS arrives.

**Token handling:** rule 2 of ADR-0007 is implemented here as `ResetObserved`: the first command after a `LRMH_TOKEN_RESET_MID` is accepted unconditionally and re-baselines the accepted token. Rejections event `LRMH_CTRL: stale cmd dropped (rule=n, incoming E=.. S=.. C=.., accepted ...)` and increment `StaleCount`.

### 4.4 LRMH_SENSE (sensor acquisition)

**Purpose:** at 100 Hz, while acquiring, call `LRMH_HAL_ReadSensors`, apply the freshness contract (ADR-0009), compute the force rate, attach the host pose and velocity alignment (ADR-0011) and publish one `LRMH_SampleTlm_t`.

**Command codes (`LRMH_SENSE_CMD_MID` and `LRMH_SEQ_TO_SENSE_MID`):** 0 NOOP, 1 RESET, 2 START (tokened from the sequencer), 3 STOP (tokened), 4 ZERO_FORCE, 5 ZERO_DEPTH (the two zero commands event and count; the tare itself is a backend responsibility).

**Freshness:** the HAL timestamp is rebased to cFE time at the read boundary (there is no cross-clock calibration), so per-channel age compares are not applied; what is enforced is the backend's `valid` and `channel_valid` bits. A channel stale for three consecutive reads (`LRMH_SENSE_CONSECUTIVE_TIMEOUT`) raises `LRMH_SENSE: SENSOR_TIMEOUT (channels stale)` and increments `TimeoutCount`. Recovery after a streak of two or more cycles is evented.

**Force rate:** `(force_n - last_force_n) / dt` between consecutive samples with a fresh force channel, published as `force_rate_nps`; LC watchpoint 6 reads it.

**Host alignment:** the app subscribes to the host pose and velocity MIDs, caches them with receipt time, and stamps every sample with the host state and its age in milliseconds. A missing host simply leaves `host_pose_valid = 0`.

### 4.5 LRMH_PROC (processing)

**Purpose:** turn each phase's samples into engineering quantities by signal processing alone (ADR-0002): no soil model, no classifier.

**Ingest:** every `LRMH_SampleTlm_t` is appended to a raw buffer of `LRMH_MAX_SAMPLES_PER_PHASE` (10000, about 100 s at 100 Hz). A sample older than the current phase start is dropped (`LRMH_PROC: stale sample dropped`). When the buffer fills the phase switches one way to STREAMING (ADR-0015): per-channel running statistics, a 1000-entry reservoir sample, the first and last 500 raw samples as provenance, and a 32 x 32 force-versus-depth histogram. The switch publishes `LRMH_PROC_PHASE_EXTENDED_TLM_MID` once and events `LRMH_PROC: phase extended → STREAMING at N raw samples`; it is a notice, not a fault.

**Per-phase quantities** (`LRMH_ProcPhaseResult_t`), computed on the sequencer's trigger:

| Quantity | Method |
|----------|--------|
| `max_force_n`, `max_sinkage_m` | Maxima over the phase |
| `bearing_proxy` | Force interpolated at 1 cm depth divided by `LRMH_PROC_BEARING_REF_N` (500 N, about 5 MPa on the 1 cm^2 tip) |
| `layer_break_depth_m` | Depth of the largest second derivative of force with respect to depth over a 5-sample window, if it exceeds `LRMH_PROC_LAYER_BREAK_D2F_THRESHOLD` (1000 N/m^2); needs at least 20 samples. At the simulator's sample spacing and noise this detector is dominated by noise and almost always reports a depth; treat it as indicative, not as a detected layer |
| `stiffness_proxy_npm` | Slope of a linear regression of force on depth over the middle 20% to 80% of the depth-sorted samples; needs at least 10 |
| `compaction_delta` | For POST_PEN only: (post stiffness minus baseline stiffness) / baseline stiffness |
| `duration_s`, `sample_count`, `storage_mode`, `quality` | Fewer than 10 samples marks the phase PARTIAL |

`variability_index` is reserved and currently always 0. In STREAMING mode the same quantities are computed from the reservoir and histogram.

### 4.6 LRMH_REPORT (site result)

**Purpose:** collect the per-phase results, assemble the binary site record and its human-readable rendering, and publish both for DS to capture (ADR-0016). No file I/O happens in the app.

**Command codes (`LRMH_REPORT_CMD_MID` and `LRMH_SEQ_TO_REPORT_MID`):** 0 NOOP, 1 RESET, 2 GENERATE (tokened trigger from the sequencer carrying site, session, sequence, paired site, profile, tool, quality and start time), 30 TAG_PARTIAL (ground: publish the site so far as PARTIAL).

**Record** (`LRMH_SiteResultRecord_t`, version 1, published on `0x08AA`): identification (session, sequence, site, paired site, profile, tool, outcome, storage mode), start and end time, host pose at start, up to 8 per-phase summaries (phase, quality, sample count, max force, max sinkage, stiffness, bearing proxy, layer break, duration), and aggregates: mean and peak penetration resistance in kPa (peak force over the nominal 1 cm^2 tip area, `LRMH_TOOL_TIP_AREA_M2`), load-sinkage stiffness in kN/m, bearing proxy, first layer-break depth, largest compaction delta, variability. `fault_flags` and `recovery_tier` are written as 0 in this release.

**Human text** (`0x08AB`, up to 2048 bytes): a fixed template beginning `=== LRMH Site Result ===` with session, site, profile, outcome (VALID, PARTIAL, FAULT), storage mode, times, host pose, one line per phase and the derived block. It is meant to be read straight from the DS file.

Events: `LRMH_REPORT: ingested phase n (m so far)` per phase result, `LRMH_REPORT: site N published (quality=q, n phases)` on publication, plus `[rec]`, `[agg]` and `[phN]` lines echoing the record contents.

### 4.7 LRMH_HK (housekeeping)

**Purpose:** subscribe to `SEQ_STATUS`, `CTRL_STATUS`, `SENSE_TLM`, `FAULT_TLM` and `CMD_STATUS`, cache the fields the ground needs, and publish `LRMH_HkTlm_t` on the 1 Hz tick or on `LRMH_SEND_HK_MID`. It makes no HAL calls and has no ground command MID.

### 4.8 LRMH_FAULT (thin orchestrator)

**Purpose:** per ADR-0017 the threshold detection lives in LC tables and the recovery sequences in SC; this app owns only the per-fault tier table, the escalation counters, the multi-step cross-app dispatch, and the polled path for conditions LC cannot express.

**Command codes (`LRMH_FAULT_CMD_MID`):** 0 NOOP, 1 RESET (clears flags and counters), 2 ENABLE, 3 DISABLE, 10 LC_NOTIFY (`uint16 FaultFlag`; sent by the per-flag SC RTS), 11 PHASE_ADVANCE (`uint8 NewPhase`; sent by the sequencer).

**Fault flags and default tiers:**

| Bit | Flag | Tier | Detected by |
|-----|------|------|-------------|
| 0x0001 | STALL | T2 | polled path (not yet implemented) |
| 0x0002 | OVERCURRENT | T3 | LC WP1 |
| 0x0004 | FORCE_SATURATION | T2 | LC WP0 |
| 0x0008 | ENCODER_LOSS | T1 | polled path (not yet implemented) |
| 0x0010 | THERMAL | T3 | LC WP3 |
| 0x0020 | FORCE_RATE | T1 | LC WP6 |
| 0x0040 | HARD_STOP | T2 | LC WP2 |
| 0x0080 | SENSOR_TIMEOUT | T1 | polled path |
| 0x0100 | DEPTH_DISAGREE | T2 | polled path (not yet implemented) |
| 0x0200 | BUS_VOLTAGE | T3 | LC WP4 or WP5 |
| 0x0400 | HOST_NAV_TIMEOUT | T2 | polled path (not yet implemented) |

**Dispatch:** T1 sends RESUME to the sequencer and counts attempts per phase; more than `LRMH_T1_MAX_ATTEMPTS_PER_PHASE` (3) escalates to T2. T2 sends SKIP and marks the current site; `LRMH_T2_ESCALATION_THRESHOLD` (3) marked sites within the last `LRMH_T2_ESCALATION_WINDOW` (5) sites escalates to T3 (the window slides by one at each site end). T3 sends ABORT to the sequencer and SAFE to `lrmh_cmd`. A category already raised is coalesced (ADR-0013) until its recovery clears the bit: a T1 bit clears when the resume is dispatched (and LC is re-armed, so a repeat counts as a new attempt), every bit clears at site end, and a T3 bit clears on the token reset that SAFE entry broadcasts. A T1 raised once the measured phases are over (during the final retract) is noted and ignored. Outside TESTING every category dispatches as T3, since there is no test to recover (a force-saturation trip during a manual move in READY safes the instrument).

**Polled path (10 Hz):** `SENSOR_TIMEOUT` is raised when no sample telemetry has been received for `LRMH_SENSOR_TIMEOUT_MS` (200 ms), evaluated only while the mode is TESTING and the phase is between BASELINE_PEN and POST_PEN, because acquisition is intentionally off outside that window. The other state-watching categories have tier-table entries but no detector yet.

## 5. FDIR tables

### 5.1 LC watchpoints (`tables/lc_watchpoints.c`)

All seven evaluate fields of `LRMH_SampleTlm_t` on `0x08A3`:

| WP | Field | Operator | Threshold | Fault flag |
|----|-------|----------|-----------|------------|
| 0 | `force_n` | > | 600 N | FORCE_SATURATION |
| 1 | `current_a` | > | 4.5 A | OVERCURRENT |
| 2 | `stroke_m` | > | 0.33 m | HARD_STOP |
| 3 | `temp_c` | > | 55 C | THERMAL |
| 4 | `bus_v` | < | 23 V | BUS_VOLTAGE |
| 5 | `bus_v` | > | 33 V | BUS_VOLTAGE |
| 6 | `force_rate_nps` | > | 800 N/s | FORCE_RATE |

### 5.2 LC actionpoints (`tables/lc_actionpoints.c`)

One actionpoint per fault flag, all ACTIVE at boot (the install script changes LC's power-on default from DISABLED). Every watchpoint result goes STALE after 2 actionpoint sample cycles (0.2 s) without a new sample, so the last reading of a test cannot keep an actionpoint tripping after acquisition stops. LC sets an actionpoint PASSIVE after it fires, so `lrmh_fault` re-arms all actionpoints (LC `SET_AP_STATE`, all ACTIVE) after a T1 or T2 dispatch and on the token reset that INIT_INSTRUMENT, a new test or a restart broadcasts; not on SAFE entry, where the health watchpoint that drove the T3 is still fresh and would fire again. Until the operator's INIT, a category that has tripped into SAFE stays silent. Each actionpoint fires its own RTS because an SC RTS is static and cannot carry the tripped watchpoint's argument. BUS_VOLTAGE is the OR of WP4 and WP5. FORCE_RATE needs three consecutive failures (`MaxFailsBeforeRTS = 3`, 300 ms at 10 Hz) because one sample pair of 1.5 N noise over 10 ms is already 150 N/s. The event texts are `LRMH force saturation`, `LRMH overcurrent`, `LRMH stroke hard stop`, `LRMH thermal`, `LRMH bus voltage`, `LRMH force rate`, with event IDs 0xA001 to 0xA006.

### 5.3 SC relative time sequences (`tables/sc/`)

| RTS | Name | Contents |
|-----|------|----------|
| 1 | NOTIFY FORCE_SATURATION | one `LC_NOTIFY` command to `lrmh_fault` with flag 0x0004 |
| 2 | RESUME_PHASE (stub) | one `NOOP` to `lrmh_cmd` |
| 3 | SKIP_SITE (stub) | one `ABORT` to `lrmh_cmd` |
| 4 | RETRACT_AND_SAFE (stub) | one `SAFE` to `lrmh_cmd` |
| 5 to 9 | NOTIFY OVERCURRENT, HARD_STOP, THERMAL, BUS_VOLTAGE, FORCE_RATE | one `LC_NOTIFY` each |

The notify RTSes are the path that runs. RTS 2 to 4 are single-command stubs that no actionpoint references and nothing starts automatically; they are enabled with the rest so ground can start them, and they are not the multi-step sequences their names suggest. SC boots with every RTS disabled; the launcher enables 1 to 9.

### 5.4 HS application monitor (`tables/hs_table.c`)

Every LRMH app is monitored on a 30-cycle window (about 36 s at HS's 1.2 s cycle). `LRMH_CMD` has no action (losing the command surface requires ground); the other seven restart. A restarted `lrmh_seq` bumps the token epoch through its cold-start path, which is how the receivers know to discard commands from the previous instance.

### 5.5 DS tables (`tables/ds_filter_table.c`, `ds_file_table.c`)

| MID | File set | Path | Basename | Rotation |
|-----|----------|------|----------|----------|
| 0x08AA | 0 | `/cf/lrmh/data/` | `lrmh_site_` + time + `.bin` | 1 MB or 24 h |
| 0x08AB | 1 | `/cf/lrmh/reports/` | `lrmh_human_` + time + `.txt` | 512 KB or 24 h |

Every message is captured (count filter 1 of 1). On the native build `/cf/` maps to `build/exe/cpu1/cf/`. FM is installed with its upstream table so the files can be listed and fetched by normal FM commands.

## 6. Sequence token protocol (ADR-0007)

Every command the sequencer sends to `lrmh_ctrl`, `lrmh_sense`, `lrmh_proc` and `lrmh_report` carries a packed 32-bit token: `Epoch[16] | SeqState[8] | SeqCount[8]`. `lrmh_cmd` bumps its epoch and broadcasts `LRMH_TOKEN_RESET_MID` on INIT_INSTRUMENT, SAFE entry, cold start and every accepted START_SITE_TEST; `lrmh_seq` bumps the epoch it stamps on its own cold start and on every new test. Receivers re-baseline on the broadcast and on the first command of a newer epoch, so the two counters need not agree. Phase start times reach `lrmh_proc` through the sequencer's segment-start command, which also clears its sample buffer, so samples from before a measured segment are never part of it. The state byte is the current phase; the count is per-epoch and wraps at 256 with modular comparison.

Receivers validate in order: (1) the receiver's own safing commands are accepted unconditionally (a retract or e-stop at `lrmh_ctrl`, a STOP at `lrmh_sense`; each app decides this in its own command namespace); (2) the first command after a reset broadcast is accepted and re-baselines; (3) a stale epoch is rejected; (4) a stale state within the same epoch is rejected; (5) a count not newer than the accepted one within the same state is rejected; (6) otherwise accept and update. Rejections are evented and counted (`StaleCount` in the controller status). The TEST_COMPLETE notify to `lrmh_cmd` and the HK, wakeup and ground paths are not tokened.

## 7. HAL boundary

All hardware access goes through `hal/include/lrmh_hal.h`: `Init`, `ReadSensors`, `GetHealth`, `CommandPosition`, `CommandForce`, `Retract`, `Hold`, `EStop`, and the sim-only `StepSim`. The cFS image links `hal/client/lrmh_hal_client.c`, a UDP client to `127.0.0.1:17011` with a 100 ms per-request timeout, per-task sockets and sequence-numbered responses. The backend on the other end is `hal/sim/lrmh_hal_sim` (physics, complete), `hal/bench/lrmh_hal_bench` (template) or a future flight backend. Position and force loops, the safety clamps and the e-stop latch all live below this line. Contract detail is in the [HAL Porting Guide](LRMH_HAL_Porting_Guide.md); the sim physics is in the [Sim Model Spec](LRMH_Sim_Model_Spec.md).

## 8. Configuration

Everything is compile-time, in `cfs_defs/lrmh_platform_cfg.h` (installed as `lrmh_defs/lrmh_platform_cfg.h`). Each envelope has an LC margin below it. None of these is a measured property of real hardware.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `LRMH_MAX_FORCE_N` / `LRMH_LC_FORCE_THRESHOLD_N` | 650 / 600 | Force envelope (OP) and LC margin |
| `LRMH_MAX_FORCE_RATE_NPS` / `LRMH_LC_FORCE_RATE_THRESHOLD_NPS` | 1000 / 800 | Force rate |
| `LRMH_MAX_CURRENT_A` / `LRMH_LC_CURRENT_THRESHOLD_A` | 5.0 / 4.5 | Motor current (HW) |
| `LRMH_MAX_STROKE_M` / `LRMH_LC_STROKE_THRESHOLD_M` | 0.35 / 0.33 | Stroke (HW hard stop) |
| `LRMH_MAX_TEMP_C` / `LRMH_LC_TEMP_THRESHOLD_C` | 60 / 55 | Temperature (OP) |
| `LRMH_BUS_V_MIN` / `MAX` | 22 / 34 | Bus window (HW); LC at 23 and 33 |
| `LRMH_SENSOR_TIMEOUT_MS` | 200 | Polled sensor-timeout floor |
| `LRMH_HOST_NAV_MAX_AGE_MS` / `LRMH_HOST_NAV_TIMEOUT_S` | 500 / 5 | Host pose freshness for dispatch, and the TESTING fault threshold |
| `LRMH_ACTUATOR_TIMEOUT_S` | 30 | Deadline for retracts, contact and the self-test; penetrations get 120 s and load ramps ramp time plus 30 s |
| `LRMH_DEPTH_DISAGREE_THRESHOLD_M` | 0.005 | Depth versus stroke disagreement (detector not yet implemented) |
| `LRMH_T1_RETRACT_DISTANCE_M` / `LRMH_T1_RECOVERY_PAUSE_MS` | 0.005 / 200 | T1 fallback RTS parameters |
| `LRMH_T1_MAX_ATTEMPTS_PER_PHASE` | 3 | T1 to T2 escalation |
| `LRMH_T2_ESCALATION_WINDOW` / `THRESHOLD` | 5 / 3 | T2 to T3 escalation: sites with a T2 skip in the last 5 sites |
| `LRMH_PROC_LAYER_BREAK_D2F_THRESHOLD` | 1000 N/m^2 | Layer-break detector |
| `LRMH_PROC_BEARING_REF_N` | 500 | Bearing-proxy normalisation |
| `LRMH_PROC_REGRESSION_LOW/HIGH_TENTHS` | 2 / 8 | Stiffness regression window |
| `LRMH_MAX_SAMPLES_PER_PHASE` | 10000 | Raw buffer before STREAMING |
| `LRMH_RESERVOIR_SIZE` / `LRMH_PROVENANCE_FIRST_LAST_N` | 1000 / 500 | Streaming accumulator |
| `LRMH_HIST_DEPTH_BINS` / `LRMH_HIST_FORCE_BINS` | 32 / 32 | Streaming histogram |
| `LRMH_MAX_PHASES_PER_TEST` / `LRMH_MAX_SITES` | 8 / 256 | Record sizing |
| `LRMH_MAX_REPORT_TEXT_LEN` | 2048 | Human text buffer |
| Pipe depths | 16 (cmd, seq, ctrl, fault), 32 (sense), 48 (proc), 8 (report, hk) | SB pipes |
| `LRMH_MAIN_LOOP_RATE_HZ` / `LRMH_SENSE_RATE_HZ` / `LRMH_HK_PERIOD_S` | 10 / 100 / 1.0 | Rates |
| `LRMH_DATA_OUTPUT_DIR` / `LRMH_REPORT_OUTPUT_DIR` | `/cf/lrmh/data/`, `/cf/lrmh/reports/` | DS paths |
| `LRMH_DS_MAX_FILE_SIZE_BYTES` / `LRMH_DS_MAX_FILE_AGE_SEC` | 1 MB / 24 h | DS rotation |

## 9. Directory layout

```
lrmh-cfs/                          This repository
├── apps/
│   ├── lrmh_cmd/fsw/              inc/ (app, events, msg headers), src/, test/
│   ├── lrmh_seq/fsw/
│   ├── lrmh_ctrl/fsw/
│   ├── lrmh_sense/fsw/
│   ├── lrmh_proc/fsw/
│   ├── lrmh_report/fsw/
│   ├── lrmh_hk/fsw/
│   └── lrmh_fault/fsw/
├── msg/                           lrmh_msgids.h, lrmh_msg.h, lrmh_seq_token.h/.c
├── cfs_defs/                      lrmh_platform_cfg.h (envelopes, rates, sizes)
├── hal/
│   ├── include/lrmh_hal.h         The HAL contract
│   ├── client/                    UDP client linked into the cFS image
│   ├── sim/                       Physics-backed HAL server (lrmh_hal_sim)
│   └── bench/                     Bench backend template (TODO(bench) markers)
├── sim/                           regolith, actuator, sensor, fault-injection models; tools/
├── tables/                        LC, SC, HS, DS table sources
├── vendor/cfs-support-apps/       DS, FM, LC, SC, HS submodules (pinned)
├── cfe_stubs/                     Minimal cFE/HAL stubs for the standalone unit tests
├── ground/                        lrmh_ui.html, lrmh_bridge.py, lrmh_console.py, lrmh_ground_test.py
├── scripts/                       launch, boot check, smoke, fault, record and HAL test tools
├── docs/                          This documentation set
├── install.sh                     Install into a cFS tree
└── CMakeLists.txt                 Standalone host build (sim, HAL backends, unit tests)
```
