#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""
LRMH WebSocket-to-UDP Bridge
=============================

Bridges browser-based ground UIs to the cFS CI_LAB/TO_LAB interface.

  Browser (WebSocket)  ←→  Bridge  ←→  cFS (UDP CI_LAB :1234 / TO_LAB :2234)

Commands flow:   Browser → WS → Bridge → UDP → cFS CI_LAB
Telemetry flows: cFS TO_LAB → UDP → Bridge → WS → Browser

On each client connect the bridge sends a TO_LAB enable command so cFS
streams telemetry to this machine.

Two message schemas coexist on the wire (an older `type` key and the
`tlm` key `lrmh_ui.html` uses); nothing in tree consumes `type` now:

  * Legacy:  {"type": "hk"|"sensor"|"seqStatus"|"siteResult"|"fault"|...}
  * New UI:  {"tlm": "hk"|"sense"|"seq_status"|"site_result"|"fault"|...}
             {"event": "info"|"warn"|"error", "text": ...}
             {"ack": "<cmd>", "ok": true|false, ...}

Telemetry messages carry BOTH keys (`type` and `tlm`) in a single
packet; clients filter on whichever they prefer. Events and acks are
new-schema only — the legacy console ignores them via its switch
default.

Usage:
  python3 lrmh_bridge.py                         # legacy mode, WS on :8765
  python3 lrmh_bridge.py --serve-ui              # serves lrmh_ui.html on :8080
  python3 lrmh_bridge.py --serve-ui --ws-port 9000
"""

from __future__ import annotations

import argparse
import os
import asyncio
import json
import logging
import mimetypes
import socket
import struct
import sys
import time
from http import HTTPStatus
from pathlib import Path
from typing import Any, Dict, Optional, Set, Tuple

try:
    import websockets
    from websockets.asyncio.server import serve as ws_serve
    from websockets.http11 import Response
    from websockets.datastructures import Headers
except ImportError:
    print(
        "lrmh_bridge.py requires the `websockets` package, version 13 or newer.\n"
        "  Install with: pip install 'websockets>=13'\n"
        "  (Or: python3 -m pip install --user 'websockets>=13')\n",
        file=sys.stderr,
    )
    sys.exit(1)


log = logging.getLogger("lrmh_bridge")


# ──────────────────────────────────────────────────────────
# cFS Message IDs (must match lrmh_msgids.h)
# ──────────────────────────────────────────────────────────
LRMH_CMD_MID       = 0x18A0
LRMH_SEND_HK_MID   = 0x18A1
LRMH_SEQ_CMD_MID   = 0x18A2
LRMH_CTRL_CMD_MID  = 0x18A3
LRMH_SENSE_CMD_MID = 0x18A4
LRMH_FAULT_CMD_MID = 0x18A7

# TO_LAB
TO_LAB_CMD_MID     = 0x1880

# Telemetry MIDs
LRMH_HK_TLM_MID            = 0x08A0
LRMH_SEQ_STATUS_MID        = 0x08A1
LRMH_SENSE_TLM_MID         = 0x08A3
LRMH_PROC_TLM_MID          = 0x08A4
LRMH_FAULT_TLM_MID         = 0x08A6
LRMH_REPORT_SITE_RECORD_MID = 0x08AA  # binary site record (ADR-0016)
LRMH_MAX_PHASES_PER_TEST   = 8

# Command Codes (see apps/lrmh_cmd/fsw/inc/lrmh_cmd_msg.h)
CC: Dict[str, int] = {
    "NOOP": 0, "RESET_COUNTERS": 1, "INIT_INSTRUMENT": 2, "DEPLOY": 3,
    "STOW": 4, "ABORT": 5, "START_SITE_TEST": 6, "REQUEST_REPORT": 7, "SET_MODE": 8,
    "RESET_FAULT": 9, "SAFE": 10, "POWER_OFF": 11,
    # Manual actuator commands route through lrmh_seq (LRMH_SEQ_CMD_MID,
    # FcnCode = LRMH_SEQ_MANUAL_CTRL_CC) so the sequencer can stamp the
    # sequence token. ADR-0007 keeps tokens FSW-internal — the bridge
    # never fabricates one. lrmh_seq accepts manual ctrl only when
    # phase == IDLE; commands during an active test get rejected with
    # an EVS error.
    "SEQ_MANUAL_CTRL": 30,
    "SENSE_START": 2, "SENSE_STOP": 3, "SENSE_ZERO_FORCE": 4, "SENSE_ZERO_DEPTH": 5,
    "FAULT_RESET": 1, "FAULT_ENABLE": 2, "FAULT_DISABLE": 3,
}

# CTRL Mode bytes — must match LRMH_CTRL_MODE_* in lrmh_ctrl_msg.h.
CTRL_MODE = {
    "MOVE":     0,   # position-mode drive
    "FORCE":    1,   # force-mode drive
    "RETRACT":  2,   # retract to home
    "HOLD":     3,   # hold current position
    "ESTOP":    4,   # emergency stop the actuator
    "PRECHECK": 5,   # HAL self-test sweep
}

# HAL transport — used only for HAL_REQ_INJECT_FAULT (sim-only, lets
# the engineer dashboard exercise FDIR without shelling out to
# scripts/hal_bench_test.py). Bench/flight backends ack the request as
# a no-op so this is safe to wire even when the sim isn't running.
# Envelope must match hal/sim/lrmh_hal_sim.c byte-for-byte.
HAL_MAGIC      = 0x4C524D48      # 'LRMH'
HAL_DEFAULT_PORT = 17011
HAL_REQ_INJECT_FAULT = 9
HAL_REQ_SET_SITE = 10          # draw and load a regolith site: (preset, seed)
HAL_REQ_ACTUATOR_FAULT = 11    # a fault in the actuator physics: (kind, active)
REGOLITH_PRESETS = {           # sim/regolith_model.h RegolithPreset_t
    "lunar_highland": 0, "lunar_mare": 1, "lunar_loose": 2,
    "earth_dry_sand": 3, "earth_sandy_loam": 4, "earth_firm_clay": 5,
    "bench_simulant": 6,
}
ACTUATOR_FAULT_KINDS = {"stall": 0, "overcurrent": 1, "encoder_loss": 2}
HAL_INJECT_KINDS: Dict[str, int] = {
    "none":        0, "force-sat":   1, "overcurrent": 2,
    "hard-stop":   3, "thermal":     4, "bus-voltage": 5,
    "force-rate":  6,
}

MODES: Dict[int, str] = {
    0: "OFF", 1: "STANDBY", 2: "READY", 3: "TESTING", 4: "SAFE", 5: "FAULT",
}
MODE_READY   = 2
MODE_TESTING = 3
MODE_SAFE    = 4
MODE_FAULT   = 5

SEQ_STATES: Dict[int, str] = {
    0: "IDLE", 1: "PRECHECK", 2: "POSITIONING", 3: "BASELINE_PEN",
    4: "LOAD_RAMP", 5: "REPEAT_LOAD", 6: "POST_PEN", 7: "TRACTION",
    8: "PROCESSING", 9: "REPORTING",
}

QUALITY: Dict[int, str] = {
    0: "VALID", 1: "PARTIAL", 2: "FAULT",   # LRMH_Quality_t in msg/lrmh_msg.h
}

# Commands accepted over WebSocket. Both lowercase (new UI) and
# uppercase (legacy console) keys map to the same internal handler.
COMMAND_ALIASES: Dict[str, str] = {
    # New UI lowercase
    "noop": "NOOP", "reset_counters": "RESET_COUNTERS",
    "init": "INIT_INSTRUMENT", "deploy": "DEPLOY", "stow": "STOW",
    "abort": "ABORT", "request_report": "REQUEST_REPORT",
    "set_mode": "SET_MODE", "power_off": "POWER_OFF",
    "reset_fault": "RESET_FAULT", "safe": "SAFE",
    "ensure_ready": "ENSURE_READY", "start_test": "START_TEST",
    "ctrl_move": "CTRL_MOVE", "ctrl_force": "CTRL_FORCE",
    "ctrl_retract": "CTRL_RETRACT", "ctrl_hold": "CTRL_HOLD",
    "ctrl_estop": "CTRL_ESTOP", "ctrl_clear_estop": "CTRL_CLEAR_ESTOP",
    "sense_start": "SENSE_START", "sense_stop": "SENSE_STOP",
    "sense_zero_force": "SENSE_ZERO_FORCE", "sense_zero_depth": "SENSE_ZERO_DEPTH",
    "fault_reset": "FAULT_RESET", "fault_enable": "FAULT_ENABLE",
    "fault_disable": "FAULT_DISABLE",
    "inject_fault": "INJECT_FAULT",
    "set_site": "SET_SITE",
    "actuator_fault": "ACTUATOR_FAULT",
    "presets_load": "PRESETS_LOAD",
    "presets_save": "PRESETS_SAVE",
    "presets_delete": "PRESETS_DELETE",
    "history_load": "HISTORY_LOAD",
    "raw_cmd": "RAW_CMD",
    "fs_list": "FS_LIST",
    "fs_fetch": "FS_FETCH",
    # Legacy uppercase passes through identity
    "NOOP": "NOOP", "RESET_COUNTERS": "RESET_COUNTERS",
    "INIT_INSTRUMENT": "INIT_INSTRUMENT", "DEPLOY": "DEPLOY", "STOW": "STOW",
    "ABORT": "ABORT", "REQUEST_REPORT": "REQUEST_REPORT",
    "SET_MODE": "SET_MODE", "START_SITE_TEST": "START_SITE_TEST",
    "POWER_OFF": "POWER_OFF", "RESET_FAULT": "RESET_FAULT", "SAFE": "SAFE",
    "CTRL_MOVE": "CTRL_MOVE", "CTRL_FORCE": "CTRL_FORCE",
    "CTRL_RETRACT": "CTRL_RETRACT", "CTRL_HOLD": "CTRL_HOLD",
    "CTRL_ESTOP": "CTRL_ESTOP", "CTRL_CLEAR_ESTOP": "CTRL_CLEAR_ESTOP",
    "SENSE_START": "SENSE_START", "SENSE_STOP": "SENSE_STOP",
    "SENSE_ZERO_FORCE": "SENSE_ZERO_FORCE", "SENSE_ZERO_DEPTH": "SENSE_ZERO_DEPTH",
    "FAULT_RESET": "FAULT_RESET", "FAULT_ENABLE": "FAULT_ENABLE",
    "FAULT_DISABLE": "FAULT_DISABLE",
}


# ──────────────────────────────────────────────────────────
# CCSDS Packet Construction (Draco format)
# CMD: 8 bytes header (6 primary + 2 secondary: FC + checksum)
# ──────────────────────────────────────────────────────────
_cmd_seq = 0


def build_ccsds_cmd(mid: int, fc: int, payload: bytes = b"") -> bytes:
    """Build a CCSDS v1 command packet for CI_LAB (Draco format)."""
    global _cmd_seq
    _cmd_seq = (_cmd_seq + 1) & 0x3FFF
    total = 8 + len(payload)
    length = total - 7  # CCSDS length field convention
    hdr = struct.pack(">HHH", mid, 0xC000 | _cmd_seq, length)
    sec = struct.pack("BB", fc, 0)
    return hdr + sec + payload


def build_to_lab_enable(ip_str: str = "127.0.0.1") -> bytes:
    """TO_LAB enable: CC=6, payload is the destination IP as a plain
    NUL-padded 16-byte C string. (cmdUtil's ``--string="16:IP"`` syntax is
    a cmdUtil length prefix, not part of the packet; sending the prefix
    makes TO_LAB parse an invalid address and telemetry never flows.)"""
    payload = ip_str.encode("ascii").ljust(16, b"\x00")
    return build_ccsds_cmd(TO_LAB_CMD_MID, 6, payload)


_site_test_seq = 0


def build_start_payload(params: Dict[str, Any]) -> bytes:
    """Build LRMH_StartSiteTestCmd_t (36-byte body).

    DispatchFlags defaults to LRMH_DISPATCH_WITNESS_TARGET (0x08) so
    the bridge works against the standalone simulator where no rover
    is publishing LRMH_HOST_POSE_MID. In a real deployment, ground
    should omit this bit (or pass dispatchFlags=0) so the FSW enforces
    the host-nav precondition.
    """
    global _site_test_seq
    _site_test_seq = (_site_test_seq + 1) & 0xFFFF
    site     = int(params.get("siteId", 17))
    profile  = int(params.get("profileId", 1))
    tool     = int(params.get("toolId", 1))
    depth    = float(params.get("maxDepth", 0.30))
    load     = float(params.get("maxLoad", 500.0))
    repeat   = int(params.get("enableRepeat", 1))
    post     = int(params.get("enablePost", 1))
    traction = int(params.get("enableTraction", 0))
    dispatch = int(params.get("dispatchFlags", 0x08))  # WITNESS_TARGET
    session  = int(params.get("sessionId", 1))
    seqnum   = int(params.get("sequenceNumber", _site_test_seq))
    paired   = int(params.get("pairedSiteId", 0))
    offx     = float(params.get("microSiteOffsetX_mm", 0.0))
    offy     = float(params.get("microSiteOffsetY_mm", 0.0))
    rptcnt   = int(params.get("repeatLoadCount", 0))
    return struct.pack(
        "<HBBffBBBBHHHHffB3x",
        site, profile, tool,
        depth, load,
        repeat, post, traction,
        dispatch,
        session, seqnum, paired, 0,
        offx, offy,
        rptcnt,
    )


def build_manual_ctrl_payload(
    mode_byte: int,
    pos: float = 0.0,
    force: float = 0.0,
    rate: float = 0.0,
    timeout: float = 60.0,
) -> bytes:
    """Build the LRMH_SeqManualCtrlCmd_t body. Layout matches
    apps/lrmh_seq/fsw/inc/lrmh_seq_msg.h: u8 Mode, u8[3] Reserved,
    f32 TargetPosition_m, f32 TargetForce_N, f32 RateLimit,
    f32 Timeout_s."""
    return struct.pack("<BBBBffff", mode_byte, 0, 0, 0, pos, force, rate, timeout)


# ──────────────────────────────────────────────────────────
# CCSDS Telemetry Decoding
# Draco TLM header: 16 bytes total
#   6  primary header (StreamID + Sequence + Length)
#   6  secondary header (time)
#   4  padding to the 8-byte alignment of the first payload field
# ──────────────────────────────────────────────────────────
TLM_HDR_LEN = 16


def decode_tlm_header(data: bytes) -> Optional[Tuple[int, int, bytes]]:
    if len(data) < TLM_HDR_LEN:
        return None
    stream_id, sequence, _length = struct.unpack(">HHH", data[0:6])
    seq_count = sequence & 0x3FFF
    return stream_id, seq_count, data[TLM_HDR_LEN:]


def parse_hk_tlm(payload: bytes) -> Optional[Dict[str, Any]]:
    """Parse LRMH_HkTlm_t payload, 88 bytes.

    See apps/lrmh_hk/fsw/inc/lrmh_hk_msg.h. Layout (post-CCSDS header):
      0:  u8  InstrumentMode, u8 ActiveTool, u16 _
      4:  u8  SeqPhase, u8 SeqQuality, u8 SeqWaitingForCtrl, u8 T1Attempts
      8:  u16 CurrentSiteId, u16 CurrentSessionId, u16 Epoch, u16 _
     16:  u8  CtrlActiveMode, u8 CtrlCommandPending, u8 HalState, u8 _
     20:  u16 HalFlags, u16 CtrlStaleCount
     24:  f32 StrokePosition_m, f32 LastCommandedForce_N
     32:  u16 SensorChannelValid, u16 _
     36:  f32 Force_N, f32 Depth_m, f32 MotorCurrent_A,
          f32 BusVoltage_V, f32 ActuatorTemp_C, f32 ForceRate_NpS
     60:  u16 FaultFlags, u8 FaultActive, u8 RecoveryTier
     64:  u32 TimeoutCount, u32 T2EventsInWindow
     72:  u32 TotalCmdCount, u32 TotalErrCount,
          u32 TotalRejectedCount, u32 HkCycleCount
    """
    if len(payload) < 88:
        return None
    try:
        mode, active_tool, _r0 = struct.unpack_from("<BBH", payload, 0)
        (seq_phase, seq_quality, seq_waiting,
         t1_attempts) = struct.unpack_from("BBBB", payload, 4)
        site_id, session_id, epoch, _r1 = struct.unpack_from("<HHHH", payload, 8)
        ctrl_mode, ctrl_pending, hal_state, _r2 = struct.unpack_from("BBBB", payload, 16)
        hal_flags, ctrl_stale = struct.unpack_from("<HH", payload, 20)
        stroke_m, last_cmd_force = struct.unpack_from("<ff", payload, 24)
        ch_valid, _r3 = struct.unpack_from("<HH", payload, 32)
        (force_n, depth_m, motor_a, bus_v,
         temp_c, force_rate) = struct.unpack_from("<ffffff", payload, 36)
        fault_flags, fault_active, recovery_tier = struct.unpack_from("<HBB", payload, 60)
        timeout_count, t2_in_window = struct.unpack_from("<II", payload, 64)
        total_cmd, total_err, total_rejected, hk_cycle = struct.unpack_from("<IIII", payload, 72)
        return {
            # Legacy + new schema both populated.
            "type": "hk", "tlm": "hk",
            "mode": mode, "modeName": MODES.get(mode, str(mode)),
            "activeTool": active_tool,
            "seqState": seq_phase,
            "seqStateName": SEQ_STATES.get(seq_phase, str(seq_phase)),
            "seqQuality": seq_quality,
            "seqWaitingForCtrl": seq_waiting,
            "siteId": site_id, "sessionId": session_id, "epoch": epoch,
            "ctrlMode": ctrl_mode, "ctrlCommandPending": ctrl_pending,
            "halState": hal_state, "halFlags": hal_flags,
            "ctrlStaleCount": ctrl_stale,
            "strokePosition": round(stroke_m, 4),
            "lastCommandedForce": round(last_cmd_force, 2),
            "sensorChannelValid": ch_valid,
            "currentForce": round(force_n, 2),
            "currentDepth": round(depth_m, 4),
            "motorCurrent":  round(motor_a, 3),
            "busVoltage":    round(bus_v, 2),
            "temperature":   round(temp_c, 1),
            "forceRate":     round(force_rate, 2),
            "faultFlags":  fault_flags,
            "faultActive": fault_active,
            "recoveryTier": recovery_tier,
            "timeoutCount": timeout_count,
            "t2EventsInWindow": t2_in_window,
            "totalCmdCount": total_cmd,
            "totalErrCount": total_err,
            "totalRejectedCount": total_rejected,
            "hkCycleCount": hk_cycle,
            # Aliases some older clients expect
            "cmdCount": total_cmd, "cmdErrCount": total_err,
            "errorFlags": fault_flags,
            "faultState": fault_active,
        }
    except struct.error:
        return None


def parse_sensor_tlm(payload: bytes) -> Optional[Dict[str, Any]]:
    """Parse LRMH_SampleTlm_t payload — 2.0 layout. Core fields only;
    host pose/vel alignment past offset 44 is parsed in tlm_kinds that
    need it.

    Layout (post-CCSDS header):
      0:  u64 sample_time_us
      8:  u32 sample_index
     12:  u16 channel_valid, u8 phase, u8 _
     16:  f32 force_n
     20:  f32 depth_m
     24:  f32 stroke_m
     28:  f32 current_a
     32:  f32 bus_v
     36:  f32 temp_c
     40:  f32 force_rate_nps
    """
    if len(payload) < 44:
        return None
    try:
        sample_time_us = struct.unpack_from("<Q", payload, 0)[0]
        sample_idx = struct.unpack_from("<I", payload, 8)[0]
        channel_valid, phase, _r = struct.unpack_from("<HBB", payload, 12)
        force_n, depth_m, stroke_m, current_a = struct.unpack_from("<ffff", payload, 16)
        bus_v, temp_c, force_rate = struct.unpack_from("<fff", payload, 32)
        mission_time_s = sample_time_us / 1e6
        return {
            # Legacy shape
            "type": "sensor",
            "sampleIndex": sample_idx,
            "channelValid": channel_valid,
            "testPhase": phase,
            "missionTime": round(mission_time_s, 6),
            "force": round(force_n, 2),
            "penetrationDepth": round(depth_m, 4),
            "strokePosition": round(stroke_m, 4),
            "motorCurrent": round(current_a, 3),
            "busVoltage": round(bus_v, 2),
            "temperature": round(temp_c, 1),
            "forceRate": round(force_rate, 2),
            # New-schema aliases (UI spec: tlm=sense, depth, current, temp, t)
            "tlm": "sense",
            "depth": round(depth_m, 4),
            "current": round(current_a, 3),
            "temp": round(temp_c, 1),
            "t": round(mission_time_s, 3),
        }
    except struct.error:
        return None


def parse_proc_tlm(payload: bytes) -> Optional[Dict[str, Any]]:
    """Parse LRMH_ProcResultTlm_t, 44 bytes after header.
      0:  u16 SiteId, u16 _
      4:  f32 max_force_n, f32 max_sinkage_m, f32 stiffness_proxy_npm,
          f32 bearing_proxy, f32 layer_break_depth_m, f32 compaction_delta,
          f32 variability_index, f32 duration_s
     36:  u32 sample_count
     40:  u8 phase, u8 quality, u8 storage_mode, u8 _
    """
    if len(payload) < 44:
        return None
    try:
        site_id, _r0 = struct.unpack_from("<HH", payload, 0)
        (max_f, max_sink, stiffness, bearing, layer_break,
         compaction, variability, duration) = struct.unpack_from("<8f", payload, 4)
        sample_count = struct.unpack_from("<I", payload, 36)[0]
        phase, quality, storage_mode, _r1 = struct.unpack_from("BBBB", payload, 40)
        return {
            "type": "result",
            "tlm": "proc",
            "siteId": site_id, "testPhase": phase,
            "quality": quality, "qualityName": QUALITY.get(quality, str(quality)),
            "storageMode": storage_mode,
            "sampleCount": sample_count,
            "maxForce": round(max_f, 1),
            "maxSinkage": round(max_sink, 4),
            "stiffnessProxy": round(stiffness, 1),
            "bearingProxy": round(bearing, 3),
            "layerBreakDepth": round(layer_break, 4),
            "compactionDelta": round(compaction, 4),
            "variability": round(variability, 4),
            "phaseDuration_s": round(duration, 3),
        }
    except struct.error:
        return None


def parse_site_record(payload: bytes) -> Optional[Dict[str, Any]]:
    """Parse LRMH_SiteResultRecord_t (LRMH_REPORT_SITE_RECORD_MID,
    0x08AA) — the canonical 2.0 site result. Big struct, ~310 bytes.
    See apps/lrmh_report/fsw/inc/lrmh_report_msg.h and
    scripts/lrmh_inspect_record.py for the matching ground decoder."""
    # Fixed-section size up to fault state:
    #   16 (id) + 16 (timing) + 24 (host pose) + 8*32 (phases)
    # + 4 (count+pad) + 28 (agg) + 4 (fault) = 348 bytes
    if len(payload) < 348:
        return None
    try:
        off = 0
        # Versioning + identification (16)
        (version, _r0, session_id, seq_num, site_id, paired_site,
         profile_id, tool_id, outcome,
         storage_mode) = struct.unpack_from("<B3sHHHHBBBB", payload, off)
        off += 16
        # Timing (16)
        start_us, end_us = struct.unpack_from("<QQ", payload, off)
        off += 16
        # Host pose (24)
        pose = struct.unpack_from("<6f", payload, off)
        off += 24
        # Per-phase summaries (8 × 32 = 256)
        phases = []
        for _ in range(LRMH_MAX_PHASES_PER_TEST):
            (p_phase, p_quality, _pres, p_samples,
             p_max_force, p_max_sink, p_stiffness, p_bearing,
             p_layer_break, p_duration) = struct.unpack_from(
                "<BB2sIffffff", payload, off)
            off += 32
            phases.append({
                "phase":         p_phase,
                "phaseName":     SEQ_STATES.get(p_phase, str(p_phase)),
                "quality":       p_quality,
                "qualityName":   QUALITY.get(p_quality, str(p_quality)),
                "samples":       p_samples,
                "maxForce_N":    round(p_max_force, 2),
                "maxSinkage_m":  round(p_max_sink, 4),
                "stiffness":     round(p_stiffness, 2),
                "bearing":       round(p_bearing, 3),
                "layerBreak_m":  round(p_layer_break, 4),
                "duration_s":    round(p_duration, 3),
            })
        # Phase count + reserved (4)
        phase_count, _r1 = struct.unpack_from("<B3s", payload, off)
        off += 4
        # Aggregated derived science (28)
        (pen_mean, pen_peak, stiff_kn_m, bearing_kpa,
         layer_break_m, compaction, variability) = struct.unpack_from(
            "<7f", payload, off)
        off += 28
        # Fault state (4)
        fault_flags, recovery_tier, _r2 = struct.unpack_from("<HBB", payload, off)
        off += 4

        # Roll up "max force" from per-phase summaries (UI Done card).
        max_force_n = max((p["maxForce_N"] for p in phases), default=0.0)
        max_sink_m  = max((p["maxSinkage_m"] for p in phases), default=0.0)

        return {
            "type": "siteResult",
            "tlm": "site_result",
            "source": "v2_record_0x08AA",
            "version": version,
            "sessionId": session_id,
            "sequenceNumber": seq_num,
            "siteId": site_id,
            "pairedSiteId": paired_site,
            "profileId": profile_id,
            "toolId": tool_id,
            "quality": outcome,
            "qualityName": QUALITY.get(outcome, str(outcome)),
            "storageMode": storage_mode,
            "duration_us": end_us - start_us,
            "duration_s": round((end_us - start_us) / 1e6, 3),
            "hostPose": {
                "x_m": round(pose[0], 3), "y_m": round(pose[1], 3),
                "z_m": round(pose[2], 3),
                "roll_rad": round(pose[3], 3),
                "pitch_rad": round(pose[4], 3),
                "yaw_rad": round(pose[5], 3),
            },
            "phases": phases,
            "phaseCount": phase_count,
            # Aggregated derived science (canonical names)
            "penResistanceMean_kPa":  round(pen_mean, 2),
            "penResistancePeak_kPa":  round(pen_peak, 2),
            "stiffness_kN_m":         round(stiff_kn_m, 2),
            "bearing_kPa":            round(bearing_kpa, 2),
            "layerBreakDepth_m":      round(layer_break_m, 4),
            "compactionDelta":        round(compaction, 4),
            "variability":            round(variability, 4),
            "faultFlags":   fault_flags,
            "recoveryTier": recovery_tier,
            # UI-friendly aliases for the Done screen card builder:
            "maxForce":             round(max_force_n, 2),
            "maxSinkage_mm":        round(max_sink_m * 1000, 1),
            "layerBreakDepth_cm":   round(layer_break_m * 100, 1),
            "stiffnessProxy":       round(stiff_kn_m, 2),
            "bearingProxy":         round(bearing_kpa, 2),
        }
    except struct.error as exc:
        log.warning("site_record parse fail: %s", exc)
        return None


def parse_seq_status(payload: bytes) -> Optional[Dict[str, Any]]:
    """Parse LRMH_SeqStatusTlm_t, 52 bytes after header.
      0:  u8 Phase, u8 PreviousPhase, u8 QualityFlag, u8 WaitingForCtrl
      4:  u16 SiteId, u16 SessionId, u16 SequenceNumber, u16 Epoch
     12:  u32 TotalSamples, u32 CmdCount, u32 ErrCount,
          u32 T1AttemptsCurrentPhase, u32 RepeatCyclesDone, u32 RepeatCyclesTarget
     36:  f32 CurrentDepth_m, f32 CurrentForce_N,
          f32 ElapsedTime_s, f32 PhaseStartTime_s
    """
    if len(payload) < 52:
        return None
    try:
        phase, prev, quality, waiting_ctrl = struct.unpack_from("BBBB", payload, 0)
        site_id, session_id, seq_num, epoch = struct.unpack_from("<HHHH", payload, 4)
        (total_samples, cmd_count, err_count,
         t1_attempts, repeat_done, repeat_target) = struct.unpack_from("<6I", payload, 12)
        cur_depth, cur_force, elapsed, phase_start = struct.unpack_from("<4f", payload, 36)
        return {
            "type": "seqStatus",
            "tlm": "seq_status",
            "seqState": phase,
            "seqStateName": SEQ_STATES.get(phase, str(phase)),
            "prevState": prev,
            "prevStateName": SEQ_STATES.get(prev, str(prev)),
            "qualityFlag": quality,
            "waitingForCtrl": waiting_ctrl,
            "siteId": site_id,
            "sessionId": session_id,
            "sequenceNumber": seq_num,
            "epoch": epoch,
            "sampleCount": total_samples,
            "cmdCount": cmd_count, "errCount": err_count,
            "t1AttemptsCurrentPhase": t1_attempts,
            "repeatCyclesDone": repeat_done,
            "repeatCyclesTarget": repeat_target,
            "currentDepth_m": round(cur_depth, 4),
            "currentForce_N": round(cur_force, 2),
            "elapsedTime_s": round(elapsed, 3),
            "phaseStartTime_s": round(phase_start, 3),
        }
    except struct.error:
        return None


def parse_fault_tlm(payload: bytes) -> Optional[Dict[str, Any]]:
    """Parse LRMH_FaultTlm_t, 32 bytes.
      0:  u16 FaultFlags
      2:  u8 FaultActive, u8 RecoveryTier,
          u8 T1AttemptsCurrentPhase, u8 T2EventsInWindow,
          u8 FaultEnabled, u8 _
      8:  u32 TimeoutCount,
     12:  u32 TotalT1Events, u32 TotalT2Events, u32 TotalT3Events,
     24:  u32 CmdCount, u32 ErrCount
    """
    if len(payload) < 32:
        return None
    try:
        flags = struct.unpack_from("<H", payload, 0)[0]
        (active, tier, t1_cur_phase, t2_in_window,
         enabled, _r) = struct.unpack_from("BBBBBB", payload, 2)
        timeout_count = struct.unpack_from("<I", payload, 8)[0]
        t1_total, t2_total, t3_total = struct.unpack_from("<III", payload, 12)
        cmd_count, err_count = struct.unpack_from("<II", payload, 24)
        return {
            "type": "fault",
            "tlm": "fault",
            "errorFlags": flags,           # legacy alias
            "faultFlags": flags,
            "faultActive": active,
            "recoveryTier": tier,
            "t1AttemptsCurrentPhase": t1_cur_phase,
            "t2EventsInWindow": t2_in_window,
            "faultEnabled": enabled,
            "timeoutCount": timeout_count,
            "totalT1Events": t1_total,
            "totalT2Events": t2_total,
            "totalT3Events": t3_total,
            "cmdCount": cmd_count, "errCount": err_count,
        }
    except struct.error:
        return None


TLM_PARSERS = {
    LRMH_HK_TLM_MID:             parse_hk_tlm,
    LRMH_SEQ_STATUS_MID:         parse_seq_status,
    LRMH_SENSE_TLM_MID:          parse_sensor_tlm,
    LRMH_PROC_TLM_MID:           parse_proc_tlm,
    LRMH_FAULT_TLM_MID:          parse_fault_tlm,
    LRMH_REPORT_SITE_RECORD_MID: parse_site_record,
}


# ──────────────────────────────────────────────────────────
# Test-type mapping (UI → FSW parameters)
# ──────────────────────────────────────────────────────────
# Maps the UI's `test_type` (cpt/plate/full) to the FSW's TestProfileId
# + tool + enable-flag set. `plate_only` is not yet defined in the FSW
# sequence table; for now we fall back to `full` and log a warning.
TEST_TYPE_MAP: Dict[str, Dict[str, Any]] = {
    "cpt":   {"profile": 1, "tool": 1, "repeat": 0, "post": 0},
    "plate": {"profile": 1, "tool": 2, "repeat": 1, "post": 1, "fallback_from": "plate_only"},
    "full":  {"profile": 1, "tool": 1, "repeat": 1, "post": 1},
}


# ──────────────────────────────────────────────────────────
# Bridge Server
# ──────────────────────────────────────────────────────────
class LRMHBridge:
    def __init__(
        self,
        cfs_host: str,
        ci_port: int,
        tlm_port: int,
        ws_port: int,
        serve_ui: bool = False,
        ui_static_dir: Optional[Path] = None,
        hal_host: str = "127.0.0.1",
        hal_port: int = HAL_DEFAULT_PORT,
        cf_dir: Optional[Path] = None,
        ws_host: str = "127.0.0.1",
        allow_raw: bool = False,
    ) -> None:
        self.ws_host = ws_host
        self.allow_raw = allow_raw
        self.cfs_host = cfs_host
        self.ci_port = ci_port
        self.tlm_port = tlm_port
        self.ws_port = ws_port
        self.serve_ui = serve_ui
        self.ui_static_dir = ui_static_dir or Path(__file__).parent
        self.hal_host = hal_host
        self.hal_port = hal_port
        self.hal_seq = 0
        # Bridge-side state store. localStorage was rejected because
        # the dashboard SPA is a sim/dev tool — production ground will
        # likely be a different UI, and presets / site history should
        # live as portable JSON files rather than browser-bound state.
        self.state_dir = (ui_static_dir or Path(__file__).parent) / "state"
        self.state_dir.mkdir(parents=True, exist_ok=True)
        self.presets_path = self.state_dir / "presets.json"
        self.history_path = self.state_dir / "site_history.json"
        # cFS file storage root for fs_list / fs_fetch: the image's
        # cf/lrmh/ directory, where DS writes data + reports. Default is
        # derived from LRMH_CFS_ROOT (or ~/lrmh-cfs-build); override
        # with --cf-dir.
        if cf_dir is None:
            cfs_root = Path(os.environ.get("LRMH_CFS_ROOT",
                                           Path.home() / "lrmh-cfs-build"))
            cf_dir = cfs_root / "build/exe/cpu1/cf/lrmh"
        self.cf_dir = cf_dir.resolve() if cf_dir.exists() else cf_dir
        self.clients: Set[Any] = set()
        self.cmd_sock: Optional[socket.socket] = None
        self.tlm_sock: Optional[socket.socket] = None
        self.running = False
        self.tlm_count = 0
        self.unknown_mids: Set[int] = set()
        # Latest parsed HK packet. Used by ensure_ready + state-aware
        # commands so they don't race incoming telemetry.
        self.last_hk: Optional[Dict[str, Any]] = None

    # ── lifecycle ─────────────────────────────────────────
    async def start(self) -> None:
        self.running = True

        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.tlm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tlm_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Bump UDP receive buffer from the Linux default (~208 KB) to
        # 8 MB. During a test run the cFS image emits a burst of
        # status TLM (sense at 100 Hz, fault + seq_status + ctrl
        # status combined ~30+ Hz, cmd_status per command), and at
        # phase transitions lrmh_proc / lrmh_report fire in tight
        # succession. The asyncio reader doesn't always drain in time
        # — particularly when the event loop is busy with a fs_fetch
        # base64-encode or a multi-client broadcast — and once the
        # default 208 KB SO_RCVBUF fills, the kernel silently drops
        # arriving datagrams. The single-shot per-test packets
        # (0x08AA REPORT_SITE_RECORD, 0x08AB REPORT_SITE_HUMAN) are
        # exactly the ones most likely to land at the wrong moment
        # and get dropped. 8 MB absorbs ~30 seconds of worst-case
        # rate without overflow; if the kernel rejects this size
        # (it won't on a default Linux config) the bridge keeps
        # working at the OS-default size.
        try:
            self.tlm_sock.setsockopt(
                socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
            actual = self.tlm_sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
            log.info("tlm SO_RCVBUF requested 8 MiB, got %d bytes", actual)
        except OSError as exc:
            log.warning("could not bump SO_RCVBUF (%s) — running at OS default", exc)
        self.tlm_sock.bind(("0.0.0.0", self.tlm_port))
        self.tlm_sock.setblocking(False)

        log.info("commands  → %s:%d (CI_LAB)", self.cfs_host, self.ci_port)
        log.info("telemetry ← 0.0.0.0:%d (TO_LAB)", self.tlm_port)
        if self.serve_ui:
            log.info("HTTP+WS   → %s:%d (--serve-ui from %s)",
                     self.ws_host, self.ws_port, self.ui_static_dir)
        else:
            log.info("WebSocket → %s:%d", self.ws_host, self.ws_port)
        if self.ws_host not in ("127.0.0.1", "localhost", "::1"):
            log.warning("WebSocket is reachable from other hosts and has no "
                        "authentication; use only on a trusted network")

        self._send_to_lab_enable()
        self._relay_task = asyncio.create_task(self.tlm_relay())

        process_request = self._serve_static_request if self.serve_ui else None
        async with ws_serve(
            self.ws_handler,
            self.ws_host,
            self.ws_port,
            process_request=process_request,
        ):
            log.info("bridge ready")
            await asyncio.Future()

    # ── HTTP static file serving (when --serve-ui) ────────
    def _serve_static_request(
        self, connection: Any, request: Any,
    ) -> Optional[Response]:
        """websockets process_request hook. Return None to allow the WS
        upgrade to proceed; return a Response to handle this request as
        a plain HTTP GET."""
        # If this is a WebSocket upgrade, let it through.
        upgrade = request.headers.get("Upgrade", "")
        if upgrade and "websocket" in upgrade.lower():
            return None

        path = request.path or "/"
        # Serve lrmh_ui.html on "/". Everything else is a file lookup.
        if path in ("/", ""):
            target = self.ui_static_dir / "lrmh_ui.html"
        else:
            rel = path.lstrip("/").split("?", 1)[0].split("#", 1)[0]
            if ".." in Path(rel).parts:
                return self._http_response(HTTPStatus.FORBIDDEN, b"forbidden\n")
            target = self.ui_static_dir / rel

        if not target.is_file():
            return self._http_response(HTTPStatus.NOT_FOUND, b"not found\n")

        try:
            body = target.read_bytes()
        except OSError as exc:
            log.error("static read failed for %s: %s", target, exc)
            return self._http_response(
                HTTPStatus.INTERNAL_SERVER_ERROR, b"read error\n"
            )

        ctype, _ = mimetypes.guess_type(str(target))
        if ctype is None:
            ctype = "application/octet-stream"
        return self._http_response(HTTPStatus.OK, body, content_type=ctype)

    @staticmethod
    def _http_response(
        status: HTTPStatus, body: bytes, content_type: str = "text/plain",
    ) -> Response:
        headers = Headers()
        headers["Content-Type"] = content_type
        headers["Content-Length"] = str(len(body))
        headers["Cache-Control"] = "no-store"
        return Response(status.value, status.phrase, headers, body)

    # ── cFS I/O ───────────────────────────────────────────
    def _send_to_lab_enable(self) -> None:
        """Discover our LAN-facing IP and tell cFS to stream TLM there."""
        assert self.cmd_sock is not None
        try:
            probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            probe.connect((self.cfs_host, self.ci_port))
            our_ip = probe.getsockname()[0]
            probe.close()
        except OSError:
            our_ip = "127.0.0.1"
        pkt = build_to_lab_enable(our_ip)
        self.cmd_sock.sendto(pkt, (self.cfs_host, self.ci_port))
        log.info("TO_LAB enable sent (telemetry → %s:%d)", our_ip, self.tlm_port)

    def _request_lrmh_hk(self) -> None:
        """Send LRMH_SEND_HK to trigger immediate housekeeping."""
        assert self.cmd_sock is not None
        pkt = build_ccsds_cmd(LRMH_SEND_HK_MID, 0)
        self.cmd_sock.sendto(pkt, (self.cfs_host, self.ci_port))

    def _send_raw(self, pkt: bytes) -> None:
        assert self.cmd_sock is not None
        self.cmd_sock.sendto(pkt, (self.cfs_host, self.ci_port))

    # ── WebSocket handlers ────────────────────────────────
    async def ws_handler(self, websocket: Any) -> None:
        self.clients.add(websocket)
        addr = websocket.remote_address
        log.info("client connected: %s", addr)

        # Resend TO_LAB enable on every client connect. The cFS TO_LAB
        # forgets between cFS restarts, and it's cheap insurance.
        self._send_to_lab_enable()
        self._request_lrmh_hk()

        # Legacy welcome
        await websocket.send(json.dumps({
            "type": "connected",
            "cfsHost": self.cfs_host,
            "ciPort": self.ci_port,
            "tlmPort": self.tlm_port,
        }))
        # New-schema welcome event
        await self._send_one(websocket, {
            "event": "info",
            "text": f"Bridge connected ({self.cfs_host}:{self.ci_port})",
        })
        # If we already have cached HK, replay it so the UI renders
        # mode state immediately without waiting for the next tick.
        if self.last_hk is not None:
            await self._send_one(websocket, self.last_hk)

        try:
            async for message in websocket:
                await self.handle_cmd(message, websocket)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)
            log.info("client disconnected: %s", addr)

    async def handle_cmd(
        self, message: str, websocket: Any
    ) -> None:
        try:
            msg = json.loads(message)
        except json.JSONDecodeError as exc:
            await self._send_one(websocket, {
                "type": "error", "message": f"bad json: {exc}",
                "event": "error", "text": f"Malformed command JSON: {exc}",
            })
            return

        raw_cmd = msg.get("cmd")
        if not raw_cmd:
            await self._send_one(websocket, {
                "event": "error", "text": "Missing 'cmd' field",
            })
            return

        normalized = COMMAND_ALIASES.get(raw_cmd)
        if normalized is None:
            await self._send_one(websocket, {
                "type": "error", "message": f"Unknown command: {raw_cmd}",
                "event": "error", "text": f"Unknown command: {raw_cmd}",
            })
            return

        try:
            await self._dispatch(normalized, msg, websocket)
        except Exception as exc:  # intentional: bridge-side failure surface
            log.exception("command %s failed", normalized)
            await self._send_one(websocket, {
                "type": "error", "message": str(exc),
                "event": "error", "text": f"{normalized} failed: {exc}",
            })

    async def _dispatch(
        self, cmd: str, msg: Dict[str, Any], websocket: Any
    ) -> None:
        # New-UI-specific commands
        if cmd == "ENSURE_READY":
            await self._cmd_ensure_ready(websocket)
            return
        if cmd == "START_TEST":
            await self._cmd_start_test(msg, websocket)
            return
        if cmd == "INJECT_FAULT":
            await self._cmd_inject_fault(msg, websocket)
            return
        if cmd == "SET_SITE":
            await self._cmd_set_site(msg, websocket)
            return
        if cmd == "ACTUATOR_FAULT":
            await self._cmd_actuator_fault(msg, websocket)
            return
        if cmd == "PRESETS_LOAD":
            await self._cmd_presets_load(websocket)
            return
        if cmd == "PRESETS_SAVE":
            await self._cmd_presets_save(msg, websocket)
            return
        if cmd == "PRESETS_DELETE":
            await self._cmd_presets_delete(msg, websocket)
            return
        if cmd == "HISTORY_LOAD":
            await self._cmd_history_load(websocket)
            return
        if cmd == "RAW_CMD":
            await self._cmd_raw_cmd(msg, websocket)
            return
        if cmd == "FS_LIST":
            await self._cmd_fs_list(websocket)
            return
        if cmd == "FS_FETCH":
            await self._cmd_fs_fetch(msg, websocket)
            return

        # Direct cFS commands (legacy + new). Build the packet, send it,
        # emit a dual-schema ack.
        params = msg.get("params", {}) if isinstance(msg, dict) else {}
        pkt, label = self._build_direct_cmd(cmd, params, msg)
        if pkt is None:
            return
        self._send_raw(pkt)
        await self._emit_cmd_ack(websocket, cmd, label, len(pkt))

    def _build_direct_cmd(
        self, cmd: str, params: Dict[str, Any], msg: Dict[str, Any]
    ) -> Tuple[Optional[bytes], str]:
        """Map a normalized cmd name to a CCSDS packet. Returns (pkt, label)."""
        label = cmd
        # LRMH_CMD MID commands
        if cmd in ("NOOP", "RESET_COUNTERS", "INIT_INSTRUMENT",
                   "DEPLOY", "STOW", "ABORT", "REQUEST_REPORT",
                   "RESET_FAULT", "SAFE", "POWER_OFF"):
            return build_ccsds_cmd(LRMH_CMD_MID, CC[cmd]), cmd
        if cmd == "START_SITE_TEST":
            payload = build_start_payload(params or msg)
            site_id = int((params or msg).get("siteId", 17))
            return (build_ccsds_cmd(LRMH_CMD_MID, CC["START_SITE_TEST"], payload),
                    f"START_SITE_TEST site={site_id}")
        if cmd == "SET_MODE":
            m = int(params.get("mode", 0))
            return (build_ccsds_cmd(LRMH_CMD_MID, CC["SET_MODE"],
                                    struct.pack("<BBBx", m, 0, 0)),
                    f"SET_MODE → {MODES.get(m, m)}")
        # Manual actuator commands → lrmh_seq, FcnCode SEQ_MANUAL_CTRL_CC.
        # lrmh_seq stamps the sequence token and forwards via
        # LRMH_SEQ_TO_CTRL_MID. Allowed only when phase == IDLE.
        if cmd == "CTRL_MOVE":
            pos  = float(params.get("position", 0.0))
            rate = float(params.get("rate", 0.005))
            timeout = float(params.get("timeout", 60.0))
            return (build_ccsds_cmd(LRMH_SEQ_CMD_MID, CC["SEQ_MANUAL_CTRL"],
                                    build_manual_ctrl_payload(
                                        CTRL_MODE["MOVE"], pos=pos,
                                        rate=rate, timeout=timeout)),
                    label)
        if cmd == "CTRL_FORCE":
            f    = float(params.get("force", 0.0))
            rate = float(params.get("rate", 0.003))
            timeout = float(params.get("timeout", 60.0))
            return (build_ccsds_cmd(LRMH_SEQ_CMD_MID, CC["SEQ_MANUAL_CTRL"],
                                    build_manual_ctrl_payload(
                                        CTRL_MODE["FORCE"], force=f,
                                        rate=rate, timeout=timeout)),
                    label)
        if cmd == "CTRL_RETRACT":
            rate = float(params.get("rate", 0.008))
            timeout = float(params.get("timeout", 30.0))
            return (build_ccsds_cmd(LRMH_SEQ_CMD_MID, CC["SEQ_MANUAL_CTRL"],
                                    build_manual_ctrl_payload(
                                        CTRL_MODE["RETRACT"],
                                        rate=rate, timeout=timeout)),
                    label)
        if cmd == "CTRL_HOLD":
            return (build_ccsds_cmd(LRMH_SEQ_CMD_MID, CC["SEQ_MANUAL_CTRL"],
                                    build_manual_ctrl_payload(CTRL_MODE["HOLD"])),
                    label)
        if cmd == "CTRL_ESTOP":
            return (build_ccsds_cmd(LRMH_SEQ_CMD_MID, CC["SEQ_MANUAL_CTRL"],
                                    build_manual_ctrl_payload(CTRL_MODE["ESTOP"])),
                    label)
        if cmd == "CTRL_CLEAR_ESTOP":
            # The HAL clears an e-stop latch on Init, reached through the
            # controller's PRECHECK mode (the sim also re-homes the tool).
            return (build_ccsds_cmd(LRMH_SEQ_CMD_MID, CC["SEQ_MANUAL_CTRL"],
                                    build_manual_ctrl_payload(CTRL_MODE["PRECHECK"])),
                    label)
        # LRMH_SENSE MID
        if cmd in ("SENSE_START", "SENSE_STOP", "SENSE_ZERO_FORCE", "SENSE_ZERO_DEPTH"):
            return build_ccsds_cmd(LRMH_SENSE_CMD_MID, CC[cmd]), cmd
        # LRMH_FAULT MID
        if cmd in ("FAULT_RESET", "FAULT_ENABLE", "FAULT_DISABLE"):
            return build_ccsds_cmd(LRMH_FAULT_CMD_MID, CC[cmd]), cmd
        return None, cmd

    # ── ensure_ready: state-aware drive-to-READY ──────────
    async def _cmd_ensure_ready(self, websocket: Any) -> None:
        hk = self.last_hk
        mode = hk.get("mode") if hk else None
        mode_name = hk.get("modeName", "?") if hk else "?"

        if mode == MODE_READY:
            await self._emit_event("info", "Already READY; no action needed")
            await self._emit_ack(websocket, "ensure_ready", True, mode=mode_name)
            return
        if mode == MODE_TESTING:
            await self._emit_event("warn",
                                   "Instrument is TESTING; ensure_ready is a no-op")
            await self._emit_ack(websocket, "ensure_ready", False,
                                 reason="testing", mode=mode_name)
            return

        # For OFF/STANDBY/SAFE (or unknown), drive INIT then DEPLOY. From
        # FAULT the mode FSM rejects INIT until RESET_FAULT has taken it
        # back to SAFE, so send that first.
        if mode == MODE_FAULT:
            await self._emit_event("info", "Instrument is FAULT; sending RESET_FAULT")
            self._send_raw(build_ccsds_cmd(LRMH_CMD_MID, CC["RESET_FAULT"]))
            await asyncio.sleep(0.5)
        await self._emit_event("info",
                               f"Ensuring READY from {mode_name or 'unknown'}; "
                               f"sending INIT_INSTRUMENT")
        self._send_raw(build_ccsds_cmd(LRMH_CMD_MID, CC["INIT_INSTRUMENT"]))
        await asyncio.sleep(0.5)
        await self._emit_event("info", "Sending DEPLOY")
        self._send_raw(build_ccsds_cmd(LRMH_CMD_MID, CC["DEPLOY"]))

        # Poll HK for up to 5 s waiting for mode = READY.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            hk = self.last_hk
            if hk and hk.get("mode") == MODE_READY:
                await self._emit_event("info", "Instrument is READY")
                await self._emit_ack(websocket, "ensure_ready", True, mode="READY")
                return
            await asyncio.sleep(0.1)

        # Timeout.
        last_mode = (self.last_hk or {}).get("modeName", "unknown")
        await self._emit_event("error",
                               f"ensure_ready timed out after 5 s (mode={last_mode})")
        await self._emit_ack(websocket, "ensure_ready", False,
                             reason="timeout", mode=last_mode)

    # ── start_test: UI test_type → FSW START_SITE_TEST ────
    async def _cmd_start_test(
        self, msg: Dict[str, Any], websocket: Any
    ) -> None:
        test_type = (msg.get("test_type") or "").lower()
        site_id = int(msg.get("site_id", 17))
        ui_params = msg.get("params", {}) or {}

        mapping = TEST_TYPE_MAP.get(test_type)
        if mapping is None:
            await self._emit_event("error",
                                   f"Unknown test_type '{test_type}'; "
                                   f"expected cpt|plate|full")
            await self._emit_ack(websocket, "start_test", False,
                                 reason="bad_test_type")
            return

        # Plate falls back to full with a warning (profile_only not in
        # FSW sequence table yet; see the Operator Guide, known gaps).
        if "fallback_from" in mapping:
            await self._emit_event(
                "warn",
                f"{mapping['fallback_from']} profile not yet in sequence table; "
                f"falling back to full characterization",
            )

        # Allow CPT to override tool from UI (cone 30° vs 60°).
        tool_id = int(ui_params.get("tool_id", mapping["tool"]))

        payload_params = {
            "siteId": site_id,
            "profileId": mapping["profile"],
            "toolId": tool_id,
            "maxDepth": float(ui_params.get("max_depth_m", 0.30)),
            "maxLoad": float(ui_params.get("max_load_n", 500.0)),
            "enableRepeat": mapping["repeat"],
            "enablePost": mapping["post"],
            "enableTraction": 0,
        }
        # Optional UI pass-through for the dispatch fields.
        for opt in ("dispatchFlags", "sessionId", "sequenceNumber",
                    "pairedSiteId", "microSiteOffsetX_mm",
                    "microSiteOffsetY_mm", "repeatLoadCount"):
            if opt in ui_params:
                payload_params[opt] = ui_params[opt]

        payload = build_start_payload(payload_params)
        pkt = build_ccsds_cmd(LRMH_CMD_MID, CC["START_SITE_TEST"], payload)
        self._send_raw(pkt)

        label = f"START_SITE_TEST site={site_id} type={test_type}"
        await self._emit_event("info", label)
        await self._emit_ack(websocket, "start_test", True,
                             site_id=site_id, test_type=test_type)

    # ── preset store + site history (bridge-side JSON) ────
    def _read_json(self, path: Path, default: Any) -> Any:
        try:
            return json.loads(path.read_text())
        except FileNotFoundError:
            return default
        except (json.JSONDecodeError, OSError) as exc:
            log.warning("state read fail %s: %s; falling back to default",
                        path.name, exc)
            return default

    def _write_json(self, path: Path, data: Any) -> bool:
        try:
            tmp = path.with_suffix(path.suffix + ".tmp")
            tmp.write_text(json.dumps(data, indent=2, sort_keys=True))
            tmp.replace(path)
            return True
        except OSError as exc:
            log.error("state write fail %s: %s", path.name, exc)
            return False

    async def _cmd_presets_load(self, websocket: Any) -> None:
        # presets shape: { "<test_type>": [ {"name": str, "params": {...}} ] }
        presets = self._read_json(self.presets_path, {})
        await self._send_one(websocket, {
            "tlm": "presets", "presets": presets,
            "timestamp": time.time(),
        })

    async def _cmd_presets_save(
        self, msg: Dict[str, Any], websocket: Any,
    ) -> None:
        test_type = str(msg.get("test_type", "")).lower()
        name = str(msg.get("name", "")).strip()
        params = msg.get("params", {})
        if not test_type or not name or not isinstance(params, dict):
            await self._emit_event(
                "error",
                "presets_save requires test_type, name, params")
            await self._emit_ack(websocket, "presets_save", False,
                                 reason="bad_args")
            return
        presets = self._read_json(self.presets_path, {})
        bucket = presets.setdefault(test_type, [])
        # Replace if name already exists; otherwise append.
        replaced = False
        for entry in bucket:
            if entry.get("name") == name:
                entry["params"] = params
                replaced = True
                break
        if not replaced:
            bucket.append({"name": name, "params": params})
        if not self._write_json(self.presets_path, presets):
            await self._emit_ack(websocket, "presets_save", False,
                                 reason="write_failed")
            return
        await self._emit_event(
            "info",
            f"Preset '{name}' {'updated' if replaced else 'saved'} "
            f"({test_type})")
        # Broadcast new preset list so all connected clients refresh.
        for ws in list(self.clients):
            await self._send_one(ws, {
                "tlm": "presets", "presets": presets,
                "timestamp": time.time(),
            })
        await self._emit_ack(websocket, "presets_save", True,
                             test_type=test_type, name=name)

    async def _cmd_presets_delete(
        self, msg: Dict[str, Any], websocket: Any,
    ) -> None:
        test_type = str(msg.get("test_type", "")).lower()
        name = str(msg.get("name", "")).strip()
        if not test_type or not name:
            await self._emit_ack(websocket, "presets_delete", False,
                                 reason="bad_args")
            return
        presets = self._read_json(self.presets_path, {})
        bucket = presets.get(test_type, [])
        new_bucket = [e for e in bucket if e.get("name") != name]
        if len(new_bucket) == len(bucket):
            await self._emit_ack(websocket, "presets_delete", False,
                                 reason="not_found")
            return
        presets[test_type] = new_bucket
        self._write_json(self.presets_path, presets)
        await self._emit_event("info",
                               f"Preset '{name}' deleted ({test_type})")
        for ws in list(self.clients):
            await self._send_one(ws, {
                "tlm": "presets", "presets": presets,
                "timestamp": time.time(),
            })
        await self._emit_ack(websocket, "presets_delete", True,
                             test_type=test_type, name=name)

    async def _cmd_history_load(self, websocket: Any) -> None:
        history = self._read_json(self.history_path, [])
        await self._send_one(websocket, {
            "tlm": "site_history", "history": history,
            "timestamp": time.time(),
        })

    def _record_site_history(self, parsed: Dict[str, Any]) -> None:
        """Append a slim record from a site_result packet."""
        history = self._read_json(self.history_path, [])
        history.append({
            "timestamp": time.time(),
            "siteId": parsed.get("siteId"),
            "qualityName": parsed.get("qualityName"),
            "profileId": parsed.get("profileId"),
            "toolId": parsed.get("toolId"),
            "duration_s": parsed.get("duration_s"),
            "maxForce": parsed.get("maxForce"),
            "faultFlags": parsed.get("faultFlags", 0),
            "recoveryTier": parsed.get("recoveryTier", 0),
        })
        # Keep at most 200 entries — newest at the end. Older are
        # truncated; if you need permanent history, the cf/lrmh/
        # reports/ directory has every record.
        if len(history) > 200:
            history = history[-200:]
        self._write_json(self.history_path, history)

    # ── fs_list / fs_fetch: cf/lrmh data + reports browser ─
    #
    # Read-only browser for the cFS file store at
    # cfs/build/exe/cpu1/cf/lrmh/{data,reports}/. Sandbox: only the
    # two known subdirs are exposed; basenames must contain no
    # path separators or ".." — paths are resolved and verified
    # to live under self.cf_dir.
    _FS_SUBDIRS = ("data", "reports")
    _FS_MAX_BYTES = 2 * 1024 * 1024  # 2 MiB cap on a single fetch

    def _fs_subdir_path(self, sub: str) -> Optional[Path]:
        if sub not in self._FS_SUBDIRS:
            return None
        return self.cf_dir / sub

    async def _cmd_fs_list(self, websocket: Any) -> None:
        listings: Dict[str, Any] = {"cf_dir": str(self.cf_dir)}
        if not self.cf_dir.is_dir():
            listings["error"] = (
                f"cf_dir not found: {self.cf_dir} "
                "(launch cFS first or pass --cf-dir)")
            for sub in self._FS_SUBDIRS:
                listings[sub] = []
            await self._send_one(websocket, {
                "tlm": "fs_list", **listings, "timestamp": time.time(),
            })
            return
        for sub in self._FS_SUBDIRS:
            dirp = self._fs_subdir_path(sub)
            entries = []
            if dirp and dirp.is_dir():
                for p in sorted(dirp.iterdir(), key=lambda q: q.name):
                    if not p.is_file():
                        continue
                    try:
                        st = p.stat()
                        entries.append({
                            "name": p.name,
                            "size": st.st_size,
                            "mtime": st.st_mtime,
                        })
                    except OSError:
                        pass
            listings[sub] = entries
        await self._send_one(websocket, {
            "tlm": "fs_list", **listings, "timestamp": time.time(),
        })

    async def _cmd_fs_fetch(
        self, msg: Dict[str, Any], websocket: Any,
    ) -> None:
        params = msg.get("params", {}) or {}
        sub = str(params.get("sub", "")).lower()
        name = str(params.get("name", ""))
        if sub not in self._FS_SUBDIRS:
            await self._emit_ack(websocket, "fs_fetch", False,
                                 reason="bad_sub")
            return
        # Disallow path-traversal sneakery: name must be a basename.
        if not name or "/" in name or "\\" in name or name in (".", ".."):
            await self._emit_ack(websocket, "fs_fetch", False,
                                 reason="bad_name")
            return
        target = (self._fs_subdir_path(sub) / name).resolve()
        # Sanity: target must live under cf_dir/sub.
        try:
            target.relative_to((self.cf_dir / sub).resolve())
        except (ValueError, FileNotFoundError):
            await self._emit_ack(websocket, "fs_fetch", False,
                                 reason="path_escape")
            return
        if not target.is_file():
            await self._emit_ack(websocket, "fs_fetch", False,
                                 reason="not_found")
            return
        try:
            data = target.read_bytes()
        except OSError as exc:
            await self._emit_event(
                "error", f"fs_fetch read failed: {exc}")
            await self._emit_ack(websocket, "fs_fetch", False,
                                 reason="read_error")
            return
        truncated = False
        if len(data) > self._FS_MAX_BYTES:
            data = data[:self._FS_MAX_BYTES]
            truncated = True
        # Heuristic: text files (reports/*.txt) get sent as utf-8
        # text; everything else as base64 binary.
        is_text = name.endswith((".txt", ".json", ".log"))
        payload: Dict[str, Any] = {
            "tlm": "fs_fetch", "sub": sub, "name": name,
            "size": len(data),
            "truncated": truncated,
            "timestamp": time.time(),
        }
        if is_text:
            try:
                payload["text"] = data.decode("utf-8", errors="replace")
            except Exception:
                payload["text"] = data.decode("latin-1", errors="replace")
        else:
            import base64
            payload["base64"] = base64.b64encode(data).decode("ascii")
        await self._send_one(websocket, payload)

    # ── raw_cmd: arbitrary MID + CC + payload sender ──────
    #
    # Engineer-only escape hatch. The bridge does NOT validate MID
    # or CC values — anything that fits the CCSDS encoder goes out.
    # Use sparingly; the typed routes above are safer.
    async def _cmd_raw_cmd(
        self, msg: Dict[str, Any], websocket: Any,
    ) -> None:
        if not self.allow_raw:
            await self._emit_ack(websocket, "raw_cmd", False, reason="disabled",
                                 hint="start the bridge with --allow-raw")
            return
        params = msg.get("params", {}) or {}
        try:
            mid = int(params.get("mid"), 0) if isinstance(params.get("mid"), str) \
                  else int(params["mid"])
        except (KeyError, TypeError, ValueError):
            await self._emit_event(
                "error", "raw_cmd: missing or malformed 'mid' (int or 0xHEX)")
            await self._emit_ack(websocket, "raw_cmd", False, reason="bad_mid")
            return
        try:
            cc = int(params.get("cc", 0))
        except (TypeError, ValueError):
            await self._emit_event("error", "raw_cmd: malformed 'cc'")
            await self._emit_ack(websocket, "raw_cmd", False, reason="bad_cc")
            return
        payload_hex = params.get("payload_hex", "") or ""
        try:
            payload = bytes.fromhex(payload_hex.replace(" ", "").replace("_", ""))
        except ValueError as exc:
            await self._emit_event("error", f"raw_cmd: bad payload_hex ({exc})")
            await self._emit_ack(websocket, "raw_cmd", False, reason="bad_payload")
            return
        if not (0 <= mid <= 0xFFFF):
            await self._emit_event(
                "error", f"raw_cmd: MID 0x{mid:X} out of range")
            await self._emit_ack(websocket, "raw_cmd", False,
                                 reason="mid_range")
            return
        if not (0 <= cc <= 0xFF):
            await self._emit_event(
                "error", f"raw_cmd: CC {cc} out of range (0-255)")
            await self._emit_ack(websocket, "raw_cmd", False, reason="cc_range")
            return
        pkt = build_ccsds_cmd(mid, cc, payload)
        self._send_raw(pkt)
        label = (f"RAW_CMD MID=0x{mid:04X} CC={cc} "
                 f"payload={len(payload)}B")
        await self._emit_event("warn", f"→ {label}")
        await self._emit_ack(websocket, "raw_cmd", True,
                             mid=f"0x{mid:04X}", cc=cc,
                             payload_bytes=len(payload),
                             total_bytes=len(pkt))

    # ── inject_fault: HAL_REQ_INJECT_FAULT to the sim ─────
    async def _cmd_inject_fault(
        self, msg: Dict[str, Any], websocket: Any
    ) -> None:
        params = msg.get("params", {}) or {}
        kind_raw = params.get("kind", "none")
        if isinstance(kind_raw, str):
            kind_str = kind_raw.lower()
            kind_int = HAL_INJECT_KINDS.get(kind_str)
            if kind_int is None:
                await self._emit_event(
                    "error",
                    f"Unknown inject_fault kind '{kind_raw}'; "
                    f"expected one of {sorted(HAL_INJECT_KINDS)}")
                await self._emit_ack(websocket, "inject_fault", False,
                                     reason="bad_kind")
                return
        else:
            kind_int = int(kind_raw)
            kind_str = next(
                (k for k, v in HAL_INJECT_KINDS.items() if v == kind_int),
                str(kind_int))

        override = float(params.get("override", 0.0))
        duration_ms = int(params.get("duration_ms", 0))

        body = struct.pack("<IfII", kind_int, override, duration_ms, 0)
        if not await self._hal_request(HAL_REQ_INJECT_FAULT, body, "inject_fault", websocket):
            return

        await self._emit_event(
            "warn" if kind_int != 0 else "info",
            f"Fault inject → {kind_str} "
            f"(seq={self.hal_seq}, dur={duration_ms or 'persistent'})")
        await self._emit_ack(websocket, "inject_fault", True,
                             kind=kind_str, hal_seq=self.hal_seq)

    async def _hal_request(self, req_type: int, body: bytes, name: str, websocket: Any) -> bool:
        """One sim-only HAL request on UDP, acked or not.

        Envelope must match hal/sim/lrmh_hal_sim.c byte-for-byte:
          header  = '<IIII' (magic, seq, req_type, payload_len)
          payload = 256 bytes, zero-padded
        The response's status (the fourth uint32) is the HAL status code.
        """
        self.hal_seq = (self.hal_seq + 1) & 0xFFFFFFFF
        header = struct.pack("<IIII", HAL_MAGIC, self.hal_seq, req_type, len(body))
        envelope = header + body.ljust(256, b"\x00")
        status = None

        def exchange() -> Optional[int]:
            # Blocking UDP round trip (up to 0.5 s), run off the event loop
            # so a missing sim does not stall telemetry.
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(0.5)
            try:
                sock.sendto(envelope, (self.hal_host, self.hal_port))
                try:
                    resp, _ = sock.recvfrom(4096)
                    if len(resp) >= 16:
                        return struct.unpack_from("<i", resp, 12)[0]
                except socket.timeout:
                    return None
            finally:
                sock.close()
            return None

        try:
            status = await asyncio.get_running_loop().run_in_executor(None, exchange)
        except OSError as exc:
            await self._emit_event("error", f"{name} UDP send failed: {exc}")
            await self._emit_ack(websocket, name, False, reason="udp_error")
            return False
        if status is not None and status != 0:
            await self._emit_event("error", f"{name} rejected by the HAL (status {status})")
            await self._emit_ack(websocket, name, False, reason="hal_rejected", status=status)
            return False
        return True

    # ── set_site: HAL_REQ_SET_SITE to the sim ──────────────
    async def _cmd_set_site(self, msg: Dict[str, Any], websocket: Any) -> None:
        """Draw and load a regolith site: params {preset: name|int, seed: int}."""
        params = msg.get("params", {}) or {}
        preset_raw = params.get("preset", "lunar_highland")
        if isinstance(preset_raw, str):
            preset = REGOLITH_PRESETS.get(preset_raw.lower())
            if preset is None:
                await self._emit_event("error", f"Unknown regolith preset '{preset_raw}'; "
                                                f"expected one of {sorted(REGOLITH_PRESETS)}")
                await self._emit_ack(websocket, "set_site", False, reason="bad_preset")
                return
            preset_str = preset_raw.lower()
        else:
            preset = int(preset_raw)
            preset_str = next((k for k, v in REGOLITH_PRESETS.items() if v == preset), str(preset))
        seed = int(params.get("seed", 1)) & 0xFFFFFFFF
        body = struct.pack("<II", preset, seed)
        if not await self._hal_request(HAL_REQ_SET_SITE, body, "set_site", websocket):
            return
        await self._emit_event("info", f"Site → {preset_str} seed {seed} (seq={self.hal_seq})")
        await self._emit_ack(websocket, "set_site", True, preset=preset_str, seed=seed, hal_seq=self.hal_seq)

    # ── actuator_fault: HAL_REQ_ACTUATOR_FAULT to the sim ──
    async def _cmd_actuator_fault(self, msg: Dict[str, Any], websocket: Any) -> None:
        """A fault in the actuator physics: params {kind: stall|overcurrent|encoder_loss, active: bool}."""
        params = msg.get("params", {}) or {}
        kind_raw = params.get("kind", "stall")
        if isinstance(kind_raw, str):
            kind = ACTUATOR_FAULT_KINDS.get(kind_raw.lower())
            if kind is None:
                await self._emit_event("error", f"Unknown actuator_fault kind '{kind_raw}'; "
                                                f"expected one of {sorted(ACTUATOR_FAULT_KINDS)}")
                await self._emit_ack(websocket, "actuator_fault", False, reason="bad_kind")
                return
            kind_str = kind_raw.lower()
        else:
            kind = int(kind_raw)
            kind_str = next((k for k, v in ACTUATOR_FAULT_KINDS.items() if v == kind), str(kind))
        active = 1 if params.get("active", True) else 0
        body = struct.pack("<II", kind, active)
        if not await self._hal_request(HAL_REQ_ACTUATOR_FAULT, body, "actuator_fault", websocket):
            return
        await self._emit_event("warn" if active else "info",
                               f"Actuator fault {kind_str} → {'on' if active else 'off'} (seq={self.hal_seq})")
        await self._emit_ack(websocket, "actuator_fault", True, kind=kind_str, active=bool(active), hal_seq=self.hal_seq)

    # ── event + ack helpers ───────────────────────────────
    async def _emit_event(self, level: str, text: str) -> None:
        """Broadcast a synthesized log event to every WS client."""
        await self.broadcast(json.dumps({
            "event": level, "text": text, "timestamp": time.time(),
        }))
        log.log(
            {"info": logging.INFO, "warn": logging.WARNING,
             "error": logging.ERROR}.get(level, logging.INFO),
            text,
        )

    async def _emit_ack(
        self, websocket: Any, cmd: str, ok: bool, **extra: Any,
    ) -> None:
        """Send an ack to one client (the one that issued the command)."""
        msg: Dict[str, Any] = {
            "ack": cmd, "ok": ok, "timestamp": time.time(), **extra,
        }
        await self._send_one(websocket, msg)

    async def _emit_cmd_ack(
        self, websocket: Any, cmd: str, label: str, nbytes: int,
    ) -> None:
        """Ack for a direct cFS command. Emits both legacy and new shape."""
        # Legacy (old console expects this exact key set).
        await self._send_one(websocket, {
            "type": "cmdAck", "cmd": label, "bytes": nbytes,
            "timestamp": time.time(),
        })
        # New schema.
        await self._emit_ack(websocket, cmd.lower(), True,
                             label=label, bytes=nbytes)
        # Info-level event for the UI log panel.
        await self._emit_event("info", f"→ {label} ({nbytes} B)")

    async def _send_one(
        self, websocket: Any, obj: Dict[str, Any],
    ) -> None:
        try:
            await websocket.send(json.dumps(obj))
        except websockets.exceptions.ConnectionClosed:
            self.clients.discard(websocket)

    # ── telemetry relay ───────────────────────────────────
    async def tlm_relay(self) -> None:
        assert self.tlm_sock is not None
        loop = asyncio.get_event_loop()
        while self.running:
            try:
                data = await loop.sock_recv(self.tlm_sock, 4096)
                if not data or len(data) < TLM_HDR_LEN:
                    continue
                self.tlm_count += 1

                hdr = decode_tlm_header(data)
                if hdr is None:
                    continue
                mid, _seq, payload = hdr

                if self.tlm_count <= 5:
                    log.debug("TLM #%d: MID=0x%04X total=%d payload=%d",
                              self.tlm_count, mid, len(data), len(payload))

                parser = TLM_PARSERS.get(mid)
                if parser is not None:
                    parsed = parser(payload)
                    if parsed is not None:
                        parsed["timestamp"] = time.time()
                        # Cache HK for ensure_ready + state-aware commands.
                        if mid == LRMH_HK_TLM_MID:
                            self.last_hk = parsed
                        await self.broadcast(json.dumps(parsed))
                        # Synthesize warn events on fault entry + safe entry.
                        await self._synthesize_tlm_events(mid, parsed)
                    else:
                        await self._emit_parse_fail(mid, data, payload)
                elif (mid & 0xFFF0) == 0x08A0:
                    # LRMH telemetry range — forward as raw so the console
                    # can still count it.
                    if mid not in self.unknown_mids:
                        self.unknown_mids.add(mid)
                        log.info("LRMH MID 0x%04X (len=%d) has no decoder; forwarded raw",
                                    mid, len(data))
                    await self.broadcast(json.dumps({
                        "type": "raw", "mid": f"0x{mid:04X}",
                        "length": len(data), "timestamp": time.time(),
                    }))
                else:
                    if mid not in self.unknown_mids:
                        self.unknown_mids.add(mid)
                        log.debug("ignoring non-LRMH MID 0x%04X (len=%d)",
                                  mid, len(data))

            except BlockingIOError:
                await asyncio.sleep(0.01)
            except Exception:
                if self.running:
                    log.exception("telemetry relay error")
                    await asyncio.sleep(0.1)

    async def _synthesize_tlm_events(self, mid: int, parsed: Dict[str, Any]) -> None:
        """Emit info/warn events on key state transitions the UI log wants."""
        if mid == LRMH_HK_TLM_MID:
            prev_mode = getattr(self, "_prev_mode", None)
            mode = parsed.get("mode")
            if prev_mode is not None and mode != prev_mode:
                name = parsed.get("modeName", str(mode))
                level = "warn" if mode in (MODE_SAFE, MODE_FAULT) else "info"
                await self._emit_event(level, f"Mode → {name}")
            self._prev_mode = mode
        elif mid == LRMH_FAULT_TLM_MID:
            flags = parsed.get("errorFlags", 0)
            if flags and flags != getattr(self, "_prev_fault_flags", 0):
                await self._emit_event("warn", f"Fault flags 0x{flags:04X}")
            self._prev_fault_flags = flags
        elif mid == LRMH_REPORT_SITE_RECORD_MID:
            q = parsed.get("qualityName", "?")
            site = parsed.get("siteId", "?")
            await self._emit_event("info",
                                   f"Site {site} complete ({q})")
            # Persist a slim entry to ground/state/site_history.json
            # so the SPA's pick screen can show recent history.
            self._record_site_history(parsed)

    async def _emit_parse_fail(self, mid: int, data: bytes, payload: bytes) -> None:
        n = getattr(self, "_parse_fail_count", 0) + 1
        self._parse_fail_count = n
        if n <= 10:
            log.warning("parse fail MID 0x%04X: total=%d payload=%d hex=%s",
                        mid, len(data), len(payload), payload[:40].hex())
        elif n == 11:
            log.warning("suppressing further parse failures (10 logged)")
        await self.broadcast(json.dumps({
            "type": "raw", "mid": f"0x{mid:04X}",
            "length": len(data), "timestamp": time.time(),
        }))

    # ── broadcast ─────────────────────────────────────────
    async def broadcast(self, message: str) -> None:
        if not self.clients:
            return
        dead: Set[Any] = set()
        for ws in list(self.clients):   # handlers may mutate the set while we await
            try:
                await ws.send(message)
            except websockets.exceptions.ConnectionClosed:
                dead.add(ws)
        self.clients -= dead


# ──────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="LRMH WebSocket-to-UDP Bridge",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
  Legacy mode (WS only on :8765):
      python3 lrmh_bridge.py

  UI-serving mode (HTTP + WS on :8080; WS path is /ws):
      python3 lrmh_bridge.py --serve-ui

  On each client connect, the bridge sends a TO_LAB enable so cFS
  starts streaming telemetry to this machine.
        """,
    )
    parser.add_argument("--cfs-host", default="127.0.0.1",
                        help="cFS host IP (default: 127.0.0.1)")
    parser.add_argument("--ci-port", type=int, default=1234,
                        help="CI_LAB UDP port (default: 1234)")
    parser.add_argument("--tlm-port", type=int, default=2234,
                        help="TO_LAB UDP port to listen on (default: 2234)")
    parser.add_argument("--allow-raw", action="store_true",
                        help="Enable the Engineer view's raw command sender "
                             "(arbitrary MID/CC/payload to CI_LAB). Off by default.")
    parser.add_argument("--ws-host", default="127.0.0.1",
                        help="Address to serve the WebSocket/UI on (default "
                             "127.0.0.1; 0.0.0.0 exposes it to the LAN. There "
                             "is no authentication on the WebSocket.)")
    parser.add_argument("--ws-port", type=int, default=None,
                        help="WebSocket port (default: 8765 legacy, "
                             "8080 with --serve-ui)")
    parser.add_argument("--serve-ui", action="store_true",
                        help="Also serve lrmh_ui.html + static files on the "
                             "WS port (default off for backward-compat)")
    parser.add_argument("--ui-dir", type=Path, default=None,
                        help="Directory of static UI files "
                             "(default: same dir as this script)")
    parser.add_argument("--hal-host", default="127.0.0.1",
                        help="HAL sim host for inject_fault (default: "
                             "127.0.0.1; sim-only feature)")
    parser.add_argument("--hal-port", type=int, default=HAL_DEFAULT_PORT,
                        help=f"HAL sim UDP port (default: {HAL_DEFAULT_PORT})")
    parser.add_argument("--cf-dir", type=Path, default=None,
                        help="cFS file root (cf/lrmh) for fs_list/fs_fetch. "
                             "Default: $LRMH_CFS_ROOT/build/exe/cpu1/cf/lrmh "
                             "(LRMH_CFS_ROOT defaults to ~/lrmh-cfs-build).")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable debug logging")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s  %(levelname)-7s  %(message)s",
        datefmt="%H:%M:%S",
    )

    ws_port = args.ws_port
    if ws_port is None:
        ws_port = 8080 if args.serve_ui else 8765

    log.info("LRMH WebSocket-UDP Bridge")
    bridge = LRMHBridge(
        cfs_host=args.cfs_host,
        ci_port=args.ci_port,
        tlm_port=args.tlm_port,
        ws_port=ws_port,
        serve_ui=args.serve_ui,
        ui_static_dir=args.ui_dir,
        hal_host=args.hal_host,
        hal_port=args.hal_port,
        cf_dir=args.cf_dir,
        ws_host=args.ws_host,
        allow_raw=args.allow_raw,
    )
    try:
        asyncio.run(bridge.start())
    except KeyboardInterrupt:
        log.info("shutting down")


if __name__ == "__main__":
    main()
