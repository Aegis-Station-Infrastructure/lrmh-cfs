#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""
LRMH Command Console
====================

Interactive command-line interface for controlling the LRMH instrument
via CI_LAB (UDP port 1234).

Type 'help' for available commands, 'quit' to exit.

Usage:
  python3 lrmh_console.py [--host 127.0.0.1] [--port 1234]
"""

import socket
import struct
import time
import argparse
import readline
import os

# ============================================================
# Message IDs (must match lrmh_msgids.h)
# ============================================================
LRMH_CMD_MID       = 0x18A0
LRMH_SEND_HK_MID   = 0x18A1
LRMH_SEQ_CMD_MID   = 0x18A2
LRMH_CTRL_CMD_MID  = 0x18A3
LRMH_SENSE_CMD_MID = 0x18A4
LRMH_FAULT_CMD_MID = 0x18A7

# ============================================================
# Command Codes (must match lrmh_msg.h)
# ============================================================
# lrmh_cmd
LRMH_NOOP_CC             = 0
LRMH_RESET_COUNTERS_CC   = 1
LRMH_INIT_INSTRUMENT_CC  = 2
LRMH_DEPLOY_CC           = 3
LRMH_STOW_CC             = 4
LRMH_ABORT_CC            = 5
LRMH_START_SITE_TEST_CC  = 6
LRMH_SET_MODE_CC         = 8
LRMH_RESET_FAULT_CC      = 9
LRMH_SAFE_CC             = 10
LRMH_POWER_OFF_CC        = 11

# Manual actuator control goes through lrmh_seq (LRMH_SEQ_CMD_MID,
# function code 30), which stamps the sequence token and forwards to
# lrmh_ctrl; it is accepted only while the sequencer is IDLE. The mode
# byte is LRMH_CTRL_MODE_* from apps/lrmh_ctrl/fsw/inc/lrmh_ctrl_msg.h.
LRMH_SEQ_MANUAL_CTRL_CC  = 30
CTRL_MODE_POSITION       = 0
CTRL_MODE_FORCE          = 1
CTRL_MODE_RETRACT        = 2
CTRL_MODE_HOLD           = 3
CTRL_MODE_ESTOP          = 4
CTRL_MODE_PRECHECK       = 5   # HAL Init / self-test; clears an e-stop latch

# lrmh_sense
LRMH_SENSE_START_CC      = 2
LRMH_SENSE_STOP_CC       = 3
LRMH_SENSE_ZERO_FORCE_CC = 4
LRMH_SENSE_ZERO_DEPTH_CC = 5

# lrmh_fault
LRMH_FAULT_RESET_CC      = 1
LRMH_FAULT_ENABLE_CC     = 2
LRMH_FAULT_DISABLE_CC    = 3

# ============================================================
# Mode and State names for display
# ============================================================
MODES = {0: "OFF", 1: "STANDBY", 2: "READY", 3: "TESTING", 4: "SAFE", 5: "FAULT"}
SEQ_STATES = {
    0: "IDLE", 1: "PRECHECK", 2: "POSITIONING", 3: "BASELINE_PEN",
    4: "LOAD_RAMP", 5: "REPEAT_LOAD", 6: "POST_PEN", 7: "TRACTION",
    8: "PROCESSING", 9: "REPORTING"
}
QUALITY = {0: "VALID", 1: "PARTIAL", 2: "FAULT"}   # LRMH_Quality_t
PROFILES = {1: "baseline_standard", 2: "quick_survey", 3: "deep_characterization"}
TOOLS = {1: "cone_30deg", 2: "plate_80mm", 3: "cone_60deg"}

# ============================================================
# Command Sender
# ============================================================
class CmdSender:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

    def _header(self, mid, fc, payload_len):
        self.seq = (self.seq + 1) & 0x3FFF
        length = payload_len + 8 - 7
        hdr = struct.pack(">HHH", mid, 0xC000 | self.seq, length)
        hdr += struct.pack("BB", fc, 0)
        return hdr

    def send(self, mid, fc, payload=b"", label=""):
        pkt = self._header(mid, fc, len(payload)) + payload
        self.sock.sendto(pkt, (self.host, self.port))
        if label:
            print(f"  \033[32m→ Sent {label} ({len(pkt)} bytes)\033[0m")

    def close(self):
        self.sock.close()

# ============================================================
# Command Handlers
# ============================================================
def cmd_noop(sender, args):
    """Send NOOP — verify command path, see counters and mode"""
    sender.send(LRMH_CMD_MID, LRMH_NOOP_CC, label="NOOP")

def cmd_reset(sender, args):
    """Reset command and error counters to zero"""
    sender.send(LRMH_CMD_MID, LRMH_RESET_COUNTERS_CC, label="RESET_COUNTERS")

def cmd_init(sender, args):
    """Initialize instrument (HAL init, token epoch bump) → Mode=STANDBY"""
    sender.send(LRMH_CMD_MID, LRMH_INIT_INSTRUMENT_CC, label="INIT_INSTRUMENT")

def cmd_deploy(sender, args):
    """Deploy instrument — STANDBY → READY, enables fault monitoring"""
    sender.send(LRMH_CMD_MID, LRMH_DEPLOY_CC, label="DEPLOY")

def cmd_stow(sender, args):
    """Retract actuator to home position"""
    sender.send(LRMH_CMD_MID, LRMH_STOW_CC, label="STOW")

def cmd_abort(sender, args):
    """Emergency abort — E-stop, retract, Mode → SAFE"""
    sender.send(LRMH_CMD_MID, LRMH_ABORT_CC, label="ABORT")

def cmd_start(sender, args):
    """Start a site test. Usage: start [site] [profile] [tool] [depth] [load]
    
    Defaults: site=17 profile=1 tool=1 depth=0.30 load=500
    Profiles: 1=baseline_standard  2=quick_survey  3=deep_characterization
    Tools:    1=cone_30deg  2=plate_80mm  3=cone_60deg"""

    site    = int(args[0]) if len(args) > 0 else 17
    profile = int(args[1]) if len(args) > 1 else 1
    tool    = int(args[2]) if len(args) > 2 else 1
    depth   = float(args[3]) if len(args) > 3 else 0.30
    load    = float(args[4]) if len(args) > 4 else 500.0

    # Flags — enable by default for standard profile
    repeat  = 1 if profile != 2 else 0
    post    = 1 if profile != 2 else 0
    traction = 0

    # LRMH_StartSiteTestCmd_t layout (36-byte body). DispatchFlags
    # defaults to LRMH_DISPATCH_WITNESS_TARGET (0x08) so the FSW skips
    # the host-nav precondition (no rover sending HostPose in the
    # standalone simulator). Drop to 0 if testing against a real host.
    dispatch_flags = 0x08
    session_id = 1
    seq_num = (cmd_start.__dict__.setdefault("_seq", 0) + 1) & 0xFFFF
    cmd_start._seq = seq_num
    payload = struct.pack(
        "<HBBffBBBBHHHHffB3x",
        site, profile, tool,
        depth, load,
        repeat, post, traction,
        dispatch_flags,
        session_id, seq_num, 0, 0,   # SessionId, SequenceNumber, PairedSiteId, Reserved16
        0.0, 0.0,                    # MicroSiteOffsetX_mm, MicroSiteOffsetY_mm
        0,                           # RepeatLoadCount (0 = default 3 cycles)
    )

    profile_name = PROFILES.get(profile, f"profile_{profile}")
    tool_name = TOOLS.get(tool, f"tool_{tool}")

    print(f"  Site:    {site}")
    print(f"  Profile: {profile} ({profile_name})")
    print(f"  Tool:    {tool} ({tool_name})")
    print(f"  Depth:   {depth} m")
    print(f"  Load:    {load} N")
    print(f"  Repeat:  {'YES' if repeat else 'NO'}")
    print(f"  PostPen: {'YES' if post else 'NO'}")

    sender.send(LRMH_CMD_MID, LRMH_START_SITE_TEST_CC, payload,
                label=f"START_SITE_TEST site={site}")

def _manual_ctrl(sender, mode, pos=0.0, force=0.0, rate=0.0, timeout=30.0, label=""):
    """LRMH_SeqManualCtrlCmd_t: u8 Mode, u8[3] reserved, f32 position,
    f32 force, f32 rate, f32 timeout (apps/lrmh_seq/fsw/inc/lrmh_seq_msg.h)."""
    payload = struct.pack("<BBBBffff", mode, 0, 0, 0, pos, force, rate, timeout)
    sender.send(LRMH_SEQ_CMD_MID, LRMH_SEQ_MANUAL_CTRL_CC, payload, label=label)

def cmd_estop(sender, args):
    """Emergency stop: drop motor enable and latch (sequencer must be IDLE)"""
    _manual_ctrl(sender, CTRL_MODE_ESTOP, label="E-STOP")

def cmd_clear_estop(sender, args):
    """Clear the e-stop latch: the HAL clears it on Init, reached here as the
    PRECHECK mode (the sim also re-homes the tool on Init)."""
    _manual_ctrl(sender, CTRL_MODE_PRECHECK, label="CLEAR_ESTOP (HAL init)")

def cmd_retract(sender, args):
    """Retract actuator to home. Usage: retract [rate_m/s]"""
    rate = float(args[0]) if len(args) > 0 else 0.008
    _manual_ctrl(sender, CTRL_MODE_RETRACT, rate=rate, label=f"RETRACT rate={rate}")

def cmd_move(sender, args):
    """Move the tool to a depth. Usage: move <position_m> [rate_m/s]"""
    if len(args) < 1:
        print("  Usage: move <position_m> [rate_m/s]")
        return
    pos = float(args[0])
    rate = float(args[1]) if len(args) > 1 else 0.005
    _manual_ctrl(sender, CTRL_MODE_POSITION, pos=pos, rate=rate, timeout=60.0,
                 label=f"MOVE pos={pos}m rate={rate}")

def cmd_force(sender, args):
    """Ramp to a force. Usage: force <force_N> [rate_N/s]"""
    if len(args) < 1:
        print("  Usage: force <force_N> [rate_N/s]")
        return
    f = float(args[0])
    rate = float(args[1]) if len(args) > 1 else 3.0
    _manual_ctrl(sender, CTRL_MODE_FORCE, force=f, rate=rate, timeout=60.0,
                 label=f"FORCE target={f}N rate={rate}")

def cmd_hold(sender, args):
    """Hold the current actuator position"""
    _manual_ctrl(sender, CTRL_MODE_HOLD, label="HOLD")

def cmd_sense_start(sender, args):
    """Start sensor acquisition"""
    sender.send(LRMH_SENSE_CMD_MID, LRMH_SENSE_START_CC, label="SENSE_START")

def cmd_sense_stop(sender, args):
    """Stop sensor acquisition"""
    sender.send(LRMH_SENSE_CMD_MID, LRMH_SENSE_STOP_CC, label="SENSE_STOP")

def cmd_zero_force(sender, args):
    """Zero/tare the force sensor"""
    sender.send(LRMH_SENSE_CMD_MID, LRMH_SENSE_ZERO_FORCE_CC, label="ZERO_FORCE")

def cmd_zero_depth(sender, args):
    """Zero/home the depth encoder"""
    sender.send(LRMH_SENSE_CMD_MID, LRMH_SENSE_ZERO_DEPTH_CC, label="ZERO_DEPTH")

def cmd_fault_reset(sender, args):
    """Clear all fault flags"""
    sender.send(LRMH_FAULT_CMD_MID, LRMH_FAULT_RESET_CC, label="FAULT_RESET")

def cmd_fault_enable(sender, args):
    """Enable fault monitoring"""
    sender.send(LRMH_FAULT_CMD_MID, LRMH_FAULT_ENABLE_CC, label="FAULT_ENABLE")

def cmd_fault_disable(sender, args):
    """Disable fault monitoring"""
    sender.send(LRMH_FAULT_CMD_MID, LRMH_FAULT_DISABLE_CC, label="FAULT_DISABLE")


def cmd_quickstart(sender, args):
    """Run the full startup sequence: INIT → DEPLOY → START
    Usage: quickstart [site] [profile] [tool] [depth] [load]"""
    print("  Running: INIT → DEPLOY → START")
    print()
    cmd_init(sender, [])
    time.sleep(1.5)
    cmd_deploy(sender, [])
    time.sleep(1.5)
    cmd_start(sender, args)

# ============================================================
# Command Registry
# ============================================================
COMMANDS = {
    # Instrument-level
    "noop":        cmd_noop,
    "reset":       cmd_reset,
    "init":        cmd_init,
    "deploy":      cmd_deploy,
    "stow":        cmd_stow,
    "abort":       cmd_abort,
    "start":       cmd_start,
    "quickstart":  cmd_quickstart,

    # Actuator control
    "estop":       cmd_estop,
    "clearestop":  cmd_clear_estop,
    "retract":     cmd_retract,
    "move":        cmd_move,
    "force":       cmd_force,
    "hold":        cmd_hold,

    # Sensor
    "sense.start": cmd_sense_start,
    "sense.stop":  cmd_sense_stop,
    "zero.force":  cmd_zero_force,
    "zero.depth":  cmd_zero_depth,

    # Fault management
    "fault.reset":   cmd_fault_reset,
    "fault.enable":  cmd_fault_enable,
    "fault.disable": cmd_fault_disable,
}

# ============================================================
# Help
# ============================================================
def print_help():
    print()
    print("  \033[1mLRMH Command Console\033[0m")
    print("  ═══════════════════════════════════════════════════════")
    print()
    print("  \033[1mInstrument Commands\033[0m")
    print("  ───────────────────────────────────────────────────────")
    print("  noop              Verify command path, show counters")
    print("  reset             Reset command/error counters")
    print("  init              Initialize instrument → STANDBY")
    print("  deploy            Deploy → READY, enable fault monitor")
    print("  start [args]      Start site test (see below)")
    print("  abort             Emergency abort → SAFE")
    print("  stow              Retract actuator to home")
    print("  quickstart [args] INIT + DEPLOY + START in sequence")
    print()
    print("  \033[1mActuator Commands\033[0m")
    print("  ───────────────────────────────────────────────────────")
    print("  move <pos> [rate] Move to position (meters)")
    print("  force <N> [rate]  Apply force (Newtons)")
    print("  retract [rate]    Retract to home")
    print("  hold              Hold current position")
    print("  estop             Emergency stop actuator")
    print("  clearestop        Clear E-stop condition")
    print()
    print("  \033[1mSensor Commands\033[0m")
    print("  ───────────────────────────────────────────────────────")
    print("  sense.start       Start acquisition")
    print("  sense.stop        Stop acquisition")
    print("  zero.force        Tare load cell")
    print("  zero.depth        Home depth encoder")
    print()
    print("  \033[1mFault Commands\033[0m")
    print("  ───────────────────────────────────────────────────────")
    print("  fault.reset       Clear all fault flags")
    print("  fault.enable      Enable fault monitoring")
    print("  fault.disable     Disable fault monitoring")
    print()
    print("  \033[1mStart Command Arguments\033[0m")
    print("  ───────────────────────────────────────────────────────")
    print("  start [site] [profile] [tool] [depth_m] [load_N]")
    print("  Defaults: site=17 profile=1 tool=1 depth=0.30 load=500")
    print()
    print("  Profiles: 1=baseline_standard")
    print("            2=quick_survey")
    print("            3=deep_characterization")
    print()
    print("  Tools:    1=cone_30deg (penetration)")
    print("            2=plate_80mm (loading)")
    print("            3=cone_60deg (deep penetration)")
    print()
    print("  \033[1mExamples\033[0m")
    print("  ───────────────────────────────────────────────────────")
    print("  quickstart                  Full startup + default test")
    print("  quickstart 42 3 1 0.35 600  Deep test at site 42")
    print("  start 17 2 1               Quick survey at site 17")
    print("  move 0.15                   Move actuator to 15cm")
    print("  force 200                   Apply 200N")
    print()
    print("  Type 'help', 'quit', or any command above.")
    print()

# ============================================================
# Tab completion
# ============================================================
def completer(text, state):
    options = [c for c in list(COMMANDS.keys()) + ["help", "quit", "exit"] if c.startswith(text)]
    return options[state] if state < len(options) else None

# ============================================================
# Main REPL
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="LRMH Command Console")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=1234)
    args = parser.parse_args()

    sender = CmdSender(args.host, args.port)

    # Set up readline with tab completion and history
    readline.set_completer(completer)
    readline.parse_and_bind("tab: complete")
    histfile = os.path.expanduser("~/.lrmh_history")
    try:
        readline.read_history_file(histfile)
    except FileNotFoundError:
        pass

    print()
    print("  \033[1m╔══════════════════════════════════════╗\033[0m")
    print("  \033[1m║       LRMH Command Console           ║\033[0m")
    print("  \033[1m║  Lunar Regolith Mechanics Head        ║\033[0m")
    print("  \033[1m╚══════════════════════════════════════╝\033[0m")
    print(f"  Target: {args.host}:{args.port}")
    print("  Type 'help' for commands, 'quit' to exit")
    print("  Tab completion enabled")
    print()

    while True:
        try:
            line = input("\033[36mlrmh>\033[0m ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()
        cmd_args = parts[1:]

        if cmd in ("quit", "exit", "q"):
            break
        elif cmd == "help":
            print_help()
        elif cmd in COMMANDS:
            try:
                COMMANDS[cmd](sender, cmd_args)
            except Exception as e:
                print(f"  \033[31mError: {e}\033[0m")
        else:
            print(f"  Unknown command: '{cmd}'. Type 'help' for available commands.")

    # Save history
    try:
        readline.write_history_file(histfile)
    except OSError:
        pass

    sender.close()
    print("  Console closed.")

if __name__ == "__main__":
    main()
