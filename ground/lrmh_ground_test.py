#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""
LRMH Ground Test Script
=======================

Sends commands to the LRMH instrument via CI_LAB (UDP port 1234)
and monitors EVS output to watch the sequencer run through a
full site test against simulated regolith.

Command sequence:
  1. NOOP                — verify command path is working
  2. INIT_INSTRUMENT     — initialize IF layer + sim models
  3. DEPLOY              — transition from STANDBY → READY
  4. START_SITE_TEST     — begin full test recipe
  5. Monitor sequencer   — watch state transitions via EVS

Usage:
  python3 lrmh_ground_test.py [--host 127.0.0.1] [--port 1234]

Requires cFS to be running with CI_LAB on UDP port 1234.
"""

import socket
import struct
import time
import argparse
import sys

# ============================================================
# cFS Command Constants
# ============================================================

# LRMH Message IDs (must match lrmh_msgids.h)
LRMH_CMD_MID = 0x18A0

# LRMH Command Codes (must match lrmh_msg.h)
LRMH_NOOP_CC             = 0
LRMH_RESET_COUNTERS_CC   = 1
LRMH_INIT_INSTRUMENT_CC  = 2
LRMH_DEPLOY_CC           = 3
LRMH_STOW_CC             = 4
LRMH_ABORT_CC            = 5
LRMH_START_SITE_TEST_CC  = 6
LRMH_SET_MODE_CC         = 8

# cFS CCSDS v1 command packet header layout:
#   Bytes 0-1: Stream ID  (MID | 0x1800 for commands, big-endian)
#   Bytes 2-3: Sequence   (0xC000 | sequence_count, big-endian)
#   Bytes 4-5: Length      (total_bytes - 7, big-endian)
#   Byte  6:   Function code (command code)
#   Byte  7:   Checksum (0 for CI_LAB, it doesn't validate)

class CfsCommandSender:
    """Builds and sends cFS command packets over UDP to CI_LAB."""

    def __init__(self, host="127.0.0.1", port=1234):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq_count = 0

    def build_cmd_header(self, mid, fc, payload_len):
        """Build a 8-byte CCSDS v1 command header."""
        # Stream ID: for commands, bit 12 set (0x1000) is already in mid
        stream_id = mid
        # Sequence: standalone packet (0xC000) | count
        self.seq_count = (self.seq_count + 1) & 0x3FFF
        sequence = 0xC000 | self.seq_count
        # Length: total packet length - 7 (header is 8 bytes, so payload_len + 1)
        length = payload_len + 8 - 7  # +8 for header, -7 per CCSDS
        # Secondary header: function code + checksum
        header = struct.pack(">HHH", stream_id, sequence, length)
        header += struct.pack("BB", fc, 0)  # FC + checksum=0
        return header

    def send_no_args(self, mid, fc, name="CMD"):
        """Send a command with no payload (just header)."""
        header = self.build_cmd_header(mid, fc, 0)
        self.sock.sendto(header, (self.host, self.port))
        print(f"  → Sent {name} (MID=0x{mid:04X} FC={fc}) [{len(header)} bytes]")

    def send_start_site_test(self, site_id=17, profile_id=1, tool_id=1,
                              max_depth=0.30, max_load=500.0,
                              enable_repeat=1, enable_post=1, enable_traction=0,
                              dispatch_flags=0x08, session_id=1):
        """Send START_SITE_TEST command with 2.0 (36-byte) payload.

        Default dispatch_flags = LRMH_DISPATCH_WITNESS_TARGET (0x08) so
        the FSW skips the host-nav precondition (no rover sending host
        pose telemetry in the standalone simulator). Pass 0 for a real
        deployment.
        """
        self._site_seq = (getattr(self, "_site_seq", 0) + 1) & 0xFFFF
        # 2.0 layout — see apps/lrmh_cmd/fsw/inc/lrmh_cmd_msg.h.
        payload = struct.pack(
            "<HBBffBBBBHHHHffB3x",
            site_id, profile_id, tool_id,
            max_depth, max_load,
            enable_repeat, enable_post, enable_traction,
            dispatch_flags,
            session_id, self._site_seq, 0, 0,   # SessionId, SequenceNumber, PairedSiteId, Reserved16
            0.0, 0.0,                           # MicroSiteOffsetX_mm, MicroSiteOffsetY_mm
            0,                                  # RepeatLoadCount (0 → default 3 cycles)
        )

        header = self.build_cmd_header(LRMH_CMD_MID, LRMH_START_SITE_TEST_CC, len(payload))
        packet = header + payload
        self.sock.sendto(packet, (self.host, self.port))
        print(f"  → Sent START_SITE_TEST (Site={site_id} Profile={profile_id} "
              f"Tool={tool_id} Depth={max_depth}m Load={max_load}N) [{len(packet)} bytes]")

    def close(self):
        self.sock.close()


def print_banner(text):
    """Print a visible section banner."""
    width = 60
    print()
    print("=" * width)
    print(f"  {text}")
    print("=" * width)


def print_step(step_num, text):
    """Print a numbered step."""
    print(f"\n--- Step {step_num}: {text} ---")


def wait_and_observe(seconds, label=""):
    """Wait while the user watches EVS output in the cFS terminal."""
    if label:
        print(f"  ⏳ Waiting {seconds}s — watch cFS terminal for: {label}")
    else:
        print(f"  ⏳ Waiting {seconds}s...")
    time.sleep(seconds)


def run_full_site_test(host, port):
    """Execute the complete LRMH ground test sequence."""

    sender = CfsCommandSender(host, port)

    print_banner("LRMH GROUND TEST SCRIPT")
    print(f"  Target: {host}:{port}")
    print(f"  Time:   {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    print("  Make sure cFS is running and you can see EVS output.")
    print("  This script sends commands and pauses so you can")
    print("  observe the sequencer state transitions in real time.")

    # --------------------------------------------------------
    # Step 1: NOOP — verify the command path works
    # --------------------------------------------------------
    print_step(1, "NOOP — verify command path")
    sender.send_no_args(LRMH_CMD_MID, LRMH_NOOP_CC, "NOOP")
    wait_and_observe(2, "LRMH_CMD: NOOP Cmd=1 Err=0 Mode=1")

    # --------------------------------------------------------
    # Step 2: RESET COUNTERS
    # --------------------------------------------------------
    print_step(2, "RESET COUNTERS")
    sender.send_no_args(LRMH_CMD_MID, LRMH_RESET_COUNTERS_CC, "RESET_COUNTERS")
    wait_and_observe(1, "LRMH_CMD: counters reset")

    # --------------------------------------------------------
    # Step 3: INIT_INSTRUMENT — initialize IF layer + sim
    # --------------------------------------------------------
    print_step(3, "INIT_INSTRUMENT — HAL init, token epoch bump")
    sender.send_no_args(LRMH_CMD_MID, LRMH_INIT_INSTRUMENT_CC, "INIT_INSTRUMENT")
    wait_and_observe(2, "LRMH_CMD: instrument initialized, token epoch bumped")

    # --------------------------------------------------------
    # Step 4: DEPLOY — transition STANDBY → READY
    # --------------------------------------------------------
    print_step(4, "DEPLOY — transition to READY mode")
    sender.send_no_args(LRMH_CMD_MID, LRMH_DEPLOY_CC, "DEPLOY")
    wait_and_observe(2, "LRMH_CMD: Mode 1 → 2 (trigger 2)   (STANDBY to READY)")

    # --------------------------------------------------------
    # Step 5: Another NOOP to confirm READY state
    # --------------------------------------------------------
    print_step(5, "NOOP — confirm READY state")
    sender.send_no_args(LRMH_CMD_MID, LRMH_NOOP_CC, "NOOP")
    wait_and_observe(2, "Mode=2 (READY)")

    # --------------------------------------------------------
    # Step 6: START_SITE_TEST — run full test recipe
    # --------------------------------------------------------
    print_step(6, "START_SITE_TEST — full recipe on simulated regolith")
    print()
    print("  Test parameters:")
    print("    Site ID:        17")
    print("    Profile:        1 (baseline_standard)")
    print("    Tool:           1 (cone_30deg)")
    print("    Max depth:      0.30 m")
    print("    Max load:       500 N")
    print("    Repeat load:    YES (3 cycles)")
    print("    Post-pen:       YES")
    print("    Traction:       NO")
    print()

    sender.send_start_site_test(
        site_id=17,
        profile_id=1,
        tool_id=1,
        max_depth=0.30,
        max_load=500.0,
        enable_repeat=1,
        enable_post=1,
        enable_traction=0
    )

    print()
    print("  Expected sequencer state progression:")
    print("    IDLE → PRECHECK → POSITIONING → BASELINE_PEN")
    print("    → LOAD_RAMP → REPEAT_LOAD (x3) → POST_PEN")
    print("    → PROCESSING → REPORTING → IDLE")
    print()
    print("  Watch the cFS terminal for EVS messages from LRMH_SEQ.")
    print("  The sequencer will advance through each state.")

    # A 0.30 m / 500 N test with three load cycles takes several
    # minutes (two penetrations at 5 mm/s and four 500 N ramps at
    # 3 N/s). The sequencer narrates each phase in the cFS log; this
    # script does not tail it, so it simply waits a fixed time and
    # leaves the test running if it has not finished.
    wait_and_observe(60, "PRECHECK → POSITIONING → BASELINE_PEN")
    wait_and_observe(60, "LOAD_RAMP (retract, then load ramp)")
    wait_and_observe(60, "REPEAT_LOAD cycles, POST_PEN, PROCESSING, REPORTING")
    print("  (the test may still be running; watch the cFS log for"
          " LRMH_SEQ: COMPLETE)")

    # --------------------------------------------------------
    # Step 8: Final NOOP to see counters
    # --------------------------------------------------------
    print_step(8, "Final NOOP — check command counters")
    sender.send_no_args(LRMH_CMD_MID, LRMH_NOOP_CC, "NOOP")
    wait_and_observe(2, "Final counters")

    # --------------------------------------------------------
    # Done
    # --------------------------------------------------------
    print_banner("TEST COMPLETE")
    print()
    print("  Check the cFS terminal EVS output for:")
    print("    ✓ All sequencer state transitions")
    print("    ✓ Sensor acquisition start/stop")
    print("    ✓ Processing results (MaxF, MaxZ, Stiffness, LayerBreak)")
    print("    ✓ Report publication (binary record + text summary)")
    print("    ✓ Site result telemetry published")
    print("    ✓ Quality flag = VALID")
    print()
    print("  If the sequencer stayed in IDLE, check that:")
    print("    - INIT_INSTRUMENT succeeded (Mode=STANDBY)")
    print("    - DEPLOY succeeded (Mode=READY)")
    print("    - START_SITE_TEST was accepted (not rejected)")
    print()
    print("  DS captures the record under the cFS image's cf/lrmh/:")
    print("    data/lrmh_site_<time>.bin     (binary record)")
    print("    reports/lrmh_human_<time>.txt (text summary)")
    print("  Read them back with scripts/lrmh_read_record.py --latest")
    print()

    sender.close()


def run_abort_test(host, port):
    """Test the abort/safing path."""

    sender = CfsCommandSender(host, port)

    print_banner("LRMH ABORT TEST")

    print_step(1, "INIT + DEPLOY")
    sender.send_no_args(LRMH_CMD_MID, LRMH_INIT_INSTRUMENT_CC, "INIT_INSTRUMENT")
    wait_and_observe(1)
    sender.send_no_args(LRMH_CMD_MID, LRMH_DEPLOY_CC, "DEPLOY")
    wait_and_observe(1)

    print_step(2, "START_SITE_TEST")
    sender.send_start_site_test(site_id=22, profile_id=1, tool_id=1)
    wait_and_observe(3, "Let sequencer start running...")

    print_step(3, "ABORT — should trigger E-stop + retract + SAFE mode")
    sender.send_no_args(LRMH_CMD_MID, LRMH_ABORT_CC, "ABORT")
    wait_and_observe(3, "LRMH_CMD: ABORT executed, Mode=SAFE")

    print_step(4, "NOOP — confirm SAFE state")
    sender.send_no_args(LRMH_CMD_MID, LRMH_NOOP_CC, "NOOP")
    wait_and_observe(2)

    print_banner("ABORT TEST COMPLETE")
    print("  Check EVS for: E-stop, retract command, SAFE mode transition")
    print("  A site that was in progress is reported with quality FAULT")

    sender.close()


def run_noop_only(host, port):
    """Just send a NOOP to verify connectivity."""
    sender = CfsCommandSender(host, port)
    print_banner("LRMH CONNECTIVITY TEST")
    sender.send_no_args(LRMH_CMD_MID, LRMH_NOOP_CC, "NOOP")
    print("  Check cFS terminal for: LRMH_CMD: NOOP")
    sender.close()


# ============================================================
# Main
# ============================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="LRMH Ground Test Script")
    parser.add_argument("--host", default="127.0.0.1", help="CI_LAB UDP host")
    parser.add_argument("--port", type=int, default=1234, help="CI_LAB UDP port")
    parser.add_argument("--test", choices=["full", "abort", "noop"],
                        default="full",
                        help="Test to run: full site test, abort test, or noop only")

    args = parser.parse_args()

    try:
        if args.test == "full":
            run_full_site_test(args.host, args.port)
        elif args.test == "abort":
            run_abort_test(args.host, args.port)
        elif args.test == "noop":
            run_noop_only(args.host, args.port)
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
        sys.exit(0)
    except Exception as e:
        print(f"\nERROR: {e}")
        sys.exit(1)
