#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""
LRMH ground test script.

Sends commands to the LRMH cFS image via CI_LAB (UDP :1234)
and watches <repo>/build/logs/cfs.log for the resulting EVS events.

Drives a full site test:

  1. NOOP                — verify command path
  2. INIT_INSTRUMENT     — initialize HAL, bump token epoch
  3. DEPLOY              — STANDBY → READY (CC=3 in 2.0, NOT 4 as in 1.0)
  4. START_SITE_TEST     — READY → TESTING, walk the 9-phase machine
  5. Tail the log for completion events

Usage:
  scripts/lrmh_smoke_test.py             # full smoke test
  scripts/lrmh_smoke_test.py --noop      # NOOP only
  scripts/lrmh_smoke_test.py --init      # NOOP + INIT
"""

import socket
import struct
import time
import argparse
import os
from pathlib import Path
import sys

# The launcher writes the cFS log here (override with LRMH_LOG_DIR).
DEFAULT_CFS_LOG = str(Path(os.environ.get("LRMH_LOG_DIR",
                        Path(__file__).resolve().parents[1] / "build" / "logs")) / "cfs.log")

# ── command codes (from apps/lrmh_cmd/fsw/inc/lrmh_cmd_msg.h)
LRMH_CMD_MID            = 0x18A0
LRMH_NOOP_CC            = 0
LRMH_RESET_COUNTERS_CC  = 1
LRMH_INIT_INSTRUMENT_CC = 2
LRMH_DEPLOY_CC          = 3   # NOTE: 1.0 had this as 4
LRMH_STOW_CC            = 4   # NOTE: 1.0 had this as 3
LRMH_ABORT_CC           = 5
LRMH_START_SITE_TEST_CC = 6
LRMH_REQUEST_REPORT_CC  = 7
LRMH_SET_MODE_CC        = 8
LRMH_RESET_FAULT_CC     = 9
LRMH_SAFE_CC            = 10

# DispatchFlags bits (from lrmh_cmd_msg.h)
LRMH_DISPATCH_OVERRIDE_HOST = 0x04


class CfsClient:
    """Builds and sends cFS command packets over UDP to CI_LAB."""

    def __init__(self, host="127.0.0.1", port=1234):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

    def send_noargs(self, mid, cc):
        """Send a no-args command (header only, 8 bytes)."""
        # CCSDS v1 cmd packet:
        #   stream_id   = mid (with command bit, big-endian)
        #   sequence    = 0xC000 | seq_count
        #   length      = total - 7
        #   func_code   = cc
        #   checksum    = 0 (CI_LAB doesn't validate)
        self.seq = (self.seq + 1) & 0x3FFF
        pkt = struct.pack(
            ">HHHBB", mid, 0xC000 | self.seq, 1, cc, 0
        )
        self.sock.sendto(pkt, (self.host, self.port))
        return len(pkt)

    def send_start_site_test(self, site_id=1, max_depth_m=0.02,
                             max_load_n=50.0, enable_repeat=0,
                             enable_post=0, enable_traction=0,
                             session_id=0xAAAA, sequence_no=1):
        # Defaults sized for a fast smoke test (~30 sec total).
        # Real instrument tests would use larger depth/load and
        # enable repeat/post phases.
        """
        Send LRMH_StartSiteTestCmd_t per ADR-0014.

        Layout (matches apps/lrmh_cmd/fsw/inc/lrmh_cmd_msg.h):
          CmdHeader (8 bytes: 6 primary + 1 fcn + 1 cksum)
          uint16  SiteId
          uint8   TestProfileId
          uint8   ToolId
          float   MaxDepth_m
          float   MaxLoad_N
          uint8   EnableRepeatLoad
          uint8   EnablePostProfile
          uint8   EnableTraction
          uint8   DispatchFlags
          uint16  SessionId
          uint16  SequenceNumber
          uint16  PairedSiteId
          uint16  Reserved16
          float   MicroSiteOffsetX_mm
          float   MicroSiteOffsetY_mm
          uint8   RepeatLoadCount
          uint8   Reserved[3]
        """
        self.seq = (self.seq + 1) & 0x3FFF
        # Use OVERRIDE_HOST so we don't need a host pose subscription
        dispatch_flags = LRMH_DISPATCH_OVERRIDE_HOST

        # Body (post-header) — 36 bytes
        body = struct.pack(
            "<HBBffBBBBHHHHffB3s",
            site_id,
            0,                       # TestProfileId
            0,                       # ToolId
            max_depth_m,
            max_load_n,
            enable_repeat,
            enable_post,
            enable_traction,
            dispatch_flags,
            session_id,
            sequence_no,
            0,                       # PairedSiteId
            0,                       # Reserved16
            0.0,                     # MicroSiteOffsetX_mm
            0.0,                     # MicroSiteOffsetY_mm
            0,                       # RepeatLoadCount (0 = default 3)
            b"\x00\x00\x00",         # Reserved[3]
        )

        # Total packet length = 8 header + 36 body = 44 bytes
        # CCSDS length field = total - 7
        total_len = 8 + len(body)
        length = total_len - 7
        # Header: stream_id, seq, length, fcn_code, checksum
        header = struct.pack(
            ">HHHBB", LRMH_CMD_MID, 0xC000 | self.seq, length,
            LRMH_START_SITE_TEST_CC, 0
        )
        pkt = header + body
        self.sock.sendto(pkt, (self.host, self.port))
        return len(pkt)


def tail_log(path, lines=20):
    """Print the last N lines of a log file."""
    try:
        with open(path) as f:
            data = f.read().splitlines()
        return "\n".join(data[-lines:])
    except FileNotFoundError:
        return f"(log not found: {path})"


def main():
    ap = argparse.ArgumentParser(description="LRMH smoke test")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1234)
    ap.add_argument("--log", default=DEFAULT_CFS_LOG)
    ap.add_argument("--noop", action="store_true",
                    help="send NOOP only and exit")
    ap.add_argument("--init", action="store_true",
                    help="send NOOP + INIT_INSTRUMENT and exit")
    ap.add_argument("--site-id", type=int, default=1)
    ap.add_argument("--enable-repeat", action="store_true",
                    help="enable REPEAT_LOAD phase (3 cycles by default)")
    ap.add_argument("--enable-post", action="store_true",
                    help="enable POST_PEN phase (compaction delta)")
    ap.add_argument("--enable-traction", action="store_true",
                    help="enable TRACTION phase (currently a stub)")
    ap.add_argument("--full", action="store_true",
                    help="enable all optional phases (repeat+post+traction)")
    args = ap.parse_args()
    if args.full:
        args.enable_repeat = True
        args.enable_post = True
        args.enable_traction = True

    cli = CfsClient(args.host, args.port)

    print(f"== LRMH smoke test → {args.host}:{args.port} ==")
    print()

    print("[1/4] NOOP")
    n = cli.send_noargs(LRMH_CMD_MID, LRMH_NOOP_CC)
    print(f"      sent {n} bytes")
    time.sleep(0.5)

    if args.noop:
        print()
        print("== log tail ==")
        print(tail_log(args.log, 5))
        return 0

    print("[2/4] INIT_INSTRUMENT")
    n = cli.send_noargs(LRMH_CMD_MID, LRMH_INIT_INSTRUMENT_CC)
    print(f"      sent {n} bytes")
    time.sleep(0.5)

    if args.init:
        print()
        print("== log tail ==")
        print(tail_log(args.log, 8))
        return 0

    print("[3/4] DEPLOY (CC=3 in 2.0)")
    n = cli.send_noargs(LRMH_CMD_MID, LRMH_DEPLOY_CC)
    print(f"      sent {n} bytes")
    time.sleep(0.5)

    print(f"[4/4] START_SITE_TEST site={args.site_id} "
          f"repeat={int(args.enable_repeat)} post={int(args.enable_post)} "
          f"traction={int(args.enable_traction)}")
    n = cli.send_start_site_test(site_id=args.site_id,
                                 enable_repeat=int(args.enable_repeat),
                                 enable_post=int(args.enable_post),
                                 enable_traction=int(args.enable_traction))
    print(f"      sent {n} bytes")
    time.sleep(2.0)

    print()
    print("== log tail (20 lines) ==")
    print(tail_log(args.log, 20))
    print()
    print("Done. The sequencer should walk through the 9-phase recipe.")
    print(f"Watch {args.log} for ongoing phase advances.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
