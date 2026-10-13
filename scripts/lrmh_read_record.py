#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""
LRMH site-record file reader (post-hoc).

Reads cFS DS-captured files written by lrmh_report into:
  cf/lrmh/data/lrmh_site_*.bin     (binary record, MID 0x08AA)
  cf/lrmh/reports/lrmh_human_*.txt (human text,    MID 0x08AB)

Strips the cFS DS file wrapper (cFE FS header + DS file header), finds
the embedded CCSDS packet, and dispatches by MID.

Companion to ``lrmh_inspect_record.py``, which decodes the SAME record
shape but reads it from a live UDP downlink. This script reads from a
file on disk after the test has finished.

Usage:
  scripts/lrmh_read_record.py --latest
  scripts/lrmh_read_record.py path/to/lrmh_site_NNN.bin
  scripts/lrmh_read_record.py path/to/lrmh_human_NNN.txt
  scripts/lrmh_read_record.py --json file.bin > record.json

The cFS DS file format is:

    [ cFE_FS_Header_t   64 bytes — magic 'cFE1', length, mission ID, ... ]
    [ DS_FileHeader_t   76 bytes — close timestamp, table index, filename ]
    [ CCSDS packet 0  ...                                                ]
    [ CCSDS packet 1  ...   (when DS captures multiple messages/file)    ]

DS appends every captured packet to the open file (rotating by size
or age), so a file usually holds several site records; the script
walks them all and shows the last by default.
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
from pathlib import Path
from typing import Any, Dict, Optional

# Reuse the wire-format decoder used by the live UDP listener.
SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from lrmh_inspect_record import (   # noqa: E402
    LRMH_REPORT_SITE_RECORD_MID,
    decode_record,
    print_record,
)

# 0x08AB — LRMH_REPORT_SITE_HUMAN_MID. Defined in msg/lrmh_msgids.h.
LRMH_REPORT_SITE_HUMAN_MID = 0x08AB

# cFS DS file wrapper:
#   CFE_FS_Header_t   = 8 × uint32 + char[CFE_FS_HDR_DESC_MAX_LEN(=32)]
#                     = 32 + 32 = 64 bytes
#   DS_FileHeader_t   = 2 × uint32 + 2 × uint16 + char[CFE_MISSION_MAX_PATH_LEN(=64)]
#                     = 8 + 4 + 64 = 76 bytes
# Total prefix before the first CCSDS packet:
DS_FILE_PREFIX_LEN = 64 + 76    # 140 bytes

# CCSDS Draco TLM header (matches lrmh_inspect_record.TLM_HEADER_LEN).
CCSDS_TLM_HDR_LEN = 16


# ──────────────────────────────────────────────────────────
# DS file wrapper decoders
# ──────────────────────────────────────────────────────────
def parse_ds_wrapper(buf: bytes) -> Dict[str, Any]:
    """Pull a few fields out of the cFE FS + DS file headers so we can
    pretty-print provenance alongside the decoded record."""
    if len(buf) < DS_FILE_PREFIX_LEN:
        raise ValueError(f"file shorter than DS prefix ({len(buf)} < {DS_FILE_PREFIX_LEN})")
    # CFE_FS_Header_t — multi-byte fields are big-endian on the wire.
    (content, subtype, hdr_len, sc_id, proc_id, app_id,
     time_s, time_ss) = struct.unpack_from(">8I", buf, 0)
    description = buf[32:64].split(b"\x00", 1)[0].decode("ascii", errors="replace")
    # DS_FileHeader_t — host endianness (little-endian on x86 in this build).
    close_s, close_ss = struct.unpack_from("<2I", buf, 64)
    fti, ftn = struct.unpack_from("<2H", buf, 72)
    filename = buf[76:140].split(b"\x00", 1)[0].decode("ascii", errors="replace")
    return {
        "fs_content": f"0x{content:08X}",
        "fs_description": description,
        "fs_create_time_s": time_s,
        "ds_close_time_s": close_s,
        "ds_table_index":  fti,
        "ds_filename":     filename,
    }


def parse_ccsds_mid(pkt: bytes) -> Optional[int]:
    if len(pkt) < 2:
        return None
    return struct.unpack(">H", pkt[:2])[0]


# ──────────────────────────────────────────────────────────
# Per-MID decoders
# ──────────────────────────────────────────────────────────
def decode_human_text(pkt: bytes) -> Dict[str, Any]:
    """LRMH_SiteResultHumanTlm_t — site_id (u16), text_len (u16),
    text bytes."""
    if len(pkt) < CCSDS_TLM_HDR_LEN + 4:
        raise ValueError("human packet truncated")
    site_id, text_len = struct.unpack_from("<HH", pkt, CCSDS_TLM_HDR_LEN)
    body = pkt[CCSDS_TLM_HDR_LEN + 4:]
    text = body[:text_len].decode("ascii", errors="replace")
    return {
        "mid": f"0x{LRMH_REPORT_SITE_HUMAN_MID:04X}",
        "site_id": site_id,
        "text_len": text_len,
        "text": text,
    }


# ──────────────────────────────────────────────────────────
# File reader entry point
# ──────────────────────────────────────────────────────────
def iter_ccsds_packets(buf: bytes):
    """Yield each CCSDS packet in a DS file body. The primary header's
    length field (bytes 4-5, big-endian) is total length minus 7."""
    off = 0
    while off + 6 <= len(buf):
        length = struct.unpack_from(">H", buf, off + 4)[0] + 7
        if length < 6 or off + length > len(buf):
            break
        yield buf[off:off + length]
        off += length


def read_record_file(path: Path, which: str = "last") -> Dict[str, Any]:
    """Decode one record from a DS file. DS appends every captured
    packet to the open file (it rotates by size or age), so a file
    usually holds several site records; `which` selects "first" or
    "last". The result also carries `packet_count`."""
    buf = path.read_bytes()
    wrapper = parse_ds_wrapper(buf)
    packets = list(iter_ccsds_packets(buf[DS_FILE_PREFIX_LEN:]))
    if not packets:
        raise ValueError("no CCSDS packet after DS prefix")
    chosen = packets[0] if which == "first" else packets[-1]
    mid = parse_ccsds_mid(chosen)
    base = {"path": str(path), "wrapper": wrapper, "packet_count": len(packets)}
    if mid == LRMH_REPORT_SITE_RECORD_MID:
        return {**base, "kind": "binary_record", "record": decode_record(chosen)}
    if mid == LRMH_REPORT_SITE_HUMAN_MID:
        return {**base, "kind": "human_text", "record": decode_human_text(chosen)}
    return {**base, "kind": "unknown", "mid": f"0x{mid:04X}", "raw_first_64": chosen[:64].hex()}


# ──────────────────────────────────────────────────────────
# Defaults and conveniences
# ──────────────────────────────────────────────────────────
# The image's cf/lrmh directory, where DS writes site records. Derived
# from LRMH_CFS_ROOT (default ~/lrmh-cfs-build), matching scripts/lrmh_launch.sh.
DEFAULT_CFS_RUNTIME = (
    Path(os.environ.get("LRMH_CFS_ROOT", Path.home() / "lrmh-cfs-build"))
    / "build/exe/cpu1/cf/lrmh"
)


def find_latest(prefer: str = "any") -> Path:
    """Return the most recently-modified site file under the cFS
    runtime ``cf/lrmh/`` tree. ``prefer`` can be ``bin``, ``txt``, or
    ``any``."""
    roots = []
    if prefer in ("bin", "any"):
        roots.append(DEFAULT_CFS_RUNTIME / "data")
    if prefer in ("txt", "any"):
        roots.append(DEFAULT_CFS_RUNTIME / "reports")
    files = []
    for r in roots:
        if r.is_dir():
            files.extend(p for p in r.iterdir() if p.is_file())
    if not files:
        raise FileNotFoundError(
            f"no site files under {DEFAULT_CFS_RUNTIME}; "
            f"is the cFS image running and capturing?"
        )
    files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0]


# ──────────────────────────────────────────────────────────
# Pretty printing
# ──────────────────────────────────────────────────────────
def print_decoded(decoded: Dict[str, Any]) -> None:
    w = decoded["wrapper"]
    print(f"# {decoded['path']}")
    print(f"#   captured by DS table {w['ds_table_index']} "
          f"as '{w['ds_filename']}'")
    print(f"#   cFS file timestamp:  {w['fs_create_time_s']} s "
          f"(close {w['ds_close_time_s']} s); {decoded['packet_count']} record(s) in file, "
          f"showing the last")
    print()
    if decoded["kind"] == "binary_record":
        print_record(decoded["record"])
    elif decoded["kind"] == "human_text":
        rec = decoded["record"]
        print(f"# Site {rec['site_id']} — {rec['text_len']} text bytes")
        print()
        print(rec["text"])
    else:
        print(f"# unknown packet MID {decoded.get('mid')}")
        print(decoded.get("raw_first_64"))


def main() -> int:
    ap = argparse.ArgumentParser(
        description="LRMH DS-captured site-record file reader")
    ap.add_argument("path", nargs="?", type=Path,
                    help="path to a .bin or .txt site-record file")
    ap.add_argument("--latest", action="store_true",
                    help="read the most-recent file under "
                         f"{DEFAULT_CFS_RUNTIME}")
    ap.add_argument("--prefer", choices=("bin", "txt", "any"), default="any",
                    help="with --latest, which dir to prefer (default: any)")
    ap.add_argument("--json", action="store_true",
                    help="emit decoded JSON to stdout instead of pretty text")
    args = ap.parse_args()

    if args.path is None and not args.latest:
        ap.error("supply a file path or --latest")
    if args.path is not None and args.latest:
        ap.error("supply either a file path or --latest, not both")

    path = args.path if args.path is not None else find_latest(args.prefer)
    if not path.is_file():
        print(f"not a file: {path}", file=sys.stderr)
        return 1

    try:
        decoded = read_record_file(path)
    except Exception as exc:
        print(f"decode failed for {path}: {exc}", file=sys.stderr)
        return 2

    if args.json:
        # Compact JSON (record dict has tuples for host_pose; convert).
        def _normalize(o: Any) -> Any:
            if isinstance(o, tuple):
                return list(o)
            if isinstance(o, dict):
                return {k: _normalize(v) for k, v in o.items()}
            if isinstance(o, list):
                return [_normalize(v) for v in o]
            return o
        json.dump(_normalize(decoded), sys.stdout, indent=2, default=str)
        sys.stdout.write("\n")
    else:
        print_decoded(decoded)
    return 0


if __name__ == "__main__":
    sys.exit(main())
