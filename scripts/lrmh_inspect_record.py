#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""
LRMH site-record inspector.

Listens on UDP :2234 (TO_LAB downlink — TO_LAB_MISSION_TLM_PORT
for cpu1), captures the next LRMH_REPORT_SITE_RECORD_MID
(0x08AA) packet, deserializes it per
apps/lrmh_report/fsw/inc/lrmh_report_msg.h, and prints the
identification, per-phase summaries, and aggregated science
quantities.

Usage:
  scripts/lrmh_inspect_record.py            # listen and print first record
  scripts/lrmh_inspect_record.py --port 2234

Run in one terminal, then run scripts/lrmh_smoke_test.py in
another to drive a site test. The downlink record arrives
shortly after the "site N published" event in the cFS log.
"""

import socket
import struct
import argparse
import sys

LRMH_REPORT_SITE_RECORD_MID = 0x08AA
LRMH_MAX_PHASES_PER_TEST    = 8

# cFS Draco native telemetry header: 6-byte CCSDS primary + 6-byte
# secondary (time) + 4 bytes of padding to the 8-byte alignment the
# first payload field needs = 16 bytes before the payload.
TLM_HEADER_LEN = 16

PHASE_NAMES = {
    0: "IDLE",
    1: "PRECHECK",
    2: "POSITIONING",
    3: "BASELINE_PEN",
    4: "LOAD_RAMP",
    5: "REPEAT_LOAD",
    6: "POST_PEN",
    7: "TRACTION",
    8: "PROCESSING",
    9: "REPORTING",
}

# Match the canonical LRMH_Quality_t enum from msg/lrmh_msg.h. The
# Ground tools use the
# 2.0 enum names so a record decoded here reads the same as the
# FSW human-text file.
QUALITY_NAMES = {
    0: "VALID",
    1: "PARTIAL",
    2: "FAULT",
}


def decode_phase(buf, off):
    """Decode one LRMH_SitePhaseSummary_t (32 bytes)."""
    fmt = "<BB2sIffffff"
    sz  = struct.calcsize(fmt)
    (phase, quality, _res, samples,
     max_force_n, max_sinkage_m, stiffness, bearing,
     layer_break_m, duration_s) = struct.unpack_from(fmt, buf, off)
    return {
        "phase":         phase,
        "phase_name":    PHASE_NAMES.get(phase, f"?{phase}"),
        "quality":       quality,
        "quality_name":  QUALITY_NAMES.get(quality, f"?{quality}"),
        "samples":       samples,
        "max_force_n":   max_force_n,
        "max_sinkage_m": max_sinkage_m,
        "stiffness":     stiffness,
        "bearing":       bearing,
        "layer_break_m": layer_break_m,
        "duration_s":    duration_s,
    }, sz


def decode_record(pkt):
    """Decode an LRMH_SiteResultRecord_t starting after the CCSDS header."""
    off = TLM_HEADER_LEN

    # Versioning + identification (16 bytes)
    fmt = "<B3sHHHHBBBB"
    (version, _r0, session_id, sequence_no, site_id, paired_site_id,
     profile_id, tool_id, outcome, storage_mode) = struct.unpack_from(fmt, pkt, off)
    off += struct.calcsize(fmt)

    # Timing (16 bytes)
    fmt = "<QQ"
    start_us, end_us = struct.unpack_from(fmt, pkt, off)
    off += struct.calcsize(fmt)

    # Host pose (24 bytes)
    fmt = "<6f"
    pose = struct.unpack_from(fmt, pkt, off)
    off += struct.calcsize(fmt)

    # Phases (8 × 32 = 256 bytes)
    phases = []
    for _ in range(LRMH_MAX_PHASES_PER_TEST):
        ph, sz = decode_phase(pkt, off)
        phases.append(ph)
        off += sz

    # Phase count + reserved (4 bytes)
    fmt = "<B3s"
    (phase_count, _r1) = struct.unpack_from(fmt, pkt, off)
    off += struct.calcsize(fmt)

    # Aggregated derived science (28 bytes)
    fmt = "<7f"
    (pen_mean, pen_peak, stiffness_kn_m, bearing_kpa,
     layer_break_m, compaction, variability) = struct.unpack_from(fmt, pkt, off)
    off += struct.calcsize(fmt)

    # Fault state (4 bytes)
    fmt = "<HBB"
    fault_flags, recovery_tier, _r2 = struct.unpack_from(fmt, pkt, off)
    off += struct.calcsize(fmt)

    return {
        "version":        version,
        "session_id":     session_id,
        "sequence_no":    sequence_no,
        "site_id":        site_id,
        "paired_site_id": paired_site_id,
        "profile_id":     profile_id,
        "tool_id":        tool_id,
        "outcome":        outcome,
        "outcome_name":   QUALITY_NAMES.get(outcome, f"?{outcome}"),
        "storage_mode":   storage_mode,
        "start_us":       start_us,
        "end_us":         end_us,
        "host_pose":      pose,
        "phases":         phases,
        "phase_count":    phase_count,
        "agg": {
            "pen_mean_kpa":    pen_mean,
            "pen_peak_kpa":    pen_peak,
            "stiffness_kn_m":  stiffness_kn_m,
            "bearing_kpa":     bearing_kpa,
            "layer_break_m":   layer_break_m,
            "compaction":      compaction,
            "variability":     variability,
        },
        "fault_flags":   fault_flags,
        "recovery_tier": recovery_tier,
        "decoded_bytes": off,
        "packet_bytes":  len(pkt),
    }


def parse_ccsds_mid(pkt):
    """Return the MID from a CCSDS primary header."""
    if len(pkt) < 6:
        return None
    return struct.unpack(">H", pkt[:2])[0]


def print_record(r):
    print(f"=== Site Result Record (v{r['version']}) ===")
    print(f"  session={r['session_id']:#06x}  seq={r['sequence_no']}  "
          f"site={r['site_id']}  paired={r['paired_site_id']}")
    print(f"  profile={r['profile_id']}  tool={r['tool_id']}  "
          f"outcome={r['outcome_name']}  storage_mode={r['storage_mode']}")
    print(f"  duration_us={r['end_us'] - r['start_us']}  "
          f"({(r['end_us'] - r['start_us']) / 1e6:.3f} s)")
    print(f"  host_pose=({r['host_pose'][0]:.3f}, {r['host_pose'][1]:.3f}, "
          f"{r['host_pose'][2]:.3f}) m")
    print()
    print(f"  Phases ({r['phase_count']} populated):")
    for i, p in enumerate(r['phases']):
        if p['samples'] == 0 and p['phase'] == 0 and p['max_force_n'] == 0:
            continue
        print(f"    [{i}] {p['phase_name']:14s} q={p['quality_name']:8s} "
              f"n={p['samples']:6d} "
              f"MaxF={p['max_force_n']:7.2f} N  "
              f"Sink={p['max_sinkage_m'] * 1000:6.2f} mm  "
              f"Stiff={p['stiffness']:7.2f}  "
              f"Bear={p['bearing']:7.2f}  "
              f"LayerBrk={p['layer_break_m'] * 1000:6.2f} mm  "
              f"dur={p['duration_s']:.3f} s")
    print()
    print(f"  Aggregated derived science:")
    a = r['agg']
    print(f"    pen_mean        = {a['pen_mean_kpa']:.2f} kPa")
    print(f"    pen_peak        = {a['pen_peak_kpa']:.2f} kPa")
    print(f"    stiffness       = {a['stiffness_kn_m']:.2f} kN/m")
    print(f"    bearing         = {a['bearing_kpa']:.2f} kPa")
    print(f"    layer_break     = {a['layer_break_m'] * 1000:.2f} mm")
    print(f"    compaction      = {a['compaction']:.4f}")
    print(f"    variability     = {a['variability']:.4f}")
    print()
    print(f"  Fault flags  = 0x{r['fault_flags']:04x}")
    print(f"  Recovery tier= {r['recovery_tier']}")
    print(f"  Decoded {r['decoded_bytes']}/{r['packet_bytes']} bytes")


def main():
    ap = argparse.ArgumentParser(description="LRMH site-record inspector")
    ap.add_argument("--port", type=int, default=2234,
                    help="UDP port to listen on (TO_LAB downlink)")
    ap.add_argument("--timeout", type=float, default=120.0,
                    help="Seconds to wait for a record")
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", args.port))
    s.settimeout(args.timeout)

    print(f"Listening on UDP :{args.port} for MID 0x{LRMH_REPORT_SITE_RECORD_MID:04X} "
          f"(timeout {args.timeout}s)")
    print("Run scripts/lrmh_smoke_test.py in another terminal to drive a site test.")
    print()

    seen = 0
    other_mid_counts = {}
    last_progress = 0
    try:
        while True:
            try:
                pkt, _ = s.recvfrom(65535)
            except socket.timeout:
                print(f"Timeout — no SITE_RECORD packet received in {args.timeout}s")
                if other_mid_counts:
                    print(f"Other MIDs seen during wait:")
                    for m, n in sorted(other_mid_counts.items()):
                        print(f"  0x{m:04x}: {n}")
                return 1
            mid = parse_ccsds_mid(pkt)
            if mid != LRMH_REPORT_SITE_RECORD_MID:
                other_mid_counts[mid] = other_mid_counts.get(mid, 0) + 1
                total = sum(other_mid_counts.values())
                if total - last_progress >= 200:
                    last_progress = total
                    print(f"  ...{total} non-target packets so far "
                          f"(MIDs: {sorted(other_mid_counts.keys())})", flush=True)
                continue
            seen += 1
            print(f"-- packet #{seen}, {len(pkt)} bytes, MID 0x{mid:04X} --", flush=True)
            try:
                r = decode_record(pkt)
                print_record(r)
            except struct.error as e:
                print(f"Decode error: {e}")
                print(f"Raw: {pkt.hex()}")
            sys.stdout.flush()
            return 0
    finally:
        s.close()


if __name__ == "__main__":
    sys.exit(main())
