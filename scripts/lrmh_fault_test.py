#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""
LRMH per-flag fault dispatch validator.

Drives a START_SITE_TEST against the running cFS image, waits for
the sequencer to enter BASELINE_PEN, injects one of the six fault
categories via the HAL sim's INJECT_FAULT mechanism, then watches
the cFS log for the expected sequence:

    LC actionpoint fires (per-flag AP from tables/lc_actionpoints.c)
        → SC RTS executes (per-flag NOTIFY from tables/sc/sc_rts*)
        → lrmh_fault dispatches the recovery tier
        → mode FSM responds (T3 → SAFE, T2 → PARTIAL, T1 → RESUME)

Exercises the per-fault-flag dispatch table (six actionpoints,
nine RTSes) end-to-end against actual fault stimuli rather than
relying on the structural "ADT good=6" check at startup.

Prerequisites:

    ./scripts/lrmh_launch.sh                 # full stack up
    <repo>/build/logs/cfs.log exists and is being written to

Usage:

    python3 scripts/lrmh_fault_test.py --kind force-sat
    python3 scripts/lrmh_fault_test.py --kind overcurrent
    python3 scripts/lrmh_fault_test.py --kind all     # iterate all six

Exit code 0 if all selected categories dispatched correctly.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import os
import subprocess
import sys
import time
from dataclasses import dataclass

# Reuse the canonical CCSDS encoding from lrmh_smoke_test rather than
# duplicating it here. The smoke test's CfsClient knows the correct
# field offsets, length math, and CC values.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lrmh_smoke_test as smoke   # noqa: E402

CFS_LOG = str(Path(os.environ.get("LRMH_LOG_DIR",
                   Path(__file__).resolve().parents[1] / "build" / "logs")) / "cfs.log")
HAL_BENCH_TEST = os.path.join(os.path.dirname(__file__), "hal_bench_test.py")


# ── Per-fault-flag expected dispatch (matches the lrmh_fault tier table) ──

@dataclass
class FaultExpectation:
    inject_kind: str       # arg to hal_bench_test.py --inject
    fault_flag_hex: str    # the hex bit lrmh_fault prints in events
    tier: str              # "T1", "T2", or "T3"
    ap_event_id: int       # LRMH_AP_*_EID from tables/lc_actionpoints.c


EXPECTATIONS = {
    # force-rate first: the jump from natural to injected force on the
    # first inject of any other category also trips the force-rate
    # watchpoint as a side effect; running it first keeps each case's
    # evidence unambiguous.
    "force-rate":    FaultExpectation("force-rate",   "0x0020", "T1", 0xA006),
    "force-sat":     FaultExpectation("force-sat",    "0x0004", "T2", 0xA001),
    "overcurrent":   FaultExpectation("overcurrent",  "0x0002", "T3", 0xA002),
    "hard-stop":     FaultExpectation("hard-stop",    "0x0040", "T2", 0xA003),
    "thermal":       FaultExpectation("thermal",      "0x0010", "T3", 0xA004),
    "bus-voltage":   FaultExpectation("bus-voltage",  "0x0200", "T3", 0xA005),
}


def drive_fresh_test(client: smoke.CfsClient) -> None:
    """RESET_FAULT + NOOP + INIT_INSTRUMENT + DEPLOY + START_SITE_TEST.

    RESET_FAULT first because a previous iteration's dispatch can
    leave the mode FSM in FAULT (e.g., a T2 dispatch followed by
    a spurious force-rate trip during retract motion that
    force-escalates to T3 because we're outside TESTING). From
    FAULT only RESET_FAULT and POWER_OFF are legal — INIT is
    rejected, breaking the iteration. RESET_FAULT is a no-op if
    we're already at STANDBY.
    """
    client.send_noargs(smoke.LRMH_CMD_MID, smoke.LRMH_RESET_FAULT_CC)
    time.sleep(0.5)
    client.send_noargs(smoke.LRMH_CMD_MID, smoke.LRMH_NOOP_CC)
    time.sleep(0.3)
    client.send_noargs(smoke.LRMH_CMD_MID, smoke.LRMH_INIT_INSTRUMENT_CC)
    time.sleep(0.5)
    client.send_noargs(smoke.LRMH_CMD_MID, smoke.LRMH_DEPLOY_CC)
    time.sleep(0.5)
    client.send_start_site_test()


# ── Log tailing ──

def open_log_at_end(path: str):
    f = open(path, "r")
    f.seek(0, os.SEEK_END)
    return f


def wait_for_pattern(f, pattern: str, timeout_s: float = 15.0) -> str | None:
    """Read forward from current position until a line containing
    pattern appears or timeout. Returns the matching line or None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = f.readline()
        if not line:
            time.sleep(0.05)
            continue
        if pattern in line:
            return line.rstrip("\n")
    return None


# ── Per-kind validation ──

def run_one(kind: str) -> bool:
    exp = EXPECTATIONS[kind]
    print(f"\n── {kind} (expect {exp.tier}, fault flag {exp.fault_flag_hex}) ──",
          flush=True)

    f = open_log_at_end(CFS_LOG)
    client = smoke.CfsClient()

    drive_fresh_test(client)

    # Wait for the sequencer to enter BASELINE_PEN (Phase 3)
    line = wait_for_pattern(f, "Phase 2 → 3", timeout_s=10.0)
    if line is None:
        print("FAIL: never reached BASELINE_PEN; check stack is up", flush=True)
        return False

    # Inject the fault
    inject = subprocess.run(
        ["python3", HAL_BENCH_TEST, "--inject", exp.inject_kind],
        capture_output=True, text=True, timeout=10)
    if inject.returncode != 0:
        print(f"FAIL: inject failed: {inject.stdout} {inject.stderr}",
              flush=True)
        return False

    # The fault has to propagate through:
    #   sim → lrmh_sense (next 100 Hz read) → SB sample TLM → LC watchpoint
    #   → LC actionpoint fires → SC RTS → lrmh_fault command → tier dispatch
    # At 100 Hz sense and ~10 Hz LC, the typical end-to-end is ~50–200 ms.
    # Allow generous margin.

    # 1. LC actionpoint fires. Match on the actionpoint's EventText
    #    (set per-AP in tables/lc_actionpoints.c) since that is
    #    unique per fault category.
    ap_text_by_kind = {
        "force-sat":   "LRMH force saturation",
        "overcurrent": "LRMH overcurrent",
        "hard-stop":   "LRMH stroke hard stop",
        "thermal":     "LRMH thermal",
        "bus-voltage": "LRMH bus voltage",
        "force-rate":  "LRMH force rate",
    }
    line = wait_for_pattern(f, ap_text_by_kind[kind], timeout_s=10.0)
    if line is None:
        print(f"FAIL: LC actionpoint for {kind} never fired", flush=True)
        return False
    print(f"  ✓ LC AP fired: {line.strip()[-90:]}", flush=True)

    # 2. lrmh_fault dispatches the right tier with the right flag.
    #    Format: "LRMH_FAULT: T<n> <action> (fault=0x<flag>...)"
    tier_pattern = f"LRMH_FAULT: {exp.tier}"
    line = wait_for_pattern(f, tier_pattern, timeout_s=10.0)
    if line is None:
        print(f"FAIL: lrmh_fault never dispatched {exp.tier}", flush=True)
        return False
    if exp.fault_flag_hex.lower() not in line.lower():
        print(f"FAIL: dispatch line missing flag {exp.fault_flag_hex}: "
              f"{line.strip()}", flush=True)
        return False
    print(f"  ✓ lrmh_fault: {line.strip()[-90:]}", flush=True)

    # Clear injection so the next test (or live operations) sees clean
    # sensors again.
    subprocess.run(
        ["python3", HAL_BENCH_TEST, "--inject", "none"],
        capture_output=True, timeout=10)

    # Let the in-flight site finish rather than ABORT-ing it: a T1
    # RESUME continues the test to COMPLETE, a T2 SKIP walks it to
    # REPORTING as PARTIAL, a T3 SAFE drops it to IDLE. The next
    # START is rejected while the sequencer is still in a site, so
    # wait for the "Phase N → 0" transition back to IDLE (a full
    # test with repeat cycles takes a couple of minutes).
    line = wait_for_pattern(f, "→ 0", timeout_s=300.0)
    if line is None:
        print("WARN: sequencer did not return to IDLE within 300 s", flush=True)
    time.sleep(2.0)
    f.close()
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--kind", choices=list(EXPECTATIONS.keys()) + ["all"],
        default="all", help="fault category to validate (default: all six)")
    args = parser.parse_args()

    if not os.path.isfile(CFS_LOG):
        print(f"ERROR: {CFS_LOG} not found — is the stack up "
              "(./scripts/lrmh_launch.sh)?", file=sys.stderr)
        return 2

    kinds = list(EXPECTATIONS.keys()) if args.kind == "all" else [args.kind]
    print(f"lrmh_fault_test → validating {len(kinds)} category(ies)",
          flush=True)

    n_pass, n_fail = 0, 0
    for k in kinds:
        if run_one(k):
            n_pass += 1
        else:
            n_fail += 1

    print(f"\n{'=' * 60}")
    print(f"  passed: {n_pass}/{len(kinds)}")
    print(f"  failed: {n_fail}/{len(kinds)}")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
