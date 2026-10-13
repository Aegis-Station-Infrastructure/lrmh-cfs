#!/usr/bin/env python3
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
"""
LRMH HAL backend standalone test harness.

Talks UDP directly to a HAL backend (`lrmh_hal_sim` or
`lrmh_hal_bench`) — no cFS image, no bridge, no browser
involved — and walks each `LRMH_HAL_*` primitive through a
canonical scenario. Pass / fail per scenario with diagnostic
detail on failure.

Use cases:

  * Bench-integration loop: replace one `TODO(bench)`
    block in `hal/bench/lrmh_hal_bench.c`, rebuild the bench
    binary, run this against it, see the matching scenario flip
    from FAIL to OK before moving on.
  * CI-style sanity check: run with `--scenario all` against the
    sim to confirm a clean baseline before launching the full
    cFS stack.

The wire envelope is reproduced from `hal/include/lrmh_hal.h`,
`hal/sim/lrmh_hal_sim.c`, `hal/bench/lrmh_hal_bench.c`, and
`hal/client/lrmh_hal_client.c` (which all carry identical
`HalRequest_t` / `HalResponse_t` shapes). If you change the
envelope in any of those files, mirror the change here.

Usage:

  python3 scripts/hal_bench_test.py                       # all scenarios
  python3 scripts/hal_bench_test.py --scenario sensors    # one scenario
  python3 scripts/hal_bench_test.py --host 192.0.2.50   # remote bench
  python3 scripts/hal_bench_test.py --verbose             # see stale-drop logs

Default target is the sim on 127.0.0.1:17011. Start the sim with
`./build/hal/sim/lrmh_hal_sim` or the bench with
`./build-bench/hal/bench/lrmh_hal_bench` before running.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from dataclasses import dataclass

# ──────────────────────────────────────────────────────────────────
# HAL UDP envelope — must match hal/include/lrmh_hal.h + the three
# server/client implementations byte-for-byte.
# ──────────────────────────────────────────────────────────────────

HAL_MAGIC = 0x4C524D48
HAL_DEFAULT_HOST = "127.0.0.1"
HAL_DEFAULT_PORT = 17011

# HalReqType_t
REQ_INIT          = 0
REQ_READ_SENSORS  = 1
REQ_GET_HEALTH    = 2
REQ_CMD_POSITION  = 3
REQ_CMD_FORCE     = 4
REQ_RETRACT       = 5
REQ_HOLD          = 6
REQ_ESTOP         = 7
REQ_STEP_SIM      = 8
REQ_INJECT_FAULT  = 9     # sim-only; bench/flight ack as no-op

# HalInjectKind_t — fault categories the sim can force on demand
INJECT_NONE          = 0
INJECT_FORCE_SAT     = 1
INJECT_OVERCURRENT   = 2
INJECT_HARD_STOP     = 3
INJECT_THERMAL       = 4
INJECT_BUS_VOLTAGE   = 5
INJECT_FORCE_RATE    = 6

INJECT_KINDS = {
    "none":          INJECT_NONE,
    "force-sat":     INJECT_FORCE_SAT,
    "overcurrent":   INJECT_OVERCURRENT,
    "hard-stop":     INJECT_HARD_STOP,
    "thermal":       INJECT_THERMAL,
    "bus-voltage":   INJECT_BUS_VOLTAGE,
    "force-rate":    INJECT_FORCE_RATE,
}

# Status codes (LRMH_HAL_* return values from lrmh_hal.h)
HAL_OK                  = 0
HAL_ERROR               = -1
HAL_NOT_INITIALIZED     = -2
HAL_REJECTED_ESTOP      = -3
HAL_REJECTED_FAULT      = -4
HAL_REJECTED_BAD_PARAM  = -5
HAL_REJECTED_BUSY       = -6
HAL_SELFTEST_FAIL       = -7
HAL_BACKEND_TIMEOUT     = -8

STATUS_NAMES = {
     0: "OK",                -1: "ERROR",
    -2: "NOT_INITIALIZED",   -3: "REJECTED_ESTOP",
    -4: "REJECTED_FAULT",    -5: "REJECTED_BAD_PARAM",
    -6: "REJECTED_BUSY",     -7: "SELFTEST_FAIL",
    -8: "BACKEND_TIMEOUT",
}

# LRMH_HAL_State_t
STATE_COLD = 0; STATE_IDLE = 1; STATE_MOVING = 2
STATE_HOLDING = 3; STATE_FAULT = 4; STATE_ESTOP = 5

STATE_NAMES = {
    0: "COLD", 1: "IDLE", 2: "MOVING",
    3: "HOLDING", 4: "FAULT", 5: "ESTOP",
}

N_CHANNELS = 6


@dataclass
class SensorData:
    valid: bool
    channel_valid: int
    time_us: int
    force_n: float
    depth_m: float
    stroke_m: float
    current_a: float
    bus_v: float
    temp_c: float


@dataclass
class Health:
    valid: bool
    time_us: int
    state: int
    flags: int
    stroke_m: float
    last_force_n: float


# ──────────────────────────────────────────────────────────────────
# UDP client
# ──────────────────────────────────────────────────────────────────

class HalClient:
    """Speaks the HAL UDP envelope. One client per test run."""

    def __init__(self, host=HAL_DEFAULT_HOST, port=HAL_DEFAULT_PORT,
                 timeout_s=0.2, verbose=False):
        self.host, self.port = host, port
        self.timeout_s = timeout_s
        self.verbose = verbose
        self.seq_no = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _send_recv(self, req_type: int, payload: bytes = b"") -> tuple[int, bytes]:
        """Send one request, drain stale responses, return (status, payload).

        Mirrors the recv loop in hal/client/lrmh_hal_client.c — bounded
        deadline, drop responses with mismatched magic / req_type / seq_no.
        """
        if len(payload) > 256:
            raise ValueError(f"payload too large: {len(payload)} > 256")

        self.seq_no += 1
        seq = self.seq_no
        req = struct.pack("<IIII", HAL_MAGIC, seq, req_type, len(payload))
        req += payload.ljust(256, b"\x00")
        self.sock.sendto(req, (self.host, self.port))

        deadline = time.monotonic() + self.timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"no matching response for req_type={req_type} seq={seq}")
            self.sock.settimeout(remaining)
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                raise TimeoutError(
                    f"recv timeout for req_type={req_type} seq={seq}")

            if len(data) < 20:
                continue                        # runt
            magic, r_seq, r_type, status, r_paylen = struct.unpack(
                "<IIIiI", data[:20])
            if magic != HAL_MAGIC:
                continue                        # foreign
            if r_type != req_type or r_seq != seq:
                if self.verbose:
                    print(f"  drop stale: seq={r_seq} type={r_type} "
                          f"(want seq={seq} type={req_type})")
                continue                        # stale

            return status, data[20:20 + r_paylen]

    # ── primitives ────────────────────────────────────────────────

    def init(self) -> int:
        return self._send_recv(REQ_INIT)[0]

    def read_sensors(self) -> tuple[int, SensorData | None]:
        status, payload = self._send_recv(REQ_READ_SENSORS)
        if status != HAL_OK:
            return status, None
        if len(payload) < 40:
            raise ValueError(f"sensor payload short: {len(payload)} < 40")
        valid, _r3, ch_valid, _r2, t_us, fn, dm, sm, ca, bv, tc = \
            struct.unpack("<B3sHHQffffff", payload[:40])
        return status, SensorData(bool(valid), ch_valid, t_us,
                                  fn, dm, sm, ca, bv, tc)

    def get_health(self) -> tuple[int, Health | None]:
        status, payload = self._send_recv(REQ_GET_HEALTH)
        if status != HAL_OK:
            return status, None
        # LRMH_HAL_Health_t is 28 bytes: bool(1) + 3-byte reserved
        # array + 4-byte implicit gcc padding (the bool+pad prefix
        # is 4 bytes wide; uint64 time_us forces 8-byte alignment) +
        # uint64 + uint8 state + uint8 reserved + uint16 flags +
        # float stroke + float last_force.
        if len(payload) < 28:
            raise ValueError(f"health payload short: {len(payload)} < 28")
        valid, _r3, t_us, state, _r1, flags, stroke, last_f = \
            struct.unpack("<B3s4xQBBHff", payload[:28])
        return status, Health(bool(valid), t_us, state, flags,
                              stroke, last_f)

    def command_position(self, target_m: float, max_force_n: float = 600.0,
                         max_speed_mps: float = 0.01) -> int:
        payload = struct.pack("<fffI", target_m, max_force_n, max_speed_mps, 0)
        return self._send_recv(REQ_CMD_POSITION, payload)[0]

    def command_force(self, target_n: float, max_stroke_m: float = 0.35,
                      ramp_nps: float = 10.0) -> int:
        payload = struct.pack("<fffI", target_n, max_stroke_m, ramp_nps, 0)
        return self._send_recv(REQ_CMD_FORCE, payload)[0]

    def retract(self) -> int:
        return self._send_recv(REQ_RETRACT)[0]

    def hold(self) -> int:
        return self._send_recv(REQ_HOLD)[0]

    def estop(self) -> int:
        return self._send_recv(REQ_ESTOP)[0]

    def inject_fault(self, kind: int, override_value: float = 0.0,
                     duration_ms: int = 0) -> int:
        """Sim-only: force a fault category to make the LC watchpoint
        trip on the next sample. duration_ms=0 is persistent until
        cleared by inject_fault(INJECT_NONE)."""
        payload = struct.pack("<IfII", kind, override_value, duration_ms, 0)
        return self._send_recv(REQ_INJECT_FAULT, payload)[0]


# ──────────────────────────────────────────────────────────────────
# Scenario-runner helpers
# ──────────────────────────────────────────────────────────────────

passed = 0
failed = 0


def _status(s: int) -> str:
    return STATUS_NAMES.get(s, f"unknown({s})")


def _state(s: int) -> str:
    return STATE_NAMES.get(s, f"unknown({s})")


def expect_status(got: int, want: int, what: str) -> None:
    assert got == want, f"{what}: got {_status(got)}, expected {_status(want)}"


def poll_until(client: HalClient, predicate, timeout_s: float = 15.0,
               interval_s: float = 0.1, what: str = "condition") -> Health:
    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        status, h = client.get_health()
        if status != HAL_OK:
            raise AssertionError(
                f"poll: get_health returned {_status(status)}")
        last = h
        if predicate(h):
            return h
        time.sleep(interval_s)
    state = _state(last.state) if last else "?"
    stroke = f"{last.stroke_m:.4f}" if last else "?"
    raise AssertionError(
        f"timed out after {timeout_s}s waiting for {what} "
        f"(last health: state={state}, stroke={stroke}m)")


def setup_baseline(client: HalClient) -> None:
    """Init + Retract; ends with actuator at IDLE near stroke = 0."""
    expect_status(client.init(), HAL_OK, "baseline: Init")
    expect_status(client.retract(), HAL_OK, "baseline: Retract")
    poll_until(client, lambda h: h.state == STATE_IDLE,
               what="baseline retract complete")


# ──────────────────────────────────────────────────────────────────
# Scenarios — each is independent; `setup_baseline` puts the
# actuator in a known state before the meaningful work.
# ──────────────────────────────────────────────────────────────────

def scenario_init(client: HalClient) -> None:
    expect_status(client.init(), HAL_OK, "Init")


def scenario_sensors(client: HalClient) -> None:
    """100 reads at 100 Hz; aggregate validity check."""
    setup_baseline(client)
    n_reads, n_valid, n_full_mask = 100, 0, 0
    for i in range(n_reads):
        status, data = client.read_sensors()
        expect_status(status, HAL_OK, f"ReadSensors[{i}]")
        if data.valid:
            n_valid += 1
        if data.channel_valid == (1 << N_CHANNELS) - 1:
            n_full_mask += 1
        time.sleep(0.01)
    # Expect ≥ 95 of 100 valid (sim noise model can clear bits randomly)
    assert n_valid >= 95, \
        f"only {n_valid}/{n_reads} reads had data.valid=true"
    assert n_full_mask >= 80, \
        f"only {n_full_mask}/{n_reads} reads had all 6 channels fresh"


def scenario_position(client: HalClient) -> None:
    """Move from 0 → 0.05m at 0.01 m/s; verify within 5mm of target."""
    setup_baseline(client)
    target = 0.05
    expect_status(client.command_position(target, 600.0, 0.01),
                  HAL_OK, "CommandPosition(0.05m)")
    h = poll_until(client,
                   lambda h: h.state in (STATE_IDLE, STATE_HOLDING),
                   timeout_s=20.0, what="position move complete")
    assert abs(h.stroke_m - target) < 0.005, \
        f"stroke {h.stroke_m:.4f}m not within 5mm of target {target}m"


def scenario_force(client: HalClient) -> None:
    """Force command is accepted and the actuator state machine
    responds.

    The sim's force-loop convergence depends on regolith engagement
    (the sim model produces force from depth × stiffness);
    against an unloaded actuator the force never builds, so the
    harness checks dispatch + state-machine response, not loop
    convergence. Bench backends with real load cells will see the
    target reached within the ramp time.

    ADR-0010 also specifies HOLDING on completion; the sim returns
    IDLE because the 1.0 actuator's ControlMode 3 (Hold) differs
    from its force-mode value. lrmh_ctrl_app.c:309 accepts either
    as "done"; bench backends should return HOLDING per the ADR.
    """
    setup_baseline(client)
    expect_status(client.command_force(5.0, 0.35, 100.0),
                  HAL_OK, "CommandForce(5N, fast ramp)")
    # Verify the actuator left IDLE (state-machine accepted the cmd)
    h = poll_until(client,
                   lambda h: h.state in (STATE_MOVING, STATE_HOLDING),
                   timeout_s=2.0,
                   what="actuator entered MOVING/HOLDING")
    assert h.state != STATE_FAULT and h.state != STATE_ESTOP, \
        f"force cmd put actuator in {_state(h.state)}"
    # Clean up; retract is unconditional
    expect_status(client.retract(), HAL_OK, "Retract after force")
    poll_until(client, lambda h: h.state == STATE_IDLE,
               timeout_s=15.0, what="retract complete")


def scenario_retract(client: HalClient) -> None:
    """Move to 0.1m, retract, verify back at IDLE near stroke 0."""
    setup_baseline(client)
    expect_status(client.command_position(0.1, 600.0, 0.02),
                  HAL_OK, "CommandPosition(0.1m)")
    poll_until(client, lambda h: h.state in (STATE_IDLE, STATE_HOLDING),
               timeout_s=20.0, what="position-pre-retract complete")
    expect_status(client.retract(), HAL_OK, "Retract")
    h = poll_until(client, lambda h: h.state == STATE_IDLE,
                   timeout_s=20.0, what="retract complete")
    assert h.stroke_m < 0.005, \
        f"after retract, stroke={h.stroke_m:.4f}m, expected ≤ 5mm"


def scenario_hold(client: HalClient) -> None:
    """Mid-motion Hold transitions actuator to HOLDING immediately."""
    setup_baseline(client)
    # Start a slow position move, hold mid-motion
    expect_status(client.command_position(0.1, 600.0, 0.005),
                  HAL_OK, "CommandPosition(0.1m, slow)")
    time.sleep(0.5)  # let it start moving
    expect_status(client.hold(), HAL_OK, "Hold")
    time.sleep(0.5)
    status, h = client.get_health()
    expect_status(status, HAL_OK, "GetHealth post-Hold")
    assert h.state == STATE_HOLDING, \
        f"after Hold, state={_state(h.state)}, expected HOLDING"


def scenario_estop(client: HalClient) -> None:
    """EStop latches; state stays ESTOP under attempted motion;
    Init+Retract clears.

    ADR-0010 specifies that CommandPosition / CommandForce should
    return REJECTED_ESTOP while latched. The sim doesn't enforce
    this at the status-code level (the sim
    Actuator_CmdPosition has no estop guard) — instead the
    actuator simply doesn't move. Bench backends should reject
    explicitly per the ADR. The harness checks the latch holds at
    the state-machine level, which is what mission code relies on.
    """
    setup_baseline(client)
    expect_status(client.estop(), HAL_OK, "EStop")
    time.sleep(0.2)

    status, h = client.get_health()
    expect_status(status, HAL_OK, "GetHealth post-EStop")
    assert h.state == STATE_ESTOP, \
        f"after EStop, state={_state(h.state)}, expected ESTOP"

    # Attempt motion; state must stay ESTOP regardless of how the
    # backend signals rejection (REJECTED_ESTOP per ADR-0010, or
    # silent ignore as the sim does).
    client.command_position(0.05)
    time.sleep(0.2)
    status, h = client.get_health()
    expect_status(status, HAL_OK, "GetHealth after CmdPos in ESTOP")
    assert h.state == STATE_ESTOP, \
        f"CmdPos in ESTOP changed state to {_state(h.state)}"

    # Init + Retract clears the latch
    expect_status(client.init(), HAL_OK, "Init clears ESTOP latch")
    expect_status(client.retract(), HAL_OK, "Retract from ESTOP-cleared")
    poll_until(client, lambda h: h.state == STATE_IDLE,
               what="retract after ESTOP clear")


def scenario_state_machine(client: HalClient) -> None:
    """Canonical lifecycle: Init → Pos → Retract → Force → Hold → Retract → EStop → Init → Retract."""
    expect_status(client.init(), HAL_OK, "SM: Init")

    expect_status(client.command_position(0.05, 600.0, 0.02),
                  HAL_OK, "SM: CommandPosition")
    poll_until(client, lambda h: h.state in (STATE_IDLE, STATE_HOLDING),
               what="SM: position done")

    expect_status(client.retract(), HAL_OK, "SM: Retract 1")
    poll_until(client, lambda h: h.state == STATE_IDLE,
               what="SM: retract 1 done")

    expect_status(client.command_force(5.0, 0.35, 100.0),
                  HAL_OK, "SM: CommandForce")
    # Sim force-loop convergence is regolith-dependent (see
    # scenario_force); just confirm the actuator state machine
    # accepted the command.
    poll_until(client,
               lambda h: h.state in (STATE_MOVING, STATE_HOLDING),
               timeout_s=2.0, what="SM: force-cmd accepted")

    expect_status(client.hold(), HAL_OK, "SM: Hold")
    expect_status(client.retract(), HAL_OK, "SM: Retract 2")
    poll_until(client, lambda h: h.state == STATE_IDLE,
               what="SM: retract 2 done")

    expect_status(client.estop(), HAL_OK, "SM: EStop")
    expect_status(client.init(), HAL_OK, "SM: Init clears")
    expect_status(client.retract(), HAL_OK, "SM: Retract 3")


SCENARIOS = {
    "init":          scenario_init,
    "sensors":       scenario_sensors,
    "position":      scenario_position,
    "force":         scenario_force,
    "retract":       scenario_retract,
    "hold":          scenario_hold,
    "estop":         scenario_estop,
    "state-machine": scenario_state_machine,
}


def run(name: str, fn, *args) -> None:
    global passed, failed
    print(f"RUN  {name}", flush=True)
    try:
        fn(*args)
    except AssertionError as e:
        failed += 1
        print(f"FAIL {name}: {e}", file=sys.stderr, flush=True)
        return
    except TimeoutError as e:
        failed += 1
        print(f"FAIL {name}: timeout: {e}", file=sys.stderr, flush=True)
        return
    except Exception as e:
        failed += 1
        print(f"FAIL {name}: {type(e).__name__}: {e}",
              file=sys.stderr, flush=True)
        return
    passed += 1
    print(f"OK   {name}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=HAL_DEFAULT_HOST,
                        help=f"backend host (default: {HAL_DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=HAL_DEFAULT_PORT,
                        help=f"backend port (default: {HAL_DEFAULT_PORT})")
    parser.add_argument("--scenario",
                        choices=["all"] + list(SCENARIOS.keys()),
                        default="all",
                        help="scenario to run (default: all)")
    parser.add_argument("--timeout-s", type=float, default=0.2,
                        help="per-request UDP timeout (default: 0.2s)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="log dropped stale responses")
    parser.add_argument("--inject", choices=list(INJECT_KINDS.keys()),
                        default=None,
                        help="one-shot: send INJECT_FAULT and exit "
                             "(sim-only; bench acks as no-op)")
    parser.add_argument("--inject-value", type=float, default=0.0,
                        help="explicit override value; 0 = past-threshold default")
    parser.add_argument("--inject-duration-ms", type=int, default=0,
                        help="duration in ms; 0 = persistent until --inject none")
    args = parser.parse_args()

    client = HalClient(host=args.host, port=args.port,
                       timeout_s=args.timeout_s, verbose=args.verbose)

    if args.inject is not None:
        kind = INJECT_KINDS[args.inject]
        status = client.inject_fault(kind, args.inject_value,
                                     args.inject_duration_ms)
        print(f"INJECT_FAULT kind={args.inject} ({kind}) "
              f"value={args.inject_value} duration_ms={args.inject_duration_ms} "
              f"→ {_status(status)}")
        return 0 if status == HAL_OK else 1

    print(f"hal_bench_test → {args.host}:{args.port}", flush=True)
    print("=" * 60, flush=True)

    to_run = (list(SCENARIOS.keys())
              if args.scenario == "all" else [args.scenario])
    for name in to_run:
        run(name, SCENARIOS[name], client)

    print("=" * 60)
    print(f"  passed: {passed}")
    print(f"  failed: {failed}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
