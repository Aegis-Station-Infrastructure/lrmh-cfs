#!/usr/bin/env bash
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
#
# lrmh_boot_check.sh — boot core-cpu1 from a built cFS tree and confirm
# that all eight LRMH apps report initialised on the EVS console.
#
#   scripts/lrmh_boot_check.sh <cfs>/build/exe/cpu1 [timeout_s]
#
# If the standalone HAL simulator has been built (build/hal/sim/
# lrmh_hal_sim) it is started first so the apps have a backend to talk
# to; otherwise the apps still initialise and report HAL timeouts.
# Used by CI; harmless to run by hand.

set -uo pipefail
EXE="${1:?usage: lrmh_boot_check.sh <path/to/build/exe/cpu1> [timeout_s]}"
TIMEOUT="${2:-40}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="${LRMH_BOOT_LOG:-$(mktemp)}"
HAL_SIM="${HERE}/build/hal/sim/lrmh_hal_sim"

APPS=(
    "LRMH_CMD: initialized"
    "LRMH_SEQ: initialized"
    "LRMH_CTRL: initialized"
    "LRMH_SENSE: initialized"
    "LRMH_PROC: initialized"
    "LRMH_REPORT: initialized"
    "LRMH_HK: initialized"
    "LRMH_FAULT: thin orchestrator initialized"
)

[ -x "$EXE/core-cpu1" ] || { echo "core-cpu1 not found in $EXE"; exit 2; }

HAL_PID=""
if [ -x "$HAL_SIM" ]; then
    "$HAL_SIM" > /dev/null 2>&1 &
    HAL_PID=$!
fi

mkdir -p "$EXE/cf/lrmh/data" "$EXE/cf/lrmh/reports"
cd "$EXE"
./core-cpu1 > "$LOG" 2>&1 &
PID=$!
trap 'kill $PID 2>/dev/null; wait $PID 2>/dev/null; [ -n "$HAL_PID" ] && kill $HAL_PID 2>/dev/null' EXIT

ok=0
for ((i=0; i<TIMEOUT; i++)); do
    sleep 1
    ok=0
    for a in "${APPS[@]}"; do
        grep -qF "$a" "$LOG" && ok=$((ok+1))
    done
    [ $ok -eq ${#APPS[@]} ] && break
    kill -0 $PID 2>/dev/null || break
done

echo "--- LRMH init events ($ok/${#APPS[@]}) after ${i}s, log: $LOG"
grep -E 'LRMH_[A-Z]+: .*initialized' "$LOG" || true
echo "--- support apps:"
grep -E '^(EVS Port1 )?.*\b(LC|SC|HS|DS|FM)\b.*[Ii]nitiali' "$LOG" | head -5 || true
if [ $ok -ne ${#APPS[@]} ]; then
    echo "--- FAILED; last 40 log lines:"; tail -40 "$LOG"; exit 1
fi
echo "--- boot check OK"
