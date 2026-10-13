#!/usr/bin/env bash
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
#
# lrmh_launch.sh — bring up the full local LRMH stack
#
#   1. lrmh_hal_sim      standalone HAL physics simulator (UDP :17011, 100 Hz)
#   2. core-cpu1         the cFS image with the eight LRMH apps + LC/SC/HS/DS/FM
#   3. lrmh_bridge.py    CCSDS <-> WebSocket bridge, serving the operator UI
#
# Usage:
#   scripts/lrmh_launch.sh            # bring up everything
#   scripts/lrmh_launch.sh stop       # stop all three processes
#
# Environment:
#   LRMH_CFS_ROOT   cFS tree that install.sh populated and `make install`
#                   built (default: ~/lrmh-cfs-build)
#   LRMH_WS_HOST    address the bridge serves the UI on (default 127.0.0.1;
#                   set 0.0.0.0 to reach the UI from other machines on a
#                   trusted LAN; the WebSocket has no authentication)
#   LRMH_UI_PORT    bridge HTTP + WebSocket port (default 8080)
#   LRMH_LOG_DIR    where the three process logs go (default build/logs in this repo)
#
# Prerequisites: `cmake -B build -S . -DTARGET=sim && cmake --build build`
# in this repo (for lrmh_hal_sim), and a built cFS tree.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CFS_ROOT="${LRMH_CFS_ROOT:-$HOME/lrmh-cfs-build}"
LOG_DIR="${LRMH_LOG_DIR:-$REPO_ROOT/build/logs}"
HAL_SIM="${REPO_ROOT}/build/hal/sim/lrmh_hal_sim"
CFS_BIN="${CFS_ROOT}/build/exe/cpu1/core-cpu1"
GROUND_DIR="${REPO_ROOT}/ground"
WS_HOST="${LRMH_WS_HOST:-127.0.0.1}"
UI_PORT="${LRMH_UI_PORT:-8080}"

CYN='\033[0;36m'; GRN='\033[0;32m'; AMB='\033[0;33m'; RST='\033[0m'
log()  { echo -e "${CYN}[LRMH]${RST} $*"; }
ok()   { echo -e "${GRN}[ OK ]${RST} $*"; }
warn() { echo -e "${AMB}[ !! ]${RST} $*"; }

case "${1:-up}" in

stop)
    log "Stopping LRMH stack..."
    # Match exact process names (not -f substrings) so a parent shell whose
    # command line merely mentions these names is never killed.
    pkill -x "lrmh_hal_sim" 2>/dev/null && ok "lrmh_hal_sim stopped" || warn "lrmh_hal_sim not running"
    pkill -x "core-cpu1"    2>/dev/null && ok "core-cpu1 stopped"    || warn "core-cpu1 not running"
    pkill -f '^python3 .*lrmh_bridge\.py' 2>/dev/null && ok "bridge stopped" || warn "bridge not running"
    exit 0
    ;;

up|"")
    log "Starting LRMH stack (cFS tree: ${CFS_ROOT})"

    [[ -x "$HAL_SIM" ]] || { warn "HAL sim missing at ${HAL_SIM}. Run: cmake -B build -S . -DTARGET=sim && cmake --build build"; exit 1; }
    [[ -x "$CFS_BIN" ]] || { warn "cFS image missing at ${CFS_BIN}. Run install.sh, then make SIMULATION=native MISSIONCONFIG=sample prep && make && make install in the cFS tree, or set LRMH_CFS_ROOT."; exit 1; }
    [[ -f "${GROUND_DIR}/lrmh_bridge.py" ]] || warn "bridge not found at ${GROUND_DIR}; HAL sim + cFS only"

    # POSIX message-queue depth. cFS software-bus pipes are mqueues; the
    # kernel caps unprivileged depth at /proc/sys/fs/mqueue/msg_max
    # (default 10) and OSAL silently truncates every pipe to it. At depth
    # 10 the single-shot end-of-test packets (site record, site summary)
    # are dropped at TO_LAB and the dashboard never shows a result.
    MSG_MAX_REQUIRED=1024
    if [[ -r /proc/sys/fs/mqueue/msg_max ]]; then
        MSG_MAX_CURRENT=$(< /proc/sys/fs/mqueue/msg_max)
        if (( MSG_MAX_CURRENT < MSG_MAX_REQUIRED )); then
            log "POSIX mqueue depth ${MSG_MAX_CURRENT} -> ${MSG_MAX_REQUIRED} (sudo)..."
            if sudo -n sysctl -w fs.mqueue.msg_max=${MSG_MAX_REQUIRED} >/dev/null 2>&1; then
                ok "fs.mqueue.msg_max set to ${MSG_MAX_REQUIRED}"
            else
                warn "fs.mqueue.msg_max is ${MSG_MAX_CURRENT} (need ${MSG_MAX_REQUIRED})."
                warn "The stack will run, but end-of-test packets may be dropped. Fix with:"
                warn "  sudo sysctl -w fs.mqueue.msg_max=${MSG_MAX_REQUIRED}"
                warn "or persistently:"
                warn "  echo 'fs.mqueue.msg_max=${MSG_MAX_REQUIRED}' | sudo tee /etc/sysctl.d/99-cfs.conf && sudo sysctl --system"
            fi
        else
            log "fs.mqueue.msg_max=${MSG_MAX_CURRENT} (>= ${MSG_MAX_REQUIRED})"
        fi
    fi

    pkill -x "lrmh_hal_sim" 2>/dev/null || true
    pkill -x "core-cpu1" 2>/dev/null || true
    pkill -f '^python3 .*lrmh_bridge\.py' 2>/dev/null || true
    sleep 1
    mkdir -p "$LOG_DIR"

    # DS writes site records under the image's cf/lrmh/{data,reports}.
    # A clean cFS rebuild wipes build/exe/cpu1/cf/, so re-create them.
    CFS_CF_DIR="$(dirname "$CFS_BIN")/cf"
    mkdir -p "${CFS_CF_DIR}/lrmh/data" "${CFS_CF_DIR}/lrmh/reports"

    # 1. HAL sim
    "$HAL_SIM" > $LOG_DIR/halsim.log 2>&1 &
    HAL_PID=$!
    sleep 1
    kill -0 $HAL_PID 2>/dev/null && ok "HAL sim PID $HAL_PID (UDP :17011)" || { warn "HAL sim died; see $LOG_DIR/halsim.log"; exit 1; }

    # 2. cFS image
    cd "$(dirname "$CFS_BIN")"
    ./core-cpu1 > $LOG_DIR/cfs.log 2>&1 &
    CFS_PID=$!
    sleep 3
    kill -0 $CFS_PID 2>/dev/null && ok "core-cpu1 PID $CFS_PID (CI_LAB :1234, TO_LAB :2234)" || { warn "core-cpu1 died; see $LOG_DIR/cfs.log"; kill $HAL_PID 2>/dev/null; exit 1; }

    # 2a. Enable TO_LAB output now, before the status telemetry fills the
    # per-MID queues. TO_LAB reads the destination as a plain C string.
    python3 - <<'PYEOF' || warn "TO_LAB enable failed (the bridge retries on connect)"
import socket, struct
TO_LAB_CMD_MID = 0x1880
TO_LAB_OUTPUT_ENABLE_CC = 6
payload = b"127.0.0.1".ljust(16, b"\x00")
length = 8 + len(payload) - 7
pkt = struct.pack(">HHH", TO_LAB_CMD_MID, 0xC000 | 1, length) \
    + struct.pack("BB", TO_LAB_OUTPUT_ENABLE_CC, 0) + payload
socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(pkt, ("127.0.0.1", 1234))
PYEOF
    ok "TO_LAB output enabled"

    # 2b. Enable SC RTSes 1-9. Upstream SC disables every RTS at boot;
    # without this each LC actionpoint trip is rejected as "RTS Disabled".
    python3 - <<'PYEOF' || warn "SC RTS enable failed (send SC_ENABLE_RTS_GRP_CC 1..9 by hand)"
import socket, struct
SC_CMD_MID = 0x18A9
SC_ENABLE_RTS_GRP_CC = 16
payload = struct.pack("<HH", 1, 9)
length = 8 + len(payload) - 7
pkt = struct.pack(">HHH", SC_CMD_MID, 0xC000 | 2, length) \
    + struct.pack("BB", SC_ENABLE_RTS_GRP_CC, 0) + payload
socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(pkt, ("127.0.0.1", 1234))
PYEOF
    ok "SC RTSes 1-9 enabled"

    # 3. Bridge (HTTP + WebSocket on one port)
    if [[ -f "${GROUND_DIR}/lrmh_bridge.py" ]]; then
        cd "$GROUND_DIR"
        python3 lrmh_bridge.py --serve-ui --ws-host "$WS_HOST" --ws-port "$UI_PORT" \
            --cf-dir "${CFS_CF_DIR}/lrmh" > $LOG_DIR/bridge.log 2>&1 &
        BRIDGE_PID=$!
        sleep 1
        kill -0 $BRIDGE_PID 2>/dev/null && ok "bridge PID $BRIDGE_PID (http+ws on ${WS_HOST}:${UI_PORT})" || warn "bridge died; see $LOG_DIR/bridge.log"

        URL_HOST="$WS_HOST"
        if [[ "$WS_HOST" == "0.0.0.0" ]]; then
            URL_HOST="$(hostname -I 2>/dev/null | awk '{print $1}')"; [[ -z "$URL_HOST" ]] && URL_HOST=127.0.0.1
        fi
        echo ""
        echo "  LRMH operator UI:   http://${URL_HOST}:${UI_PORT}/"
        echo ""
        echo "  Logs:  $LOG_DIR/{halsim,cfs,bridge}.log"
        echo "  CLI:   python3 ${GROUND_DIR}/lrmh_console.py"
        echo "  Test:  python3 ${GROUND_DIR}/lrmh_ground_test.py --test full"
        echo "  Stop:  scripts/lrmh_launch.sh stop"
        echo ""
    fi
    ;;

*)
    echo "Usage: $0 [up|stop]"
    exit 1
    ;;
esac
