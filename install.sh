#!/usr/bin/env bash
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
#
# install.sh — Install the LRMH apps into an existing NASA cFS v7.0.0 tree
#
# Usage:
#   cd lrmh-cfs
#   ./install.sh ~/lrmh-cfs-build
#
# Copies the eight LRMH mission apps, the shared message/HAL sources
# (lrmh_defs) and the five NASA support apps (DS, FM, LC, SC, HS, from
# the vendor/ submodules) into the cFS tree, creates sample_defs from
# the cFE template if needed, then patches:
#
#   sample_defs/targets.cmake                 — adds the apps to the APPLIST
#   sample_defs/cpu1_cfe_es_startup.scr       — loads the apps at boot
#   sample_defs/default_osconfig.cmake        — queue depth, path/filename length
#   apps/to_lab/fsw/tables/to_lab_sub.c       — TO_LAB subscriptions
#   apps/sch_lab/fsw/tables/sch_lab_table.c   — SCH_LAB wakeups
#   apps/lc, apps/sc, apps/hs, apps/ds        — LRMH FDIR / storage tables
#
# Idempotent: every step checks for its own sentinel before patching, so
# re-running against an already-installed tree is a no-op.
#
# Linux only (GNU sed and bash). The native cFS build this targets is
# Linux as well.
#
# After running this script:
#   cd <cfs_root>
#   make SIMULATION=native MISSIONCONFIG=sample prep
#   make -j$(nproc)
#   make install
#
# The image lands at <cfs_root>/build/exe/cpu1/core-cpu1.

set -euo pipefail

if [[ -z "${1:-}" ]]; then
    echo "Usage: ./install.sh <cfs_root>"
    echo "  Example: ./install.sh ~/lrmh-cfs-build"
    exit 1
fi

CFS_DIR="$(cd "$1" && pwd)"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RED='\033[0;31m'; GRN='\033[0;32m'; CYN='\033[0;36m'; AMB='\033[0;33m'; RST='\033[0m'
log()  { echo -e "${CYN}[LRMH]${RST} $*"; }
ok()   { echo -e "${GRN}[ OK ]${RST} $*"; }
warn() { echo -e "${AMB}[ !! ]${RST} $*"; }
err()  { echo -e "${RED}[ERR ]${RST} $*" >&2; }

[[ -d "${CFS_DIR}/cfe" ]] || { err "${CFS_DIR} does not look like a cFS tree (no cfe/ directory)"; exit 1; }
[[ -d "${CFS_DIR}/apps/to_lab" && -d "${CFS_DIR}/apps/sch_lab" ]] || {
    err "${CFS_DIR}/apps is missing to_lab or sch_lab. Clone cFS with --recurse-submodules."; exit 1; }

# LRMH is developed and verified against the cFS v7.0.0 (Draco) bundle
# tag. Other revisions may work, but the patches below are only tested
# there.
CFS_TAG="$(git -C "$CFS_DIR" describe --tags --always 2>/dev/null || echo unknown)"
if [[ "$CFS_TAG" != "v7.0.0" ]]; then
    warn "${CFS_DIR} is at '${CFS_TAG}', not the cFS v7.0.0 tag."
    warn "LRMH is verified on v7.0.0 only; continuing anyway."
fi

# cFS software-bus pipes are POSIX mqueues. The kernel default depth
# (10) clamps every pipe and drops the end-of-test packets. See the
# Build & Install Guide.
MSG_MAX=$(cat /proc/sys/fs/mqueue/msg_max 2>/dev/null || echo 0)
if (( MSG_MAX < 1024 )); then
    warn "fs.mqueue.msg_max is ${MSG_MAX}; cFS pipes clamp to this depth."
    warn "Before running the image:  sudo sysctl fs.mqueue.msg_max=1024"
fi

log "Installing LRMH into ${CFS_DIR}"

LRMH_APPS=(lrmh_cmd lrmh_seq lrmh_ctrl lrmh_sense lrmh_proc lrmh_report lrmh_hk lrmh_fault)

# ── 1. Clean any previous LRMH install ──────────────────────
for app in "${LRMH_APPS[@]}"; do
    rm -rf "${CFS_DIR}/apps/${app}"
done
rm -rf "${CFS_DIR}/lrmh_defs"
ok "Previous LRMH state cleared"

# ── 2. Mission apps ─────────────────────────────────────────
for app in "${LRMH_APPS[@]}"; do
    src="${REPO_ROOT}/apps/${app}"
    dst="${CFS_DIR}/apps/${app}"
    [[ -d "$src" ]] || { err "missing source: $src"; exit 1; }
    mkdir -p "$dst"
    cp -r "$src/CMakeLists.txt" "$dst/"
    cp -r "$src/fsw" "$dst/"
    log "  $app"
done
ok "Mission apps installed"

# ── 3. Shared sources: lrmh_defs ────────────────────────────
##
## cFS apps are loadable shared objects and cannot link against each
## other, so the sequence-token source and the HAL UDP client are
## compiled into every app via target_sources() in each app's
## CMakeLists (referenced as ${MISSION_SOURCE_DIR}/lrmh_defs/...).
## The LC watchpoint tables also need the per-app message headers for
## field offsets, so those are copied here too.
mkdir -p "${CFS_DIR}/lrmh_defs"
cp "${REPO_ROOT}/msg/lrmh_msgids.h"                        "${CFS_DIR}/lrmh_defs/"
cp "${REPO_ROOT}/msg/lrmh_msg.h"                           "${CFS_DIR}/lrmh_defs/"
cp "${REPO_ROOT}/msg/lrmh_seq_token.h"                     "${CFS_DIR}/lrmh_defs/"
cp "${REPO_ROOT}/msg/lrmh_seq_token.c"                     "${CFS_DIR}/lrmh_defs/"
cp "${REPO_ROOT}/cfs_defs/lrmh_platform_cfg.h"             "${CFS_DIR}/lrmh_defs/"
cp "${REPO_ROOT}/apps/lrmh_sense/fsw/inc/lrmh_sense_msg.h" "${CFS_DIR}/lrmh_defs/"
cp "${REPO_ROOT}/apps/lrmh_fault/fsw/inc/lrmh_fault_msg.h" "${CFS_DIR}/lrmh_defs/"
cp "${REPO_ROOT}/hal/include/lrmh_hal.h"                   "${CFS_DIR}/lrmh_defs/"
cp "${REPO_ROOT}/hal/client/lrmh_hal_client.c"             "${CFS_DIR}/lrmh_defs/"
ok "lrmh_defs installed"

# ── 4. sample_defs from the cFE template ────────────────────
SDEFS="${CFS_DIR}/sample_defs"
if [[ ! -d "$SDEFS" ]]; then
    cp -r "${CFS_DIR}/cfe/cmake/sample_defs" "$SDEFS"
    ok "sample_defs created from cfe/cmake/sample_defs"
fi

# ── 5. targets.cmake ────────────────────────────────────────
if ! grep -q "lrmh_cmd" "${SDEFS}/targets.cmake" 2>/dev/null; then
    cat >> "${SDEFS}/targets.cmake" << 'EOF'

# ── LRMH (added by lrmh-cfs/install.sh) ──
list(APPEND MISSION_GLOBAL_APPLIST
    ds fm lc sc hs
    lrmh_cmd lrmh_seq lrmh_ctrl lrmh_sense
    lrmh_proc lrmh_report lrmh_hk lrmh_fault
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lrmh_defs)
EOF
    ok "targets.cmake patched"
else
    log "targets.cmake already patched"
fi

# ── 6. NASA support apps (DS, FM, LC, SC, HS) ───────────────
##
## These are separate upstream NASA repos, not part of the cFS bundle.
## They are git submodules under vendor/cfs-support-apps/, pinned to
## known SHAs. Fail loudly if they are not checked out: an image
## without LC/SC/HS has no FDIR detection or recovery path.
##
##   DS  binary site record + human report capture
##   FM  file manager (ground fetch of DS-written files)
##   LC  limit checker (watchpoints on threshold violations)
##   SC  stored commands (RTS recovery sequences)
##   HS  health & safety (app restart policy)
install_support_app() {
    local appname="$1"
    local src="${REPO_ROOT}/vendor/cfs-support-apps/${appname}"
    if [[ ! -d "$src" ]] || [[ -z "$(ls -A "$src" 2>/dev/null)" ]]; then
        err "${appname}: vendor submodule not checked out at ${src}"
        err "  run: git submodule update --init --recursive"
        err "  (or clone this repo with --recurse-submodules)"
        exit 1
    fi
    rm -rf "${CFS_DIR}/apps/${appname}"
    cp -r "$src" "${CFS_DIR}/apps/${appname}"
    rm -rf "${CFS_DIR}/apps/${appname}/unit-test"
    rm -rf "${CFS_DIR}/apps/${appname}/.git"
    sed -i '/add_subdirectory(unit-test)/d;/if(ENABLE_UNIT_TESTS)/,/endif()/d' \
        "${CFS_DIR}/apps/${appname}/CMakeLists.txt"
}

install_support_app ds
cp "${REPO_ROOT}/tables/ds_filter_table.c" "${CFS_DIR}/apps/ds/fsw/tables/ds_filter_tbl.c"
cp "${REPO_ROOT}/tables/ds_file_table.c"   "${CFS_DIR}/apps/ds/fsw/tables/ds_file_tbl.c"
ok "DS installed with LRMH filter/file tables"

install_support_app fm
ok "FM installed (upstream default monitor table)"

install_support_app sc
# SC's upstream HK_TLM topic 0xAA resolves to 0x08AA, which is
# LRMH_REPORT_SITE_RECORD_MID: an SC housekeeping packet would be
# captured by DS as a site record. Move it to 0xC8 (0x08C8).
SC_TIDS="${CFS_DIR}/apps/sc/fsw/inc/sc_topicids.h"
if [[ -f "$SC_TIDS" ]] && grep -q "DEFAULT_CFE_MISSION_SC_HK_TLM_TOPICID\s*0xAA" "$SC_TIDS"; then
    sed -i 's/DEFAULT_CFE_MISSION_SC_HK_TLM_TOPICID\s*0xAA/DEFAULT_CFE_MISSION_SC_HK_TLM_TOPICID 0xC8/' "$SC_TIDS"
    log "  SC HK TLM topic 0xAA -> 0xC8 (0x08AA is LRMH_REPORT_SITE_RECORD_MID)"
fi
# LRMH ships nine RTSes plus a placeholder ATS:
#   001         NOTIFY FORCE_SATURATION
#   002 003 004 RESUME_PHASE / SKIP_SITE / RETRACT_AND_SAFE fallbacks
#   005..009    NOTIFY OVERCURRENT / HARD_STOP / THERMAL / BUS_VOLTAGE / FORCE_RATE
for n in 1 2 3 4 5 6 7 8 9; do
    cp "${REPO_ROOT}/tables/sc/sc_rts00${n}.c" "${CFS_DIR}/apps/sc/fsw/tables/sc_rts00${n}.c"
done
cp "${REPO_ROOT}/tables/sc/sc_ats1.c" "${CFS_DIR}/apps/sc/fsw/tables/sc_ats1.c"

# RTS pool: upstream default is 4; LRMH needs 9.
SC_CFG="${CFS_DIR}/apps/sc/fsw/inc/sc_interface_cfg.h"
if [[ -f "$SC_CFG" ]] && grep -q "DEFAULT_SC_INTERFACE_NUMBER_OF_RTS 4$" "$SC_CFG"; then
    sed -i 's/DEFAULT_SC_INTERFACE_NUMBER_OF_RTS 4$/DEFAULT_SC_INTERFACE_NUMBER_OF_RTS 16/' "$SC_CFG"
    log "  SC NUMBER_OF_RTS 4 -> 16"
fi
# Header-update mode: SC computes the CCSDS checksum of each embedded
# RTS command at send time. The default (false) validates pre-computed
# checksums and rejects table sources that carry a zero checksum byte.
SC_INT_CFG="${CFS_DIR}/apps/sc/fsw/inc/sc_internal_cfg.h"
if [[ -f "$SC_INT_CFG" ]] && grep -q "DEFAULT_SC_INTERNAL_PLATFORM_ENABLE_HEADER_UPDATE false" "$SC_INT_CFG"; then
    sed -i 's/DEFAULT_SC_INTERNAL_PLATFORM_ENABLE_HEADER_UPDATE false/DEFAULT_SC_INTERNAL_PLATFORM_ENABLE_HEADER_UPDATE true/' "$SC_INT_CFG"
    log "  SC header update enabled"
fi
# Compile RTS 005-009 into .tbl files.
SC_CMAKE="${CFS_DIR}/apps/sc/CMakeLists.txt"
if [[ -f "$SC_CMAKE" ]] && ! grep -q "sc_rts005" "$SC_CMAKE"; then
    sed -i '/fsw\/tables\/sc_rts004.c/a\
  fsw/tables/sc_rts005.c\
  fsw/tables/sc_rts006.c\
  fsw/tables/sc_rts007.c\
  fsw/tables/sc_rts008.c\
  fsw/tables/sc_rts009.c' "$SC_CMAKE"
    log "  SC CMakeLists APP_TABLE_FILES extended"
fi
ok "SC installed with LRMH RTS tables"

install_support_app hs
cp "${REPO_ROOT}/tables/hs_table.c" "${CFS_DIR}/apps/hs/fsw/tables/hs_amt.c"
# HS's upstream WAKEUP topic 0xB0 resolves to 0x18B0, which is
# LRMH_SEQ_TO_CTRL_MID: every sequencer actuator command would be
# delivered to HS as a wakeup. Move it to 0xC7 (0x18C7), just above
# LC's relocated topics; the SCH_LAB patch below drives it at 1 Hz.
HS_TIDS="${CFS_DIR}/apps/hs/fsw/inc/hs_topicids.h"
if [[ -f "$HS_TIDS" ]] && grep -q "DEFAULT_CFE_MISSION_HS_WAKEUP_TOPICID\s*0xB0" "$HS_TIDS"; then
    sed -i 's/DEFAULT_CFE_MISSION_HS_WAKEUP_TOPICID\s*0xB0/DEFAULT_CFE_MISSION_HS_WAKEUP_TOPICID 0xC7/' "$HS_TIDS"
    log "  HS wakeup topic 0xB0 -> 0xC7 (0x18B0 is LRMH_SEQ_TO_CTRL_MID)"
fi
ok "HS installed with LRMH AMT (upstream default EMT/MAT/XCT)"

install_support_app lc
# LC powers on DISABLED upstream; LRMH treats FDIR as always-on.
LC_CFG="${CFS_DIR}/apps/lc/fsw/inc/lc_internal_cfg.h"
if [[ -f "$LC_CFG" ]] && grep -q "DEFAULT_LC_STATE_POWER_ON_RESET LC_STATE_DISABLED" "$LC_CFG"; then
    sed -i 's/DEFAULT_LC_STATE_POWER_ON_RESET LC_STATE_DISABLED/DEFAULT_LC_STATE_POWER_ON_RESET LC_STATE_ACTIVE/' "$LC_CFG"
    log "  LC power-on state DISABLED -> ACTIVE"
fi
# LC's upstream CMD/SEND_HK/SAMPLE_AP topic IDs (0xA4/A5/A6) collide
# with LRMH_SENSE/PROC/REPORT_CMD_MID. Move them to 0xC3/C4/C5, just
# above LRMH's wakeup block.
LC_TIDS="${CFS_DIR}/apps/lc/fsw/inc/lc_topicids.h"
if [[ -f "$LC_TIDS" ]] && grep -q "DEFAULT_LC_MISSION_CMD_TOPICID\s*0xA4" "$LC_TIDS"; then
    sed -i \
        -e 's/DEFAULT_LC_MISSION_CMD_TOPICID\s*0xA4/DEFAULT_LC_MISSION_CMD_TOPICID       0xC3/' \
        -e 's/DEFAULT_LC_MISSION_SEND_HK_TOPICID\s*0xA5/DEFAULT_LC_MISSION_SEND_HK_TOPICID   0xC4/' \
        -e 's/DEFAULT_LC_MISSION_SAMPLE_AP_TOPICID\s*0xA6/DEFAULT_LC_MISSION_SAMPLE_AP_TOPICID 0xC5/' \
        "$LC_TIDS"
    log "  LC topic IDs 0xA4-A6 -> 0xC3-C5"
fi

# LC watchpoint / actionpoint tables. The LRMH sources declare only
# the slots they use; the LC validator rejects zero-initialised
# designated-initializer holes, so pad the arrays with explicit
# NOT_USED entries up to LC_MAX_WATCHPOINTS / LC_MAX_ACTIONPOINTS.
WDT="${CFS_DIR}/apps/lc/fsw/tables/lc_def_wdt.c"
ADT="${CFS_DIR}/apps/lc/fsw/tables/lc_def_adt.c"
cp "${REPO_ROOT}/tables/lc_watchpoints.c"  "$WDT"
cp "${REPO_ROOT}/tables/lc_actionpoints.c" "$ADT"
python3 - "$WDT" "$ADT" <<'PYEOF'
import re, sys

WDT, ADT = sys.argv[1], sys.argv[2]
WDT_MAX = 176
ADT_MAX = 176

def count_designated(src):
    return len(re.findall(r"\[LRMH_(?:WP|AP)_[A-Z_]+\]\s*=\s*(?:\{|LRMH_)", src))

with open(WDT) as f:
    src = f.read()
n_used = count_designated(src)
n_pad = WDT_MAX - n_used
print(f"  WDT: {n_used} used, padding {n_pad} unused")
pad_entry = (
    "    {\n"
    "        .DataType                   = LC_DATA_WATCH_NOT_USED,\n"
    "        .OperatorID                 = LC_OPER_NONE,\n"
    "        .MessageID                  = CFE_SB_MSGID_RESERVED,\n"
    "        .WatchpointOffset           = 0,\n"
    "        .BitMask                    = LC_BITMASK_NONE,\n"
    "        .CustomFuncArgument         = 0,\n"
    "        .ResultAgeWhenStale         = 0,\n"
    "        .ComparisonValue.Unsigned32 = 0,\n"
    "    }"
)
src = src.replace("};", ",\n" + ",\n".join([pad_entry] * n_pad) + "\n};", 1)
with open(WDT, "w") as f:
    f.write(src)

with open(ADT) as f:
    src = f.read()
n_used = count_designated(src)
n_pad = ADT_MAX - n_used
print(f"  ADT: {n_used} used, padding {n_pad} unused")
pad_entry = (
    "    {\n"
    "        .DefaultState        = LC_APSTATE_NOT_USED,\n"
    "        .MaxPassiveEvents    = 0,\n"
    "        .MaxPassFailEvents   = 0,\n"
    "        .MaxFailPassEvents   = 0,\n"
    "        .RTSId               = 0,\n"
    "        .MaxFailsBeforeRTS   = 0,\n"
    "        .RPNEquation         = { LC_RPN_EQUAL },\n"
    "        .EventType           = CFE_EVS_EventType_INFORMATION,\n"
    "        .EventID             = 0,\n"
    "        .EventText           = \"\"\n"
    "    }"
)
src = src.replace("};", ",\n" + ",\n".join([pad_entry] * n_pad) + "\n};", 1)
with open(ADT, "w") as f:
    f.write(src)
PYEOF
ok "LC installed with LRMH watchpoint/actionpoint tables"

# ── 7. Startup script ───────────────────────────────────────
SCR="${SDEFS}/cpu1_cfe_es_startup.scr"
if [[ -f "$SCR" ]] && ! grep -q "LRMH_CMD_AppMain" "$SCR"; then
    sed -i '/^!/d' "$SCR"
    sed -i -e :a -e '/^\n*$/{$d;N;ba' -e '}' "$SCR"
    cat >> "$SCR" << 'EOF'
CFE_APP, ds,         DS_AppMain,         DS,           95,   16384, 0x0, 0;
CFE_APP, fm,         FM_AppMain,         FM,           96,   16384, 0x0, 0;
CFE_APP, lc,         LC_AppMain,         LC,           97,   16384, 0x0, 0;
CFE_APP, sc,         SC_AppMain,         SC,           98,   16384, 0x0, 0;
CFE_APP, hs,         HS_AppMain,         HS,           99,   16384, 0x0, 0;
CFE_APP, lrmh_cmd,    LRMH_CMD_AppMain,    LRMH_CMD,    100, 16384, 0x0, 0;
CFE_APP, lrmh_seq,    LRMH_SEQ_AppMain,    LRMH_SEQ,    100, 16384, 0x0, 0;
CFE_APP, lrmh_ctrl,   LRMH_CTRL_AppMain,   LRMH_CTRL,   100, 16384, 0x0, 0;
CFE_APP, lrmh_sense,  LRMH_SENSE_AppMain,  LRMH_SENSE,  100, 16384, 0x0, 0;
CFE_APP, lrmh_proc,   LRMH_PROC_AppMain,   LRMH_PROC,   100, 16384, 0x0, 0;
CFE_APP, lrmh_report, LRMH_REPORT_AppMain, LRMH_REPORT, 100, 16384, 0x0, 0;
CFE_APP, lrmh_hk,     LRMH_HK_AppMain,     LRMH_HK,     100, 16384, 0x0, 0;
CFE_APP, lrmh_fault,  LRMH_FAULT_AppMain,  LRMH_FAULT,  100, 16384, 0x0, 0;
!
EOF
    ok "Startup script patched"
else
    log "Startup script already patched"
fi

# ── 8. OSAL configuration ───────────────────────────────────
##
## Queue depth: the LRMH apps emit ~200 status packets/s during a test;
## the default 50-deep TO_LAB pipe overflows within 250 ms and the
## single-shot end-of-test packets are dropped. 1024 is a ceiling; on
## Linux the effective depth is min(this, /proc/sys/fs/mqueue/msg_max).
## Path length: the native PSP maps /cf/... to an absolute host path.
## Filename length: DS filenames like lrmh_site_<13 digits>.bin are
## 27 characters, over the 20-byte default.
OSCFG="${SDEFS}/default_osconfig.cmake"
if [[ -f "$OSCFG" ]] && ! grep -q "OSAL_CONFIG_QUEUE_MAX_DEPTH" "$OSCFG"; then
    cat >> "$OSCFG" <<'EOF'

# LRMH overrides (added by lrmh-cfs/install.sh)
set(OSAL_CONFIG_QUEUE_MAX_DEPTH 1024 CACHE STRING "Maximum depth of message queue" FORCE)
set(OSAL_CONFIG_MAX_PATH_LEN     256 CACHE STRING "Maximum FS path length" FORCE)
set(OSAL_CONFIG_MAX_FILE_NAME     64 CACHE STRING "Maximum filename length" FORCE)
EOF
    ok "OSAL queue depth / path length / filename length patched"
else
    log "OSAL config already patched"
fi

# ── 9. TO_LAB subscriptions ─────────────────────────────────
##
## Inserted before the {CFE_SB_MSGID_RESERVED, {0, 0}, 0} terminator,
## anchored on the entry shape so the comment above it cannot match.
## Any pre-existing 0x08A_ lines are stripped first so re-runs are
## idempotent. EVS long events are deliberately not subscribed: the
## boot-time burst overflows the pipe and the dashboard synthesises
## its own events from telemetry transitions.
TOLAB="${CFS_DIR}/apps/to_lab/fsw/tables/to_lab_sub.c"
if [[ -f "$TOLAB" ]]; then
    python3 - "$TOLAB" <<'PYEOF'
import re, sys
path = sys.argv[1]
with open(path) as f:
    c = f.read()
c = re.sub(r'^[ \t]*\{CFE_SB_MSGID_WRAP_VALUE\(0x08A[0-9A-F]\),.*$\n', '', c, flags=re.MULTILINE)
c = re.sub(r'^[ \t]*\{CFE_SB_MSGID_WRAP_VALUE\(CFE_EVS_LONG_EVENT_MSG_MID\),'
           r'\s*\{0,\s*0\},\s*250\},.*$\n', '', c, flags=re.MULTILINE)
m = re.compile(r'^(?P<indent>[ \t]*)\{CFE_SB_MSGID_RESERVED,\s*\{0,\s*0\},\s*0\}',
               re.MULTILINE).search(c)
if m is None:
    print("to_lab_sub.c: end-of-table entry not found; cannot patch", file=sys.stderr)
    sys.exit(1)
indent = m.group("indent")
mids = [
    ("0x08A0", "LRMH_HK_TLM_MID            "),
    ("0x08A1", "LRMH_SEQ_STATUS_MID        "),
    ("0x08A2", "LRMH_CTRL_STATUS_MID       "),
    ("0x08A3", "LRMH_SENSE_TLM_MID         "),
    ("0x08A4", "LRMH_PROC_TLM_MID          "),
    ("0x08A6", "LRMH_FAULT_TLM_MID         "),
    ("0x08AA", "LRMH_REPORT_SITE_RECORD_MID"),
    ("0x08AB", "LRMH_REPORT_SITE_HUMAN_MID "),
    ("0x08AC", "LRMH_CMD_STATUS_MID        "),
]
block = "\n".join(f"{indent}{{CFE_SB_MSGID_WRAP_VALUE({mid}), {{0, 0}}, 50}},  /* {name} */"
                  for mid, name in mids) + "\n"
c = c[:m.start()] + block + c[m.start():]
with open(path, "w") as f:
    f.write(c)
PYEOF
    ok "TO_LAB subscriptions patched"
else
    err "TO_LAB table not found at $TOLAB"; exit 1
fi

# ── 10. SCH_LAB schedule ────────────────────────────────────
SCHLAB="${CFS_DIR}/apps/sch_lab/fsw/tables/sch_lab_table.c"
if [[ -f "$SCHLAB" ]] && ! grep -q "LRMH_SEND_HK_MID" "$SCHLAB"; then
    python3 - "$SCHLAB" <<'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    c = f.read()
c = c.replace(
    '{CFE_SB_MSGID_RESERVED, 0, 0},',
    '''{CFE_SB_MSGID_WRAP_VALUE(0x18A1), 100, 0},   /* LRMH_SEND_HK_MID @ 1 Hz (tick rate is 100 Hz) */
        {CFE_SB_MSGID_WRAP_VALUE(0x18C0), 100, 0},   /* LRMH_SEQ_WAKEUP_MID @ 1 Hz; the sequencer paces itself at 10 Hz on its pipe timeout */
        {CFE_SB_MSGID_WRAP_VALUE(0x18C1),  10, 0},   /* LRMH_SENSE_WAKEUP_MID @ 10 Hz; acquisition paces itself at 100 Hz on its pipe timeout */
        {CFE_SB_MSGID_WRAP_VALUE(0x18C2), 100, 0},   /* LRMH_FAULT_WAKEUP_MID @ 1 Hz; the polled path paces itself at 10 Hz on its pipe timeout */
        /* LC_SAMPLE_AP_MID @ 10 Hz, payload {Start=0xFFFF, End=0xFFFF, UpdateAge=1}:
         * sample all enabled actionpoints. 0x18C5 is LC's relocated SAMPLE_AP topic. */
        {CFE_SB_MSGID_WRAP_VALUE(0x18C5), 10, 0, 8, {0xFFFF, 0xFFFF, 1, 0}},
        /* SC_WAKEUP_MID @ 10 Hz: drives RTS/ATS command advancement. */
        {CFE_SB_MSGID_WRAP_VALUE(0x18AB), 10, 0, 0, {0}},
        /* HS_WAKEUP_MID @ 1 Hz on its relocated topic 0xC7 (HS also self-paces on its pipe timeout). */
        {CFE_SB_MSGID_WRAP_VALUE(0x18C7), 100, 0, 0, {0}},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_ES_SEND_HK_MID), 100, 0},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_EVS_SEND_HK_MID), 100, 0},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_SB_SEND_HK_MID), 100, 0},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_TIME_SEND_HK_MID), 100, 0},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_TBL_SEND_HK_MID), 100, 0},
        {CFE_SB_MSGID_RESERVED, 0, 0},''', 1)
with open(path, 'w') as f:
    f.write(c)
PYEOF
    ok "SCH_LAB schedule patched"
else
    log "SCH_LAB schedule already patched"
fi

# ── 11. Top-level Makefile ──────────────────────────────────
if [[ ! -f "${CFS_DIR}/Makefile" ]] && [[ -f "${CFS_DIR}/cfe/cmake/Makefile.sample" ]]; then
    cp "${CFS_DIR}/cfe/cmake/Makefile.sample" "${CFS_DIR}/Makefile"
    ok "Top-level Makefile installed"
fi

echo ""
ok "Install complete."
echo ""
echo "  Build the image:"
echo "    cd ${CFS_DIR}"
echo "    make SIMULATION=native MISSIONCONFIG=sample prep   # MISSIONCONFIG: the tree has both sample_defs and lrmh_defs"
echo "    make -j\$(nproc)"
echo "    make install"
echo ""
echo "  Then from this repo:"
echo "    cmake -B build -S . -DTARGET=sim && cmake --build build   # HAL sim + tests"
echo "    LRMH_CFS_ROOT=${CFS_DIR} scripts/lrmh_launch.sh            # full stack"
echo ""
