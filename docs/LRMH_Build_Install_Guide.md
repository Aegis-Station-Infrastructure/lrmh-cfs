# LRMH cFS Build and Install Guide

**Document:** LRMH-BI-001
**Version:** 1.0
**Date:** October 2026
**Platform:** Ubuntu-family Linux, NASA cFS v7.0.0

*Scope: step by step from a bare Linux box to a running site test, with the manual path behind install.sh and the troubleshooting that goes with it.*

---

## Prerequisites

- git
- cmake >= 3.13
- gcc, make (`apt install build-essential`)
- python3 3.10+ and the `websockets` package, version 13 or newer, for the bridge (`pip install 'websockets>=13'`; distribution packages are often older); the console, the scripted drivers and the record readers need only the standard library
- about 600 MB of disk for the cFS tree and build

Verify:

    git --version
    cmake --version
    gcc --version

### Kernel message-queue depth (required)

cFS software-bus pipes are POSIX message queues. Linux caps an unprivileged queue at `fs.mqueue.msg_max` messages, 10 by default, and OSAL silently truncates every pipe to that limit. During a site test the LRMH apps publish around 200 packets per second through TO_LAB's pipe (100 Hz samples plus the status packets), so a depth-10 pipe overflows within a fraction of a second. The packets that get dropped are the single-shot ones at the end of the test: the site record (`0x08AA`) and the human-readable summary (`0x08AB`). The DS file is still written, but the UI never sees the result.

Raise the limit:

    sudo sysctl fs.mqueue.msg_max=1024

Make it persistent:

    echo "fs.mqueue.msg_max=1024" | sudo tee /etc/sysctl.d/99-cfs.conf
    sudo sysctl --system

Verify:

    cat /proc/sys/fs/mqueue/msg_max     # should print 1024

`scripts/lrmh_launch.sh` checks this at startup and tries `sudo -n sysctl` if it is too low; if it cannot, it warns and continues.

## 1. Clone cFS v7.0.0

Clone the `v7.0.0` tag, the release LRMH is verified against. A bare `git clone` gives you the development branch, which contains a different app set.

    git clone --branch v7.0.0 --recurse-submodules https://github.com/nasa/cFS.git ~/lrmh-cfs-build

This pulls cFE, OSAL, PSP, CI_LAB, TO_LAB, SCH_LAB and sample_app.

Verify:

    ls ~/lrmh-cfs-build/cfe/CMakeLists.txt ~/lrmh-cfs-build/osal ~/lrmh-cfs-build/psp
    ls ~/lrmh-cfs-build/apps/          # ci_lab  sample_app  sch_lab  to_lab

## 2. Clone this repository with its submodules

    git clone --recurse-submodules https://github.com/Aegis-Station-Infrastructure/lrmh-cfs.git ~/lrmh-cfs
    cd ~/lrmh-cfs

The five NASA support apps that LRMH's FDIR and storage depend on (DS, FM, LC, SC, HS) are git submodules under `vendor/cfs-support-apps/`, each pinned to a known upstream SHA at its v7.0.0 tag. If you cloned without `--recurse-submodules`:

    git submodule update --init --recursive

`install.sh` fails loudly if a submodule directory is empty, because a cFS image without LC, SC and HS has no fault detection or recovery path.

## 3. Install LRMH into the cFS tree

    ./install.sh ~/lrmh-cfs-build

The script:

1. Removes any previous LRMH install from the tree.
2. Copies the eight mission apps (`apps/lrmh_*`) into `<cfs>/apps/`.
3. Installs the shared sources into `<cfs>/lrmh_defs/`: `lrmh_msgids.h`, `lrmh_msg.h`, `lrmh_seq_token.h` and `.c`, `lrmh_platform_cfg.h`, the HAL header and the UDP HAL client (`hal/client/lrmh_hal_client.c`), and the two per-app message headers the LC tables need.
4. Creates `<cfs>/sample_defs/` from the cFE template if it does not exist.
5. Appends the thirteen apps (five support apps plus eight LRMH apps) to `MISSION_GLOBAL_APPLIST` in `targets.cmake` and adds `lrmh_defs` to the include path.
6. Copies DS, FM, LC, SC and HS from the submodules into `<cfs>/apps/` (dropping their unit-test directories) and overlays the LRMH tables: DS filter and file tables, nine SC RTS sources plus a placeholder ATS, the HS application monitor table, and the LC watchpoint and actionpoint tables (padded to `LC_MAX_WATCHPOINTS` / `LC_MAX_ACTIONPOINTS` so the LC validator accepts them).
7. Patches the support apps: LC's power-on state from DISABLED to ACTIVE; LC's command, send-HK and sample-actionpoint topic IDs from 0xA4 to 0xA6 (which collide with LRMH's command MIDs) to 0xC3 to 0xC5; HS's wakeup topic from 0xB0 (which is `LRMH_SEQ_TO_CTRL_MID`) to 0xC7; SC's HK telemetry topic from 0xAA (which is the site record MID) to 0xC8; SC's RTS pool from 4 to 16, its table list extended with RTS 005 to 009, and its header-update mode on (so SC computes the checksum of each RTS command at send time instead of rejecting the scaffolded ones).
8. Rewrites the startup script `cpu1_cfe_es_startup.scr` so the thirteen apps load before the `!` end marker.
9. Appends OSAL overrides to `default_osconfig.cmake`: queue depth 1024, maximum path length 256, maximum filename length 64 (DS-built names like `lrmh_site_<13 digits>.bin` exceed the 20-byte default).
10. Patches TO_LAB's subscription table with the LRMH telemetry MIDs and SCH_LAB's schedule with the LRMH wakeups, the LC sample-actionpoint message and the SC wakeup.
11. Copies the top-level `Makefile` from the cFE template if missing.

It warns if the tree is not at the `v7.0.0` tag and is safe to re-run: every step checks for its own sentinel before patching.

## 4. Build the cFS image

    cd ~/lrmh-cfs-build
    make SIMULATION=native MISSIONCONFIG=sample prep
    make -j$(nproc)
    make install

`MISSIONCONFIG=sample` is required on the prep step because the tree now contains both `sample_defs/` and `lrmh_defs/` and cFS cannot pick the mission configuration by itself. Plain `make` and `make install` follow.

Verify the apps built:

    ls ~/lrmh-cfs-build/build/exe/cpu1/cf/lrmh_*.so

Expected:

    lrmh_cmd.so  lrmh_ctrl.so  lrmh_fault.so  lrmh_hk.so  lrmh_proc.so  lrmh_report.so  lrmh_seq.so  lrmh_sense.so

and the support apps and tables alongside them (`ds.so`, `fm.so`, `lc.so`, `sc.so`, `hs.so`, `lc_def_wdt.tbl`, `lc_def_adt.tbl`, `sc_rts001.tbl` through `sc_rts009.tbl`, `hs_amt.tbl`, `ds_filter_tbl.tbl`, `ds_file_tbl.tbl`).

## 5. Build the HAL simulator and the unit tests

Back in this repository:

    cd ~/lrmh-cfs
    cmake -B build -S . -DTARGET=sim
    cmake --build build
    (cd build && ctest --output-on-failure)

This standalone build does not need the cFS tree. It produces the physics models (`sim/`), the HAL simulator `build/hal/sim/lrmh_hal_sim`, the site probe `build/sim/regolith_probe`, and eight per-app unit test executables (`test_lrmh_cmd` through `test_lrmh_fault`, each registered with ctest) that compile the app source against the stubs under `cfe_stubs/`.

`scripts/lrmh_launch.sh` requires `build/hal/sim/lrmh_hal_sim` to exist.

## 6. Boot check

    scripts/lrmh_boot_check.sh ~/lrmh-cfs-build/build/exe/cpu1

This starts the HAL simulator if it has been built, boots `core-cpu1` in the background, waits up to 40 s (second argument) for the eight init events, prints them and exits non-zero if any are missing. It is what CI runs. The eight lines it looks for:

    LRMH_CMD: initialized, Mode=STANDBY
    LRMH_SEQ: initialized, Phase=IDLE
    LRMH_CTRL: initialized
    LRMH_SENSE: initialized @ 100 Hz
    LRMH_PROC: initialized
    LRMH_REPORT: initialized (DS-routed I/O)
    LRMH_HK: initialized
    LRMH_FAULT: thin orchestrator initialized

## 7. Run the stack

    scripts/lrmh_launch.sh

The launcher finds the cFS tree through `LRMH_CFS_ROOT` (default `~/lrmh-cfs-build`) and:

1. Checks `fs.mqueue.msg_max`.
2. Pre-creates `<cfs>/build/exe/cpu1/cf/lrmh/data/` and `cf/lrmh/reports/` (a clean rebuild wipes `cf/`, and DS silently drops captures when the directory is missing).
3. Starts `lrmh_hal_sim` (UDP 17011) with its log in `build/logs/halsim.log` (override the directory with `LRMH_LOG_DIR`).
4. Starts `core-cpu1` from its install directory with its log in `build/logs/cfs.log`.
5. Sends the TO_LAB output-enable command (destination `127.0.0.1`) and the SC enable-RTS-group command for RTS 1 to 9. Upstream SC boots with every RTS disabled; without this step every LC actionpoint trip is rejected with "RTS Disabled".
6. Starts `ground/lrmh_bridge.py --serve-ui` with the UI on `http://127.0.0.1:8080/` and its log in `build/logs/bridge.log`.

Environment variables: `LRMH_CFS_ROOT` (cFS tree), `LRMH_WS_HOST` (bridge bind address, default `127.0.0.1`; `0.0.0.0` exposes the unauthenticated WebSocket to the local network), `LRMH_UI_PORT` (default 8080).

    scripts/lrmh_launch.sh stop

stops the three processes.

## 8. Run a site test

In a second terminal:

    python3 scripts/lrmh_smoke_test.py

sends NOOP, INIT_INSTRUMENT, DEPLOY and a short START_SITE_TEST (2 cm, 50 N) and tails the cFS log. Or open the UI, pick a test and start it. The README's "Your first site test" walks through what you will see. Then:

    python3 scripts/lrmh_read_record.py --latest

prints the DS-captured site record.

## 9. Manual integration (reference)

If you would rather see what `install.sh` does than run it, the pieces are:

**targets.cmake**

    list(APPEND MISSION_GLOBAL_APPLIST
        ds fm lc sc hs
        lrmh_cmd lrmh_seq lrmh_ctrl lrmh_sense
        lrmh_proc lrmh_report lrmh_hk lrmh_fault
    )
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lrmh_defs)

**Startup script** (before the `!` line):

    CFE_APP, ds,          DS_AppMain,          DS,          95,  16384, 0x0, 0;
    CFE_APP, fm,          FM_AppMain,          FM,          96,  16384, 0x0, 0;
    CFE_APP, lc,          LC_AppMain,          LC,          97,  16384, 0x0, 0;
    CFE_APP, sc,          SC_AppMain,          SC,          98,  16384, 0x0, 0;
    CFE_APP, hs,          HS_AppMain,          HS,          99,  16384, 0x0, 0;
    CFE_APP, lrmh_cmd,    LRMH_CMD_AppMain,    LRMH_CMD,    100, 16384, 0x0, 0;
    CFE_APP, lrmh_seq,    LRMH_SEQ_AppMain,    LRMH_SEQ,    100, 16384, 0x0, 0;
    CFE_APP, lrmh_ctrl,   LRMH_CTRL_AppMain,   LRMH_CTRL,   100, 16384, 0x0, 0;
    CFE_APP, lrmh_sense,  LRMH_SENSE_AppMain,  LRMH_SENSE,  100, 16384, 0x0, 0;
    CFE_APP, lrmh_proc,   LRMH_PROC_AppMain,   LRMH_PROC,   100, 16384, 0x0, 0;
    CFE_APP, lrmh_report, LRMH_REPORT_AppMain, LRMH_REPORT, 100, 16384, 0x0, 0;
    CFE_APP, lrmh_hk,     LRMH_HK_AppMain,     LRMH_HK,     100, 16384, 0x0, 0;
    CFE_APP, lrmh_fault,  LRMH_FAULT_AppMain,  LRMH_FAULT,  100, 16384, 0x0, 0;

**TO_LAB subscriptions** (`to_lab_sub.c`, before the `CFE_SB_MSGID_RESERVED` entry, each with a message limit of 50): `0x08A0` HK, `0x08A1` SEQ_STATUS, `0x08A2` CTRL_STATUS, `0x08A3` SENSE, `0x08A4` PROC, `0x08A5` (reserved), `0x08A6` FAULT, `0x08AA` SITE_RECORD, `0x08AB` SITE_HUMAN, `0x08AC` CMD_STATUS. The EVS long-event subscription is deliberately not added: the boot-time event burst overflows the pipe, and the UI synthesises its own events from telemetry.

**SCH_LAB schedule** (`sch_lab_table.c`, before the `CFE_SB_MSGID_RESERVED` entry; the stock table ticks at 100 Hz, so the second field is the period in ticks):

    {CFE_SB_MSGID_WRAP_VALUE(0x18A1), 100, 0},   /* LRMH_SEND_HK_MID      1 Hz   */
    {CFE_SB_MSGID_WRAP_VALUE(0x18C0), 100, 0},   /* LRMH_SEQ_WAKEUP_MID   1 Hz   */
    {CFE_SB_MSGID_WRAP_VALUE(0x18C1),  10, 0},   /* LRMH_SENSE_WAKEUP_MID 10 Hz  */
    {CFE_SB_MSGID_WRAP_VALUE(0x18C2), 100, 0},   /* LRMH_FAULT_WAKEUP_MID 1 Hz   */
    {CFE_SB_MSGID_WRAP_VALUE(0x18C5), 10, 0, 8, {0xFFFF, 0xFFFF, 1, 0}},  /* LC sample all APs, 10 Hz */
    {CFE_SB_MSGID_WRAP_VALUE(0x18AB), 10, 0, 0, {0}},                     /* SC wakeup, 10 Hz */
    {CFE_SB_MSGID_WRAP_VALUE(0x18C7), 100, 0, 0, {0}},                    /* HS wakeup on its relocated topic, 1 Hz */

Note: the stock SCH_LAB table runs at 100 ticks per second, so `100` means 1 Hz and `10` means 10 Hz; the LRMH_SEQ and LRMH_FAULT wakeups are therefore scheduled at 1 Hz by this table even though the apps are designed for 10 Hz. The sequencer also runs its phase machine after every message it receives (its receive call has a 100 ms timeout), so phase advance is not gated on the wakeup rate in practice. Without the LC entry the watchpoints never evaluate; without the SC entry an RTS starts but never advances.

**OSAL** (`default_osconfig.cmake`):

    set(OSAL_CONFIG_QUEUE_MAX_DEPTH 1024 CACHE STRING "" FORCE)
    set(OSAL_CONFIG_MAX_PATH_LEN 256 CACHE STRING "" FORCE)
    set(OSAL_CONFIG_MAX_FILE_NAME 64 CACHE STRING "" FORCE)

**Support app patches**: `lc_internal_cfg.h` `DEFAULT_LC_STATE_POWER_ON_RESET` to `LC_STATE_ACTIVE`; `lc_topicids.h` CMD/SEND_HK/SAMPLE_AP topic IDs 0xA4/0xA5/0xA6 to 0xC3/0xC4/0xC5; `hs_topicids.h` WAKEUP 0xB0 to 0xC7; `sc_topicids.h` HK_TLM 0xAA to 0xC8; `sc/CMakeLists.txt` table list extended with `sc_rts005.c` to `sc_rts009.c`; `sc_interface_cfg.h` `DEFAULT_SC_INTERFACE_NUMBER_OF_RTS` 4 to 16; `sc_internal_cfg.h` `DEFAULT_SC_INTERNAL_PLATFORM_ENABLE_HEADER_UPDATE` to true; SC's `CMakeLists.txt` table list extended with `sc_rts005.c` to `sc_rts009.c`.

## 10. Building against the bench backend

    cmake -S . -B build-bench -DTARGET=bench
    cmake --build build-bench
    ./build-bench/hal/bench/lrmh_hal_bench

The cFS image is unchanged; it talks UDP 17011 to whichever backend is running. See the [HAL Porting Guide](LRMH_HAL_Porting_Guide.md). `-DTARGET=flight` only warns: there is no flight backend.

## Troubleshooting

**`install.sh` stops with "vendor submodule not checked out"**: run `git submodule update --init --recursive` in this repository and re-run the script.

**`make prep` complains about the mission configuration**: add `MISSIONCONFIG=sample`. The tree has both `sample_defs/` and `lrmh_defs/`.

**Apps not loading**: check that the LRMH entries are before the `!` in `build/exe/cpu1/cf/cfe_es_startup.scr`. The `!` marks end of file for the startup parser.

**"Startup Sync failed"**: normal; the cfe_testcase app does not complete sync.

**"No subscribers for MsgId"**: normal during startup for some cFE internal messages.

**`LRMH_SENSE: HAL read failed` or `LRMH_CMD: HAL Init failed (0xFFFFFFF8)`**: the HAL backend is not running. `-8` is `LRMH_HAL_BACKEND_TIMEOUT`. Start `build/hal/sim/lrmh_hal_sim` (the launcher does this).

**Site result never reaches the UI but `cf/lrmh/data/` has a file**: `fs.mqueue.msg_max` is below 1024. Fix the sysctl and restart.

**"Start RTS NNN Rejected: RTS Disabled"**: the SC enable-group command was not sent. Send `SC_ENABLE_RTS_GRP_CC` (function code 16 on `0x18A9`) for RTS 1 to 9, or restart with the launcher.

**LC actionpoints never fire**: confirm the SCH_LAB entry for `0x18C5` (LC sample actionpoints) exists and that LC's state is ACTIVE in its HK.

**"RTS NNN Command Failed Checksum"**: SC header-update mode is off. The install script turns it on in `sc_internal_cfg.h`; rebuild after changing it.

**`OS_FS_ERR_NAME_TOO_LONG` from DS**: the OSAL filename override (64) did not take. Check `default_osconfig.cmake` and re-run `make prep`.

**Stale shared memory after an abnormal shutdown**: `ipcrm --all=shm`. This removes every SysV shared-memory segment you own, not only cFS's.

**MID collisions with NASA apps**: upstream LC's topic IDs 0xA4 to 0xA6 resolve to `0x18A4` to `0x18A6` (LRMH's SENSE, PROC and REPORT command MIDs), HS's wakeup topic 0xB0 resolves to `0x18B0` (`LRMH_SEQ_TO_CTRL_MID`), and SC's HK telemetry topic 0xAA resolves to `0x08AA` (the site record MID). `install.sh` relocates all of them (LC to 0xC3 to 0xC5, HS wakeup to 0xC7, SC HK to 0xC8); if you integrate by hand, do the same.

**Build errors about `CFE_SB_MsgId_t`**: app source must wrap raw MIDs with `CFE_SB_ValueToMsgId()` before `CFE_SB_Subscribe()`; integer MIDs do not compile on v7.0.0.
