/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file hs_table.c
 * @brief LRMH HS (Health & Safety) Application Monitor Table
 *
 * Per ADR-0017 (FDIR via LC + SC + HS) §"HS app-restart policy".
 *
 * Per-app restart policy:
 *
 *   lrmh_cmd    → NO RESTART. lrmh_cmd owns the mode FSM and the
 *                 dispatch cache (ADR-0014). Losing the command
 *                 surface requires ground intervention.
 *
 *   lrmh_seq    → APP_RESTART. Restart bumps sequence-token epoch
 *                 via the cold-start init path (ADR-0007).
 *
 *   lrmh_ctrl   → APP_RESTART. HAL safety clamps catch any
 *                 in-progress runaway during the restart window.
 *
 *   lrmh_sense  → APP_RESTART. Per ADR-0017 the LRMH policy also
 *                 wants RTS_RETRACT_AND_SAFE on this restart, but
 *                 the HS Draco version exposes only
 *                 NOACT / PROC_RESET / APP_RESTART / EVENT —
 *                 the compound "restart + RTS" action requires
 *                 the MAT (Message Action Table) mechanism which
 *                 is a separate ADR-0017 follow-up. APP_RESTART
 *                 alone is the conservative reduction.
 *
 *   lrmh_proc   → APP_RESTART. Site fails to PARTIAL on next
 *                 processing attempt.
 *
 *   lrmh_report → APP_RESTART. Site fails to PARTIAL.
 *
 *   lrmh_hk     → APP_RESTART. Ground visibility outage during
 *                 the restart window.
 *
 *   lrmh_fault  → APP_RESTART. Same RTS-on-restart caveat as
 *                 lrmh_sense.
 *
 * The cycle counts (CycleCount) define the HS detection window —
 * an app that fails to send its expected message within the
 * cycle count triggers the configured action. HS evaluates at
 * 1 Hz so 30 cycles ≈ 30 s detection window.
 */

#include "cfe.h"
#include "hs_mission_cfg.h"
#include "hs_tbl.h"
#include "hs_tbldefs.h"
#include "cfe_tbl_filedef.h"

#define LRMH_HS_CYCLES   30   /* 30 s @ 1 Hz HS evaluation */

HS_AMTEntry_t HS_AppMon_Tbl[HS_MAX_MONITORED_APPS] = {
    /*          AppName       NullTerm CycleCount   ActionType */
    /*   0 */ {"LRMH_CMD",    0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*   1 */ {"LRMH_SEQ",    0, LRMH_HS_CYCLES, HS_AMTActType_APP_RESTART},
    /*   2 */ {"LRMH_CTRL",   0, LRMH_HS_CYCLES, HS_AMTActType_APP_RESTART},
    /*   3 */ {"LRMH_SENSE",  0, LRMH_HS_CYCLES, HS_AMTActType_APP_RESTART},
    /*   4 */ {"LRMH_PROC",   0, LRMH_HS_CYCLES, HS_AMTActType_APP_RESTART},
    /*   5 */ {"LRMH_REPORT", 0, LRMH_HS_CYCLES, HS_AMTActType_APP_RESTART},
    /*   6 */ {"LRMH_HK",     0, LRMH_HS_CYCLES, HS_AMTActType_APP_RESTART},
    /*   7 */ {"LRMH_FAULT",  0, LRMH_HS_CYCLES, HS_AMTActType_APP_RESTART},
    /*   8 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*   9 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  10 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  11 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  12 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  13 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  14 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  15 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  16 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  17 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  18 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  19 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  20 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  21 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  22 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  23 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  24 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  25 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  26 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  27 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  28 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  29 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  30 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT},
    /*  31 */ {"", 0, LRMH_HS_CYCLES, HS_AMTActType_NOACT}
};

CFE_TBL_FILEDEF(HS_AppMon_Tbl, HS.AppMon_Tbl, LRMH HS app monitor, hs_amt.tbl)
