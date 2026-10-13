/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lc_actionpoints.c
 * @brief LRMH LC (Limit Checker) Actionpoint Definition Table
 *
 * Per ADR-0017 (FDIR via LC + SC + HS).
 *
 * One actionpoint per fault flag. Each AP watches the LC
 * watchpoint(s) that map to its flag and fires the per-flag
 * NOTIFY RTS, which is a static CCSDS command to lrmh_fault
 * carrying the correct LRMH_FAULT_FLAG_* in the payload.
 *
 * SC RTSes are static and cannot inject the per-watchpoint
 * CustomFuncArgument at fire time, so we need one RTS per
 * distinct fault flag rather than a single OR-aggregated path.
 * lrmh_fault then performs the multi-step recovery
 * orchestration (T1 RESUME / T2 SKIP_SITE / T3 SAFE per
 * ADR-0008) using the per-fault tier table in lrmh_fault_app.c.
 *
 * RTS 002-004 (tables/sc/sc_rts002.c to sc_rts004.c) are single-
 * command, ground-startable stubs named after the recovery tiers;
 * no actionpoint references them. The text below describes what a
 * table-only fallback would do if lrmh_fault were unavailable. They would run unconditionally for the affected
 * tier and don't carry per-WP information.
 */

#include "cfe_tbl_filedef.h"
#include "cfe_evs_extern_typedefs.h"
#include "lc_platform_cfg.h"
#include "lc_msgdefs.h"
#include "lc_tbl.h"

#include "lrmh_msgids.h"

/* Match the watchpoint indices in lc_watchpoints.c */
#define LRMH_WP_FORCE        0
#define LRMH_WP_CURRENT      1
#define LRMH_WP_STROKE       2
#define LRMH_WP_TEMP         3
#define LRMH_WP_BUS_LOW      4
#define LRMH_WP_BUS_HIGH     5
#define LRMH_WP_FORCE_RATE   6

/* Actionpoint indices — one per distinct fault flag */
#define LRMH_AP_FORCE_SATURATION  0
#define LRMH_AP_OVERCURRENT       1
#define LRMH_AP_HARD_STOP         2
#define LRMH_AP_THERMAL           3
#define LRMH_AP_BUS_VOLTAGE       4   /* OR of BUS_LOW + BUS_HIGH */
#define LRMH_AP_FORCE_RATE        5

/* RTS IDs — match tables/sc/sc_rts*.c filenames.
 * 001 is the legacy slot, kept for FORCE_SATURATION; the
 * recovery-fallback RTSes occupy 002-004; per-flag NOTIFY
 * RTSes occupy 005-009. */
#define LRMH_RTS_NOTIFY_FORCE_SATURATION  1
#define LRMH_RTS_RESUME_PHASE             2   /* T1 fallback */
#define LRMH_RTS_SKIP_SITE                3   /* T2 fallback */
#define LRMH_RTS_RETRACT_AND_SAFE         4   /* T3 fallback */
#define LRMH_RTS_NOTIFY_OVERCURRENT       5
#define LRMH_RTS_NOTIFY_HARD_STOP         6
#define LRMH_RTS_NOTIFY_THERMAL           7
#define LRMH_RTS_NOTIFY_BUS_VOLTAGE       8
#define LRMH_RTS_NOTIFY_FORCE_RATE        9

/* Event IDs */
#define LRMH_AP_FORCE_SATURATION_EID  0xA001
#define LRMH_AP_OVERCURRENT_EID       0xA002
#define LRMH_AP_HARD_STOP_EID         0xA003
#define LRMH_AP_THERMAL_EID           0xA004
#define LRMH_AP_BUS_VOLTAGE_EID       0xA005
#define LRMH_AP_FORCE_RATE_EID        0xA006

/* Common AP shape for the single-WP cases. RPN postfix is
 * "WP =". The text argument must be passed as a bare string
 * literal — wrapping it in parens (e.g. via additional macros)
 * produces a parenthesized array initializer that gcc rejects
 * under -Wpedantic -Werror. */
#define LRMH_AP_SINGLE_WP(wp_idx, rts_id, eid, text) {                 \
    .DefaultState      = LC_APSTATE_ACTIVE,                            \
    .MaxPassiveEvents  = 1,                                            \
    .MaxPassFailEvents = 1,                                            \
    .MaxFailPassEvents = 1,                                            \
    .RTSId             = rts_id,                                       \
    .MaxFailsBeforeRTS = 1,                                            \
    .RPNEquation       = { wp_idx, LC_RPN_EQUAL },                     \
    .EventType         = CFE_EVS_EventType_INFORMATION,                \
    .EventID           = eid,                                          \
    .EventText         = text                                          \
}

LC_ADTEntry_t LC_ADT[LC_MAX_ACTIONPOINTS] = {

    [LRMH_AP_FORCE_SATURATION] = LRMH_AP_SINGLE_WP(
        LRMH_WP_FORCE,
        LRMH_RTS_NOTIFY_FORCE_SATURATION,
        LRMH_AP_FORCE_SATURATION_EID,
        "LRMH force saturation"),

    [LRMH_AP_OVERCURRENT] = LRMH_AP_SINGLE_WP(
        LRMH_WP_CURRENT,
        LRMH_RTS_NOTIFY_OVERCURRENT,
        LRMH_AP_OVERCURRENT_EID,
        "LRMH overcurrent"),

    [LRMH_AP_HARD_STOP] = LRMH_AP_SINGLE_WP(
        LRMH_WP_STROKE,
        LRMH_RTS_NOTIFY_HARD_STOP,
        LRMH_AP_HARD_STOP_EID,
        "LRMH stroke hard stop"),

    [LRMH_AP_THERMAL] = LRMH_AP_SINGLE_WP(
        LRMH_WP_TEMP,
        LRMH_RTS_NOTIFY_THERMAL,
        LRMH_AP_THERMAL_EID,
        "LRMH thermal"),

    /* BUS_VOLTAGE OR-aggregates the two voltage WPs (under-
     * and over-voltage) since both raise the same fault
     * flag; the RTS payload carries LRMH_FAULT_FLAG_BUS_VOLTAGE
     * regardless of which WP tripped. */
    [LRMH_AP_BUS_VOLTAGE] = {
        .DefaultState      = LC_APSTATE_ACTIVE,
        .MaxPassiveEvents  = 1,
        .MaxPassFailEvents = 1,
        .MaxFailPassEvents = 1,
        .RTSId             = LRMH_RTS_NOTIFY_BUS_VOLTAGE,
        .MaxFailsBeforeRTS = 1,
        .RPNEquation       = {
            LRMH_WP_BUS_LOW,
            LRMH_WP_BUS_HIGH, LC_RPN_OR,
            LC_RPN_EQUAL
        },
        .EventType         = CFE_EVS_EventType_INFORMATION,
        .EventID           = LRMH_AP_BUS_VOLTAGE_EID,
        .EventText         = "LRMH bus voltage"
    },

    /* FORCE_RATE is a derivative of two noisy samples (1.5 N of noise
     * over 10 ms is 150 N/s), so one evaluation over the threshold is
     * noise; three in a row (300 ms at LC's 10 Hz) is an excursion. */
    [LRMH_AP_FORCE_RATE] = {
        .DefaultState      = LC_APSTATE_ACTIVE,
        .MaxPassiveEvents  = 1,
        .MaxPassFailEvents = 1,
        .MaxFailPassEvents = 1,
        .RTSId             = LRMH_RTS_NOTIFY_FORCE_RATE,
        .MaxFailsBeforeRTS = 3,
        .RPNEquation       = { LRMH_WP_FORCE_RATE, LC_RPN_EQUAL },
        .EventType         = CFE_EVS_EventType_INFORMATION,
        .EventID           = LRMH_AP_FORCE_RATE_EID,
        .EventText         = "LRMH force rate"
    }

    /* Remaining LC_MAX_ACTIONPOINTS slots are LC_ACTION_NOT_USED. */
};

CFE_TBL_FileDef_t CFE_TBL_FileDef = {
    "LC_ADT",                              /* table object name */
    "LC.LC_ADT",                           /* app.table_name */
    "LRMH LC actionpoints",            /* description */
    "lc_def_adt.tbl",                      /* output filename */
    sizeof(LC_ADTEntry_t) * LC_MAX_ACTIONPOINTS
};
