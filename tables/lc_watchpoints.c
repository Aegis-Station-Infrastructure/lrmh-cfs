/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lc_watchpoints.c
 * @brief LRMH LC (Limit Checker) Watchpoint Definition Table
 *
 * Per ADR-0017 (FDIR via LC + SC + HS). Each watchpoint references
 * an envelope value from lrmh_platform_cfg.h (ADR-0012). The
 * threshold is the LC margin value, NOT the envelope value — the
 * watchpoint fires before the limit is crossed so the FSW has time
 * to react.
 *
 * Six threshold-based fault categories live here. The state-
 * watching categories (SENSOR_TIMEOUT implemented; STALL,
 * ENCODER_LOSS, DEPTH_DISAGREE and HOST_NAV_TIMEOUT reserved) belong
 * to lrmh_fault's polled path because they need multi-cycle
 * correlation that LC's single-sample evaluation cannot express.
 * See ADR-0013.
 */

#include "cfe_tbl_filedef.h"
#include "cfe_sb_api_typedefs.h"
#include "lc_platform_cfg.h"
#include "lc_msgdefs.h"
#include "lc_tbl.h"

#include "lrmh_msgids.h"
#include "lrmh_msg.h"
#include "lrmh_sense_msg.h"   /* for LRMH_SampleTlm_t field offsets */
#include "lrmh_fault_msg.h"   /* for LRMH_FAULT_FLAG_* */
#include "lrmh_platform_cfg.h"

/* ───────────────────────────────────────────────────────
 * Watchpoint indices (used by lc_actionpoints.c RPN)
 * ─────────────────────────────────────────────────────── */
#define LRMH_WP_FORCE        0
#define LRMH_WP_CURRENT      1
#define LRMH_WP_STROKE       2
#define LRMH_WP_TEMP         3
#define LRMH_WP_BUS_LOW      4
#define LRMH_WP_BUS_HIGH     5
#define LRMH_WP_FORCE_RATE   6

/* A watchpoint result older than this many LC actionpoint sample
 * cycles (10 Hz) is STALE and does not count as a failure. Without it
 * (0 = never stale) the last sample of a test, which is often a
 * retract transient, stays latched as FAIL after acquisition stops and
 * the actionpoint re-fires on every re-arm. Two cycles: a latched
 * result can be sampled twice at most, below the three consecutive
 * failures FORCE_RATE needs, while a live result (samples arrive at
 * 100 Hz during a test) is refreshed every cycle and never stale. */
#define LRMH_LC_RESULT_STALE_CYCLES  2

/* ───────────────────────────────────────────────────────
 * The watchpoint definition table
 * ─────────────────────────────────────────────────────── */
LC_WDTEntry_t LC_WDT[LC_MAX_WATCHPOINTS] = {

    /* WP 0: Force overload → LRMH_FAULT_FLAG_FORCE_SATURATION (T2) */
    [LRMH_WP_FORCE] = {
        .DataType            = LC_DATA_FLOAT_LE,
        .OperatorID          = LC_OPER_GT,
        .MessageID           = CFE_SB_MSGID_WRAP_VALUE(LRMH_SENSE_TLM_MID),
        .WatchpointOffset    = offsetof(LRMH_SampleTlm_t, force_n),
        .BitMask             = LC_NO_BITMASK,
        .CustomFuncArgument  = LRMH_FAULT_FLAG_FORCE_SATURATION,
        .ResultAgeWhenStale  = LRMH_LC_RESULT_STALE_CYCLES,
        .ComparisonValue.Float32 = LRMH_LC_FORCE_THRESHOLD_N
    },

    /* WP 1: Motor overcurrent → LRMH_FAULT_FLAG_OVERCURRENT (T3) */
    [LRMH_WP_CURRENT] = {
        .DataType            = LC_DATA_FLOAT_LE,
        .OperatorID          = LC_OPER_GT,
        .MessageID           = CFE_SB_MSGID_WRAP_VALUE(LRMH_SENSE_TLM_MID),
        .WatchpointOffset    = offsetof(LRMH_SampleTlm_t, current_a),
        .BitMask             = LC_NO_BITMASK,
        .CustomFuncArgument  = LRMH_FAULT_FLAG_OVERCURRENT,
        .ResultAgeWhenStale  = LRMH_LC_RESULT_STALE_CYCLES,
        .ComparisonValue.Float32 = LRMH_LC_CURRENT_THRESHOLD_A
    },

    /* WP 2: Stroke overrun → LRMH_FAULT_FLAG_HARD_STOP (T2) */
    [LRMH_WP_STROKE] = {
        .DataType            = LC_DATA_FLOAT_LE,
        .OperatorID          = LC_OPER_GT,
        .MessageID           = CFE_SB_MSGID_WRAP_VALUE(LRMH_SENSE_TLM_MID),
        .WatchpointOffset    = offsetof(LRMH_SampleTlm_t, stroke_m),
        .BitMask             = LC_NO_BITMASK,
        .CustomFuncArgument  = LRMH_FAULT_FLAG_HARD_STOP,
        .ResultAgeWhenStale  = LRMH_LC_RESULT_STALE_CYCLES,
        .ComparisonValue.Float32 = LRMH_LC_STROKE_THRESHOLD_M
    },

    /* WP 3: Thermal → LRMH_FAULT_FLAG_THERMAL (T3) */
    [LRMH_WP_TEMP] = {
        .DataType            = LC_DATA_FLOAT_LE,
        .OperatorID          = LC_OPER_GT,
        .MessageID           = CFE_SB_MSGID_WRAP_VALUE(LRMH_SENSE_TLM_MID),
        .WatchpointOffset    = offsetof(LRMH_SampleTlm_t, temp_c),
        .BitMask             = LC_NO_BITMASK,
        .CustomFuncArgument  = LRMH_FAULT_FLAG_THERMAL,
        .ResultAgeWhenStale  = LRMH_LC_RESULT_STALE_CYCLES,
        .ComparisonValue.Float32 = LRMH_LC_TEMP_THRESHOLD_C
    },

    /* WP 4: Bus voltage low → LRMH_FAULT_FLAG_BUS_VOLTAGE (T3) */
    [LRMH_WP_BUS_LOW] = {
        .DataType            = LC_DATA_FLOAT_LE,
        .OperatorID          = LC_OPER_LT,
        .MessageID           = CFE_SB_MSGID_WRAP_VALUE(LRMH_SENSE_TLM_MID),
        .WatchpointOffset    = offsetof(LRMH_SampleTlm_t, bus_v),
        .BitMask             = LC_NO_BITMASK,
        .CustomFuncArgument  = LRMH_FAULT_FLAG_BUS_VOLTAGE,
        .ResultAgeWhenStale  = LRMH_LC_RESULT_STALE_CYCLES,
        .ComparisonValue.Float32 = LRMH_LC_BUS_V_LOW_THRESHOLD
    },

    /* WP 5: Bus voltage high → LRMH_FAULT_FLAG_BUS_VOLTAGE (T3) */
    [LRMH_WP_BUS_HIGH] = {
        .DataType            = LC_DATA_FLOAT_LE,
        .OperatorID          = LC_OPER_GT,
        .MessageID           = CFE_SB_MSGID_WRAP_VALUE(LRMH_SENSE_TLM_MID),
        .WatchpointOffset    = offsetof(LRMH_SampleTlm_t, bus_v),
        .BitMask             = LC_NO_BITMASK,
        .CustomFuncArgument  = LRMH_FAULT_FLAG_BUS_VOLTAGE,
        .ResultAgeWhenStale  = LRMH_LC_RESULT_STALE_CYCLES,
        .ComparisonValue.Float32 = LRMH_LC_BUS_V_HIGH_THRESHOLD
    },

    /* WP 6: Force rate excursion → LRMH_FAULT_FLAG_FORCE_RATE (T1)
     * lrmh_sense computes dF/dt and exposes it on a TLM field. */
    [LRMH_WP_FORCE_RATE] = {
        .DataType            = LC_DATA_FLOAT_LE,
        .OperatorID          = LC_OPER_GT,
        .MessageID           = CFE_SB_MSGID_WRAP_VALUE(LRMH_SENSE_TLM_MID),
        .WatchpointOffset    = offsetof(LRMH_SampleTlm_t, force_rate_nps),
        .BitMask             = LC_NO_BITMASK,
        .CustomFuncArgument  = LRMH_FAULT_FLAG_FORCE_RATE,
        .ResultAgeWhenStale  = LRMH_LC_RESULT_STALE_CYCLES,
        .ComparisonValue.Float32 = LRMH_LC_FORCE_RATE_THRESHOLD_NPS
    }

    /* Remaining LC_MAX_WATCHPOINTS slots are LC_WATCH_NOT_USED by
     * default. Reserve room for future fault categories without
     * needing to renumber existing watchpoints. */
};

/* ───────────────────────────────────────────────────────
 * cFE table file header — written into the .tbl file
 *
 * The LC app registers its WDT as LC_APP_NAME "." LC_WDT_TABLENAME
 * → "LC.LC_WDT" — and looks for it on disk as
 * /cf/lc_def_wdt.tbl per ds_internal_cfg.h /
 * lc_internal_cfg.h. Match that name exactly.
 * ─────────────────────────────────────────────────────── */
CFE_TBL_FileDef_t CFE_TBL_FileDef = {
    "LC_WDT",                              /* table object name */
    "LC.LC_WDT",                           /* app.table_name */
    "LRMH LC watchpoints",             /* description */
    "lc_def_wdt.tbl",                      /* output filename */
    sizeof(LC_WDTEntry_t) * LC_MAX_WATCHPOINTS
};
