/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_cmd_msg.h
 * @brief LRMH lrmh_cmd command and telemetry message definitions
 *
 * Per ADR-0014 (site-test dispatch protocol): LRMH_StartSiteTestCmd_t
 * carries the test parameters and the dispatch fields (DispatchFlags,
 * SessionId, SequenceNumber, PairedSiteId, MicroSiteOffset,
 * RepeatLoadCount).
 */

#ifndef LRMH_CMD_MSG_H
#define LRMH_CMD_MSG_H

#include "cfe.h"
#include "lrmh_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Command codes for LRMH_CMD_MID
 * ============================================================ */
#define LRMH_NOOP_CC               0
#define LRMH_RESET_COUNTERS_CC     1
#define LRMH_INIT_INSTRUMENT_CC    2
#define LRMH_DEPLOY_CC             3
#define LRMH_STOW_CC               4
#define LRMH_ABORT_CC              5
#define LRMH_START_SITE_TEST_CC    6
#define LRMH_REQUEST_REPORT_CC     7
#define LRMH_SET_MODE_CC           8
#define LRMH_RESET_FAULT_CC        9    /* 2.0: ADR-0004 FAULT → SAFE manual override */
#define LRMH_SAFE_CC              10    /* 2.0: explicit ground SAFE command */
#define LRMH_POWER_OFF_CC         11    /* 2.0: STANDBY → OFF (split from STOW) */

/* ============================================================
 * DispatchFlags bits (ADR-0014)
 * ============================================================ */
#define LRMH_DISPATCH_STRICT_NAV       (1u << 0)  /* refuse if cached host pose stale or NavState != IDLE */
#define LRMH_DISPATCH_IDEMPOTENT       (1u << 1)  /* retry; check dispatch cache */
#define LRMH_DISPATCH_OVERRIDE_HOST    (1u << 2)  /* allow despite host moving */
#define LRMH_DISPATCH_WITNESS_TARGET   (1u << 3)  /* test against witness target, bypass host check */

/* ============================================================
 * StartSiteTestCmd per ADR-0014. Static_assert below verifies
 * CCSDS alignment.
 * ============================================================ */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;

    /* ── Test parameters ──────────────────────────────── */
    uint16  SiteId;
    uint8   TestProfileId;
    uint8   ToolId;
    float   MaxDepth_m;
    float   MaxLoad_N;
    uint8   EnableRepeatLoad;
    uint8   EnablePostProfile;
    uint8   EnableTraction;     /* deferred stub per ADR-0004 */

    /* ── Dispatch protocol (ADR-0014) ─────────────────── */
    uint8   DispatchFlags;       /* LRMH_DISPATCH_* bits */
    uint16  SessionId;
    uint16  SequenceNumber;
    uint16  PairedSiteId;        /* 0 = standalone; otherwise pre-traffic baseline ref */
    uint16  Reserved16;
    float   MicroSiteOffsetX_mm;
    float   MicroSiteOffsetY_mm;
    uint8   RepeatLoadCount;     /* 0 = default 3 cycles */
    uint8   Reserved[3];
} LRMH_StartSiteTestCmd_t;

_Static_assert(sizeof(LRMH_StartSiteTestCmd_t) % 4 == 0,
               "LRMH_StartSiteTestCmd_t must be 4-byte aligned per CCSDS");

/* ============================================================
 * Set mode command
 * ============================================================ */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
    uint8   NewMode;             /* LRMH_InstrumentMode_t */
    uint8   Reserved[3];
} LRMH_SetModeCmd_t;

/* ============================================================
 * lrmh_cmd telemetry packet — instrument-level state
 *
 * lrmh_hk copies Mode and the counters from this packet into
 * LRMH_HK_TLM_MID; it is also downlinked on its own.
 * ============================================================ */
typedef struct {
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    uint8   Mode;                /* LRMH_InstrumentMode_t */
    uint8   Reserved8[3];
    uint32  CmdCount;
    uint32  ErrCount;
    uint32  RejectedCount;       /* illegal mode transitions, ADR-0004 */
    uint32  DispatchCacheHits;   /* idempotent retries absorbed, ADR-0014 */
    uint16  CurrentSessionId;
    uint16  CurrentSequenceNumber;
    uint16  CurrentSiteId;
    uint16  Reserved16;
} LRMH_CmdTlm_t;

#ifdef __cplusplus
}
#endif

#endif /* LRMH_CMD_MSG_H */
