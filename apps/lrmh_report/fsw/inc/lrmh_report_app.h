/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_report_app.h
 * @brief LRMH lrmh_report application — public interface
 *
 * Per ADR-0016: site result assembly and publication. **No
 * direct file I/O.** Site results publish on two SB MIDs
 * (binary record + human text) for the standard cFS DS app to
 * capture into rotated files. JSON moves ground-side.
 *
 * Per ADR-0006, lrmh_report is its own app (separate from
 * lrmh_proc) so a slow DS write doesn't block signal processing.
 * The DS write happens in the DS app, not here; the publication
 * is a non-blocking SB transmit either way.
 */

#ifndef LRMH_REPORT_APP_H
#define LRMH_REPORT_APP_H

#include "cfe.h"
#include "lrmh_msgids.h"
#include "lrmh_msg.h"
#include "lrmh_seq_token.h"
#include "lrmh_platform_cfg.h"

/* Bring in the per-phase result shape from lrmh_proc — lrmh_report
 * subscribes to LRMH_PROC_TLM_MID and consumes those packets. */
#include "lrmh_proc_msg.h"

#include "lrmh_report_msg.h"
#include "lrmh_report_events.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * App data
 * ============================================================ */
typedef struct {
    CFE_SB_PipeId_t  CmdPipe;

    /* Per-phase result accumulator */
    LRMH_ProcPhaseResult_t  Phases[LRMH_MAX_PHASES_PER_TEST];
    uint8                   PhaseCount;

    /* Site identification */
    uint16                  CurrentSiteId;
    uint16                  CurrentSessionId;
    uint16                  CurrentSequenceNumber;
    uint16                  PairedSiteId;
    uint8                   TestProfileId;     /* latched from report trigger */
    uint8                   ToolId;            /* latched from report trigger */

    /* Timing */
    uint64                  SiteStartTime_us;
    uint64                  SiteEndTime_us;

    /* Counters */
    uint32                  CmdCount;
    uint32                  ErrCount;
    uint32                  PartialTaggedCount;  /* T2 SKIP_SITE tags */

    /* Receiver token state */
    LRMH_SeqToken_t         AcceptedToken;
    bool                    ResetObserved;
} LRMH_REPORT_AppData_t;

/* ============================================================
 * Public entry point
 * ============================================================ */
void LRMH_REPORT_AppMain(void);

/* ============================================================
 * Test-exposed internal API
 * ============================================================ */
extern LRMH_REPORT_AppData_t g_ReportApp;

void LRMH_REPORT_IngestPhaseResult(const LRMH_ProcResultTlm_t *result);
void LRMH_REPORT_GenerateSiteResult(uint16 site_id, uint8 quality);
void LRMH_REPORT_TagPartial(uint16 site_id);
void LRMH_REPORT_OnTokenReset(uint8 reason);
int  LRMH_REPORT_FormatHumanText(const LRMH_SiteResultRecord_t *rec,
                                  char *out, size_t out_max);

#ifdef __cplusplus
}
#endif

#endif /* LRMH_REPORT_APP_H */
