/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_seq_app.h
 * @brief LRMH lrmh_seq application — public interface
 *
 * lrmh_seq owns the site-test phase machine (ADR-0004), the
 * sequence-token stamping for every outbound command (ADR-0007),
 * and the recovery-tier dispatch (ADR-0008).
 *
 * Per ADR-0006, lrmh_seq is deliberately separate from lrmh_cmd:
 * a sequencer crash must not take down the command surface, and
 * the seq's HS restart bumps the token epoch automatically via
 * the cold-start init path.
 */

#ifndef LRMH_SEQ_APP_H
#define LRMH_SEQ_APP_H

#include "cfe.h"
#include "lrmh_msgids.h"
#include "lrmh_msg.h"
#include "lrmh_seq_token.h"
#include "lrmh_platform_cfg.h"

#include "lrmh_seq_msg.h"
#include "lrmh_seq_events.h"

/* lrmh_seq receives LRMH_StartSiteTestCmd_t on LRMH_SEQ_CMD_MID
 * via the lrmh_cmd forwarding path. The struct is defined in
 * lrmh_cmd's per-app msg header — this is a deliberate
 * cross-app dependency: lrmh_seq is a *consumer* of the
 * dispatch protocol that lrmh_cmd owns. */
#include "lrmh_cmd_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * App data
 * ============================================================ */
typedef struct {
    CFE_SB_PipeId_t  CmdPipe;

    /* Phase machine state */
    LRMH_Phase_t     Phase;
    LRMH_Phase_t     PreviousPhase;

    /* Current test parameters (latched from StartSiteTestCmd) */
    uint16   SiteId;
    uint16   SessionId;
    uint16   SequenceNumber;
    uint16   PairedSiteId;
    uint8    TestProfileId;
    uint8    ToolId;
    uint8    EnableRepeatLoad;
    uint8    EnablePostProfile;
    uint8    EnableTraction;
    uint8    DispatchFlags;
    float    MaxDepth_m;
    float    MaxLoad_N;
    float    MicroSiteOffsetX_mm;
    float    MicroSiteOffsetY_mm;

    /* Runtime tracking */
    uint32   SampleCount;            /* reserved: not populated (the sequencer does not read sample telemetry) */
    uint32   RepeatCyclesDone;
    uint32   RepeatCyclesTarget;
    float    ElapsedTime_s;
    float    PhaseStartTime_s;
    uint64   TestStartTime_us;       /* stamped on START_SITE_TEST receipt */
    uint8    WaitingForCtrl;         /* 1 while an actuator command is in flight */
    uint8    WaitingForProc;         /* processing triggers sent and not yet answered */
    uint8    QualityFlag;            /* LRMH_Quality_t */

    /* Recovery tier tracking (ADR-0008) */
    uint8    T1AttemptsCurrentPhase;
    uint8    PhaseStep;              /* sub-step within the current phase; 0 on entry */
    uint8    Reserved8;
    float    CtrlDeadline_s;         /* ElapsedTime_s by which the in-flight command must complete */

    /* Counters */
    uint32   CmdCount;
    uint32   ErrCount;
} LRMH_SEQ_AppData_t;

/* ============================================================
 * Public entry point
 * ============================================================ */
void LRMH_SEQ_AppMain(void);

/* ============================================================
 * Test-exposed internal API (the unit test #includes this source)
 * ============================================================ */
extern LRMH_SEQ_AppData_t g_SeqApp;

void LRMH_SEQ_TransitionTo(LRMH_Phase_t new_phase);
bool LRMH_SEQ_HandleStartTest(const LRMH_StartSiteTestCmd_t *cmd);
void LRMH_SEQ_HandleRecovery(uint8 fault_code, uint8 tier);
void LRMH_SEQ_AdvancePhase(void);

#ifdef __cplusplus
}
#endif

#endif /* LRMH_SEQ_APP_H */
