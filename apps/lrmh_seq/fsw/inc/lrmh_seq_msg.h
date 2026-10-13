/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_seq_msg.h
 * @brief LRMH lrmh_seq command and telemetry messages
 *
 * Per ADRs 0004 (phase machine), 0007 (sequence token), 0008
 * (recovery tiers).
 */

#ifndef LRMH_SEQ_MSG_H
#define LRMH_SEQ_MSG_H

#include "cfe.h"
#include "lrmh_msg.h"

/* ============================================================
 * Command codes for LRMH_SEQ_CMD_MID
 * ============================================================ */
#define LRMH_SEQ_NOOP_CC              0
#define LRMH_SEQ_RESET_COUNTERS_CC    1
#define LRMH_SEQ_START_SITE_TEST_CC   6   /* same value as LRMH_START_SITE_TEST_CC for forwarding */

/* Recovery commands from lrmh_fault (per ADR-0008) */
#define LRMH_SEQ_RESUME_CC           20   /* T1 RESUME_PHASE */
#define LRMH_SEQ_SKIP_CC             21   /* T2 SKIP_SITE */
#define LRMH_SEQ_ABORT_CC             5   /* T3 (mirrors LRMH_CMD_ABORT_CC) */

/* Manual actuator command from ground (CTRL_MOVE / FORCE / RETRACT
 * / HOLD / ESTOP / clear-ESTOP). lrmh_seq remains the sole token
 * producer per ADR-0007; ground cannot directly stamp tokens, so
 * the command is funneled through here. Allowed only when the
 * phase machine is IDLE so a stray manual command can never race
 * the sequencer's own dispatch during a test. */
#define LRMH_SEQ_MANUAL_CTRL_CC      30

/* ============================================================
 * Recovery decision command (sent by lrmh_fault → lrmh_seq)
 *
 * The fault orchestrator looks up the recovery tier from the
 * per-fault table (ADR-0008) and dispatches via this command.
 * lrmh_seq executes the corresponding tier action.
 * ============================================================ */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
    uint8   FaultCode;          /* LRMH_FAULT_FLAG_* bit */
    uint8   Tier;               /* LRMH_RecoveryTier_t */
    uint8   Reserved[2];
} LRMH_SeqRecoveryCmd_t;

/* The seq → cmd FSM notify struct (LRMH_SeqToCmdNotify_t) and its
 * notify-type defines live in msg/lrmh_msg.h because both lrmh_seq
 * (publisher) and lrmh_cmd (subscriber) need them.
 */

/* ============================================================
 * Manual actuator command (LRMH_SEQ_CMD_MID, FcnCode
 * LRMH_SEQ_MANUAL_CTRL_CC)
 *
 * Ground sends this on lrmh_seq's command MID when it wants to
 * directly drive the actuator outside a site test (positioning,
 * dev moves, manual retract, e-stop). lrmh_seq stamps a sequence
 * token (ADR-0007) and forwards on LRMH_SEQ_TO_CTRL_MID using the
 * existing seq→ctrl path. Same payload fields as LRMH_CtrlCmd_t
 * minus the token, which lrmh_seq adds.
 * ============================================================ */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
    uint8   Mode;                   /* LRMH_CTRL_MODE_* */
    uint8   Reserved[3];
    float   TargetPosition_m;       /* tool-tip depth, not stroke */
    float   TargetForce_N;
    float   RateLimit;              /* m/s for position, N/s for force */
    float   Timeout_s;
} LRMH_SeqManualCtrlCmd_t;

/* ============================================================
 * Sequencer status telemetry (LRMH_SEQ_STATUS_MID)
 * ============================================================ */
typedef struct {
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint8   Phase;              /* LRMH_Phase_t */
    uint8   PreviousPhase;
    uint8   QualityFlag;        /* LRMH_Quality_t */
    uint8   WaitingForCtrl;
    uint16  SiteId;
    uint16  SessionId;
    uint16  SequenceNumber;
    uint16  Epoch;              /* current sequence-token epoch */
    uint32  TotalSamples;       /* reserved: not populated */
    uint32  CmdCount;
    uint32  ErrCount;
    uint32  T1AttemptsCurrentPhase;  /* ADR-0008 escalation tracking */
    uint32  RepeatCyclesDone;
    uint32  RepeatCyclesTarget;
    float   CurrentDepth_m;     /* reserved: not populated (the sequencer does not read sample telemetry) */
    float   CurrentForce_N;     /* reserved: not populated */
    float   ElapsedTime_s;
    float   PhaseStartTime_s;
} LRMH_SeqStatusTlm_t;

#endif /* LRMH_SEQ_MSG_H */
