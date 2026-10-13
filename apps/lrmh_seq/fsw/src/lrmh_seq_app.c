/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_seq_app.c
 * @brief LRMH lrmh_seq application — implementation
 *
 * Owns the site-test phase machine (ADR-0004), the sequence-token
 * stamping for every outbound command (ADR-0007), and the
 * recovery-tier dispatch handlers (ADR-0008).
 *
 * Per ADR-0006, lrmh_seq sends commands to lrmh_ctrl/lrmh_sense/
 * lrmh_proc/lrmh_report and never touches the HAL directly. The
 * sequencer is policy; lrmh_ctrl is mechanism.
 *
 * Per ADR-0001 (apps unchanged across environments), lrmh_seq does
 * not call any sim or hardware step function: the sim runs as a
 * separate process and steps itself.
 */

#include "lrmh_seq_app.h"  /* transitively includes lrmh_cmd_msg.h */
#include "lrmh_ctrl_msg.h"
#include "lrmh_sense_msg.h"
#include "lrmh_proc_msg.h"
#include "lrmh_report_msg.h"
#include "lrmh_fault_msg.h"

#include <string.h>

/* ============================================================
 * Global app data (visible to standalone test)
 * ============================================================ */
LRMH_SEQ_AppData_t g_SeqApp;

/* ============================================================
 * Forward declarations
 * ============================================================ */
static int32 LRMH_SEQ_Init(void);
static void  LRMH_SEQ_ProcessMsg(const CFE_SB_Buffer_t *BufPtr);
static void  LRMH_SEQ_RunPhaseMachine(void);
static void  LRMH_SEQ_PublishStatus(void);

static void  LRMH_SEQ_SendCtrlCmd(uint8 mode, float target_pos, float target_force,
                                  float rate, float timeout);
static void  LRMH_SEQ_SendSenseCmd(uint8 start_stop);
static void  LRMH_SEQ_SendProcTrigger(uint8 phase);
static void  LRMH_SEQ_SendProcSegmentStart(uint8 phase);
static void  LRMH_SEQ_SendCtrlAndWait(uint8 mode, float target_pos, float target_force,
                                      float rate, float timeout);
static uint64 LRMH_SEQ_NowUs(void);
static void  LRMH_SEQ_SendReportTrigger(void);
static void  LRMH_SEQ_SendSafeBroadcast(void);
static void  LRMH_SEQ_SendCmdNotify(uint8 notify_type);
static void  LRMH_SEQ_HandleManualCtrl(const LRMH_SeqManualCtrlCmd_t *cmd);

/* ============================================================
 * App main
 * ============================================================ */
void LRMH_SEQ_AppMain(void)
{
    int32 Status;
    CFE_SB_Buffer_t *BufPtr = NULL;
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = LRMH_SEQ_Init();
    if (Status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog("LRMH_SEQ: Init failed\n");
        CFE_ES_ExitApp(CFE_ES_RunStatus_APP_ERROR);
        return;
    }

    CFE_EVS_SendEvent(LRMH_SEQ_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "LRMH_SEQ: initialized, Phase=IDLE");

    while (CFE_ES_RunLoop(&RunStatus) == true) {
        /* Pend with timeout — wakeup MID drives periodic execution */
        Status = CFE_SB_ReceiveBuffer(&BufPtr, g_SeqApp.CmdPipe, 100);
        if (Status == CFE_SUCCESS && BufPtr != NULL) {
            LRMH_SEQ_ProcessMsg(BufPtr);
        }
        else if (Status != CFE_SB_TIME_OUT) {
            /* A software-bus receive error is not something the app can
             * recover from in place: event it and let ES restart the app
             * (HS policy) rather than spin on a broken pipe. */
            g_SeqApp.ErrCount++;
            CFE_EVS_SendEvent(LRMH_SEQ_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "LRMH_SEQ: SB receive error 0x%08lX, exiting",
                              (unsigned long)Status);
            RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
        LRMH_SEQ_RunPhaseMachine();
    }

    CFE_ES_ExitApp(RunStatus);
}

/* ============================================================
 * Initialization
 *
 * Cold-start path: bumps the sequence-token epoch (ADR-0007)
 * and broadcasts LRMH_TOKEN_RESET_MID. lrmh_cmd does the same
 * on its own init; both paths use the shared seq_token library.
 * ============================================================ */
static int32 LRMH_SEQ_Init(void)
{
    int32 Status;

    memset(&g_SeqApp, 0, sizeof(g_SeqApp));
    g_SeqApp.Phase = LRMH_PHASE_IDLE;
    g_SeqApp.QualityFlag = LRMH_QUALITY_VALID;

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) return Status;

    Status = CFE_SB_CreatePipe(&g_SeqApp.CmdPipe,
                               LRMH_SEQ_PIPE_DEPTH, "LRMH_SEQ_PIPE");
    if (Status != CFE_SUCCESS) return Status;

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEQ_CMD_MID),
                     g_SeqApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_FAULT_TO_SEQ_MID),
                     g_SeqApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEQ_WAKEUP_MID),
                     g_SeqApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_CTRL_STATUS_MID),
                     g_SeqApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_PROC_TLM_MID),
                     g_SeqApp.CmdPipe);

    /* Cold-start epoch bump per ADR-0007 */
    LRMH_SeqToken_BumpEpoch(LRMH_TOKEN_RESET_COLD_START);

    return CFE_SUCCESS;
}

/* ============================================================
 * Message router
 * ============================================================ */
static void LRMH_SEQ_ProcessMsg(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_SB_MsgId_t MsgId;
    CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId)) {
    case LRMH_SEQ_CMD_MID: {
        CFE_MSG_FcnCode_t FcnCode;
        CFE_MSG_GetFcnCode(&BufPtr->Msg, &FcnCode);
        switch (FcnCode) {
        case LRMH_SEQ_NOOP_CC:
            g_SeqApp.CmdCount++;
            CFE_EVS_SendEvent(LRMH_SEQ_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "LRMH_SEQ: NOOP Cmd=%u Err=%u Phase=%u",
                              (unsigned)g_SeqApp.CmdCount, (unsigned)g_SeqApp.ErrCount,
                              (unsigned)g_SeqApp.Phase);
            break;
        case LRMH_SEQ_RESET_COUNTERS_CC:
            g_SeqApp.CmdCount = 0;
            g_SeqApp.ErrCount = 0;
            CFE_EVS_SendEvent(LRMH_SEQ_RESET_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "LRMH_SEQ: counters reset");
            break;
        case LRMH_SEQ_START_SITE_TEST_CC:
            LRMH_SEQ_HandleStartTest((const LRMH_StartSiteTestCmd_t *)BufPtr);
            break;
        case LRMH_SEQ_RESUME_CC:
            /* T1 RESUME_PHASE — re-enter current phase */
            LRMH_SEQ_HandleRecovery(0, LRMH_TIER_T1_RESUME);
            break;
        case LRMH_SEQ_SKIP_CC:
            LRMH_SEQ_HandleRecovery(0, LRMH_TIER_T2_SKIP);
            break;
        case LRMH_SEQ_ABORT_CC:
            LRMH_SEQ_HandleRecovery(0, LRMH_TIER_T3_SAFE);
            break;
        case LRMH_SEQ_MANUAL_CTRL_CC:
            LRMH_SEQ_HandleManualCtrl((const LRMH_SeqManualCtrlCmd_t *)BufPtr);
            break;
        default:
            g_SeqApp.ErrCount++;
            CFE_EVS_SendEvent(LRMH_SEQ_FCN_ERR_EID, CFE_EVS_EventType_ERROR,
                              "LRMH_SEQ: unknown cc %u", (unsigned)FcnCode);
            break;
        }
        break;
    }
    case LRMH_FAULT_TO_SEQ_MID: {
        const LRMH_SeqRecoveryCmd_t *rec = (const LRMH_SeqRecoveryCmd_t *)BufPtr;
        LRMH_SEQ_HandleRecovery(rec->FaultCode, rec->Tier);
        break;
    }
    case LRMH_SEQ_WAKEUP_MID:
        /* Phase machine runs after this returns */
        break;
    case LRMH_CTRL_STATUS_MID: {
        /* lrmh_ctrl publishes status every cycle; only the
         * CommandComplete bit means the in-flight command is done. */
        const LRMH_CtrlStatusTlm_t *st =
            (const LRMH_CtrlStatusTlm_t *)BufPtr;
        if (st->CommandComplete) {
            g_SeqApp.WaitingForCtrl = 0;
        }
        break;
    }
    case LRMH_PROC_TLM_MID:
        /* One phase result per trigger sent; PROCESSING waits until
         * every trigger has been answered before REPORTING. */
        if (g_SeqApp.WaitingForProc > 0) {
            g_SeqApp.WaitingForProc--;
        }
        break;
    default:
        break;
    }
}

/* ============================================================
 * StartSiteTest handler
 *
 * Latches all 2.0 fields from the dispatch command, bumps the
 * sequence-token epoch, broadcasts the reset, and transitions
 * to PRECHECK.
 * ============================================================ */
bool LRMH_SEQ_HandleStartTest(const LRMH_StartSiteTestCmd_t *cmd)
{
    /* Phase precondition: sequencer must be IDLE */
    if (g_SeqApp.Phase != LRMH_PHASE_IDLE) {
        g_SeqApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_SEQ_PHASE_REJECT_EID, CFE_EVS_EventType_ERROR,
                          "LRMH_SEQ: START rejected, Phase=%u (need IDLE)",
                          (unsigned)g_SeqApp.Phase);
        return false;
    }

    /* Latch test parameters */
    g_SeqApp.SiteId               = cmd->SiteId;
    g_SeqApp.SessionId            = cmd->SessionId;
    g_SeqApp.SequenceNumber       = cmd->SequenceNumber;
    g_SeqApp.PairedSiteId         = cmd->PairedSiteId;
    g_SeqApp.TestProfileId        = cmd->TestProfileId;
    g_SeqApp.ToolId               = cmd->ToolId;
    g_SeqApp.EnableRepeatLoad     = cmd->EnableRepeatLoad;
    g_SeqApp.EnablePostProfile    = cmd->EnablePostProfile;
    g_SeqApp.EnableTraction       = cmd->EnableTraction;
    g_SeqApp.DispatchFlags        = cmd->DispatchFlags;
    g_SeqApp.MaxDepth_m           = cmd->MaxDepth_m;
    g_SeqApp.MaxLoad_N            = cmd->MaxLoad_N;
    g_SeqApp.MicroSiteOffsetX_mm  = cmd->MicroSiteOffsetX_mm;
    g_SeqApp.MicroSiteOffsetY_mm  = cmd->MicroSiteOffsetY_mm;

    /* RepeatLoadCount: 0 = default 3 cycles per ADR-0014 */
    g_SeqApp.RepeatCyclesTarget = (cmd->RepeatLoadCount > 0)
                                  ? cmd->RepeatLoadCount : 3;
    g_SeqApp.RepeatCyclesDone   = 0;

    /* Reset runtime */
    g_SeqApp.SampleCount             = 0;
    g_SeqApp.ElapsedTime_s           = 0.0f;
    g_SeqApp.PhaseStartTime_s        = 0.0f;
    {
        /* Stamp test start time so lrmh_report can record duration. */
        CFE_TIME_SysTime_t now = CFE_TIME_GetTime();
        g_SeqApp.TestStartTime_us =
            (uint64)now.Seconds * 1000000ULL +
            (((uint64)now.Subseconds * 1000000ULL) >> 32);
    }
    g_SeqApp.WaitingForCtrl          = 0;
    g_SeqApp.WaitingForProc          = 0;
    g_SeqApp.PhaseStep               = 0;
    g_SeqApp.CtrlDeadline_s          = 0.0f;
    g_SeqApp.QualityFlag             = LRMH_QUALITY_VALID;
    g_SeqApp.T1AttemptsCurrentPhase  = 0;

    /* Bump token epoch — fresh test, fresh epoch (ADR-0007) */
    LRMH_SeqToken_BumpEpoch(LRMH_TOKEN_RESET_PHASE_ADVANCE);

    g_SeqApp.CmdCount++;
    CFE_EVS_SendEvent(LRMH_SEQ_TEST_START_EID, CFE_EVS_EventType_INFORMATION,
                      "LRMH_SEQ: New test Session=%u Seq=%u Site=%u",
                      (unsigned)cmd->SessionId,
                      (unsigned)cmd->SequenceNumber,
                      (unsigned)cmd->SiteId);

    LRMH_SEQ_TransitionTo(LRMH_PHASE_PRECHECK);
    return true;
}

/* ============================================================
 * Manual actuator command from ground
 *
 * Allowed only when phase == IDLE so a manual command can never
 * race the sequencer's own LRMH_SEQ_TO_CTRL_MID dispatch during a
 * test. lrmh_seq stamps the sequence token (ADR-0007 keeps tokens
 * FSW-internal — ground does not fabricate them).
 * ============================================================ */
static void LRMH_SEQ_HandleManualCtrl(const LRMH_SeqManualCtrlCmd_t *cmd)
{
    if (g_SeqApp.Phase != LRMH_PHASE_IDLE) {
        g_SeqApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_SEQ_MANUAL_CTRL_REJECT_EID,
                          CFE_EVS_EventType_ERROR,
                          "LRMH_SEQ: manual ctrl rejected, "
                          "Phase=%u (need IDLE)",
                          (unsigned)g_SeqApp.Phase);
        return;
    }
    LRMH_SEQ_SendCtrlCmd(cmd->Mode, cmd->TargetPosition_m, cmd->TargetForce_N,
                         cmd->RateLimit, cmd->Timeout_s);
    g_SeqApp.CmdCount++;
    CFE_EVS_SendEvent(LRMH_SEQ_MANUAL_CTRL_EID, CFE_EVS_EventType_INFORMATION,
                      "LRMH_SEQ: manual ctrl mode=%u pos=%.3f force=%.1f",
                      (unsigned)cmd->Mode,
                      (double)cmd->TargetPosition_m,
                      (double)cmd->TargetForce_N);
}

/* ============================================================
 * Recovery tier dispatch (ADR-0008)
 * ============================================================ */
void LRMH_SEQ_HandleRecovery(uint8 fault_code, uint8 tier)
{
    (void)fault_code;

    switch ((LRMH_RecoveryTier_t)tier) {

    case LRMH_TIER_T1_RESUME: {
        /* Re-enter the current phase. The token's seq_count
         * advances but seq_state does not (the receiver sees a
         * fresh command for the same state). */
        if (g_SeqApp.Phase == LRMH_PHASE_IDLE ||
            g_SeqApp.Phase >= LRMH_PHASE_PROCESSING) {
            CFE_EVS_SendEvent(LRMH_SEQ_RECOVERY_REJECT_EID,
                              CFE_EVS_EventType_ERROR,
                              "LRMH_SEQ: T1 RESUME rejected in Phase=%u",
                              (unsigned)g_SeqApp.Phase);
            return;
        }
        g_SeqApp.T1AttemptsCurrentPhase++;
        if (g_SeqApp.T1AttemptsCurrentPhase >
            LRMH_T1_MAX_ATTEMPTS_PER_PHASE) {
            /* Escalate to T2 — too many T1 attempts in this phase */
            LRMH_SEQ_HandleRecovery(fault_code, LRMH_TIER_T2_SKIP);
            return;
        }
        /* Re-enter the phase from its first step; the in-flight
         * command, if any, is superseded by the re-issued one. */
        g_SeqApp.WaitingForCtrl = 0;
        g_SeqApp.PhaseStep      = 0;
        CFE_EVS_SendEvent(LRMH_SEQ_RECOVERY_T1_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "LRMH_SEQ: T1 RESUME phase %u (attempt %u)",
                          (unsigned)g_SeqApp.Phase,
                          (unsigned)g_SeqApp.T1AttemptsCurrentPhase);
        /* Phase machine re-executes on the next tick */
        return;
    }

    case LRMH_TIER_T2_SKIP: {
        if (g_SeqApp.Phase == LRMH_PHASE_IDLE) return;
        g_SeqApp.QualityFlag    = LRMH_QUALITY_PARTIAL;
        g_SeqApp.WaitingForCtrl = 0;   /* PROCESSING issues its own retract */
        CFE_EVS_SendEvent(LRMH_SEQ_RECOVERY_T2_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "LRMH_SEQ: T2 SKIP_SITE from Phase=%u",
                          (unsigned)g_SeqApp.Phase);
        LRMH_SEQ_TransitionTo(LRMH_PHASE_PROCESSING);
        return;
    }

    case LRMH_TIER_T3_SAFE: {
        g_SeqApp.QualityFlag    = LRMH_QUALITY_FAULT;
        g_SeqApp.WaitingForCtrl = 0;
        g_SeqApp.WaitingForProc = 0;
        CFE_EVS_SendEvent(LRMH_SEQ_RECOVERY_T3_EID,
                          CFE_EVS_EventType_CRITICAL,
                          "LRMH_SEQ: T3 SAFE from Phase=%u",
                          (unsigned)g_SeqApp.Phase);
        /* Stop sensor acquisition (carries SAFE token, accepted
         * unconditionally per ADR-0007 rule 1) */
        LRMH_SEQ_SendSenseCmd(0);
        /* Retract actuator */
        LRMH_SEQ_SendCtrlCmd(LRMH_CTRL_MODE_RETRACT, 0.0f, 0.0f,
                             0.008f, 30.0f);
        /* Broadcast SAFE token to all receivers */
        LRMH_SEQ_SendSafeBroadcast();
        /* Report whatever was collected with quality FAULT, unless
         * no site was running. */
        if (g_SeqApp.Phase != LRMH_PHASE_IDLE) {
            LRMH_SEQ_SendReportTrigger();
        }
        /* Reset to IDLE; lrmh_cmd's mode FSM moves to SAFE in
         * parallel through its own ABORT/SAFE handler. */
        LRMH_SEQ_TransitionTo(LRMH_PHASE_IDLE);
        return;
    }

    default:
        g_SeqApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_SEQ_RECOVERY_REJECT_EID,
                          CFE_EVS_EventType_ERROR,
                          "LRMH_SEQ: unknown recovery tier %u",
                          (unsigned)tier);
        return;
    }
}

/* ============================================================
 * Phase transition
 * ============================================================ */
void LRMH_SEQ_TransitionTo(LRMH_Phase_t new_phase)
{
    g_SeqApp.PreviousPhase = g_SeqApp.Phase;
    g_SeqApp.Phase = new_phase;
    g_SeqApp.PhaseStartTime_s = g_SeqApp.ElapsedTime_s;
    g_SeqApp.PhaseStep = 0;

    /* T1 attempt counter is per-phase; reset on any phase change
     * other than a self-transition (RESUME_PHASE leaves the phase
     * the same and explicitly increments via HandleRecovery). */
    if (new_phase != g_SeqApp.PreviousPhase) {
        g_SeqApp.T1AttemptsCurrentPhase = 0;
    }

    /* Update the seq_token's current phase so the next Stamp()
     * call uses the new value (ADR-0007). */
    LRMH_SeqToken_SetPhase((uint8)new_phase);

    /* Notify lrmh_fault so it can reset T1AttemptsCurrentPhase
     * per ADR-0008. */
    {
        LRMH_FaultPhaseAdvanceCmd_t pa_cmd;
        memset(&pa_cmd, 0, sizeof(pa_cmd));
        CFE_MSG_Init(CFE_MSG_PTR(pa_cmd.CmdHeader),
                     CFE_SB_ValueToMsgId(LRMH_FAULT_CMD_MID), sizeof(pa_cmd));
        CFE_MSG_SetFcnCode(CFE_MSG_PTR(pa_cmd.CmdHeader),
                           LRMH_FAULT_PHASE_ADVANCE_CC);
        pa_cmd.NewPhase = (uint8)new_phase;
        CFE_SB_TransmitMsg(CFE_MSG_PTR(pa_cmd.CmdHeader), true);
    }

    CFE_EVS_SendEvent(LRMH_SEQ_PHASE_ADVANCE_EID, CFE_EVS_EventType_INFORMATION,
                      "LRMH_SEQ: Phase %u → %u",
                      (unsigned)g_SeqApp.PreviousPhase,
                      (unsigned)new_phase);
}

/* ============================================================
 * Phase machine
 *
 * Runs after every message or pipe timeout. A phase is a short
 * step sequence: each step issues at most one actuator command
 * and the machine then waits for lrmh_ctrl to report completion
 * (or for the command's deadline), so the phase label always
 * names the motion in progress. Token stamping happens inside
 * the Send helpers via LRMH_SeqToken_Stamp.
 *
 *   PRECHECK      actuator self-test (HAL Init via lrmh_ctrl)
 *   POSITIONING   start acquisition; 5 N contact at 3 N/s
 *   BASELINE_PEN  position to MaxDepth under the MaxLoad clamp;
 *                 trigger processing of the baseline samples
 *   LOAD_RAMP     retract to home, then ramp force to MaxLoad
 *                 (one load-sinkage cycle)
 *   REPEAT_LOAD   RepeatCyclesTarget more unload/load cycles;
 *                 trigger processing of all load samples
 *   POST_PEN      retract to home, then a second penetration to
 *                 MaxDepth down the same hole; trigger processing
 *   TRACTION      stub, passes through
 *   PROCESSING    stop acquisition, retract, wait for every
 *                 processing result
 *   REPORTING     trigger the site record, notify lrmh_cmd, IDLE
 * ============================================================ */
void LRMH_SEQ_AdvancePhase(void)
{
    /* Single-step the phase machine — used by tests to drive
     * deterministic phase advance without the wakeup loop. */
    LRMH_SEQ_RunPhaseMachine();
}

static uint64 LRMH_SEQ_NowUs(void)
{
    CFE_TIME_SysTime_t now = CFE_TIME_GetTime();
    return (uint64)now.Seconds * 1000000ULL +
           (((uint64)now.Subseconds * 1000000ULL) >> 32);
}

/* Issue an actuator command and arm its deadline. */
static void LRMH_SEQ_SendCtrlAndWait(uint8 mode, float target_pos, float target_force,
                                     float rate, float timeout)
{
    LRMH_SEQ_SendCtrlCmd(mode, target_pos, target_force, rate, timeout);
    g_SeqApp.WaitingForCtrl = 1;
    g_SeqApp.CtrlDeadline_s = g_SeqApp.ElapsedTime_s + timeout;
}

/* Time budget for a force ramp to MaxLoad at `rate_nps`, plus margin. */
static float LRMH_SEQ_RampTimeout(float target_force, float rate_nps)
{
    float ramp_s = (rate_nps > 0.0f) ? target_force / rate_nps : 0.0f;
    return ramp_s + LRMH_ACTUATOR_TIMEOUT_S;
}

/* Where the test goes after the load phases. */
static LRMH_Phase_t LRMH_SEQ_AfterLoadPhases(void)
{
    if (g_SeqApp.EnablePostProfile) return LRMH_PHASE_POST_PEN;
    if (g_SeqApp.EnableTraction)    return LRMH_PHASE_TRACTION;
    return LRMH_PHASE_PROCESSING;
}

static void LRMH_SEQ_RunPhaseMachine(void)
{
    /* Elapsed wall-clock time since the test started. Outside a
     * test (or in the standalone tests, where the stub clock is 0)
     * the value is left alone. */
    if (g_SeqApp.TestStartTime_us > 0) {
        uint64 now_us = LRMH_SEQ_NowUs();
        if (now_us > g_SeqApp.TestStartTime_us) {
            g_SeqApp.ElapsedTime_s =
                (float)(now_us - g_SeqApp.TestStartTime_us) / 1e6f;
        }
    }

    /* An actuator command is in flight: wait for completion or
     * for its deadline. A move that never completes is treated as
     * a T2 skip of the site; during PROCESSING the retract is
     * simply abandoned so the site still gets reported. */
    if (g_SeqApp.WaitingForCtrl) {
        if (g_SeqApp.ElapsedTime_s > g_SeqApp.CtrlDeadline_s) {
            g_SeqApp.ErrCount++;
            CFE_EVS_SendEvent(LRMH_SEQ_PHASE_TIMEOUT_EID, CFE_EVS_EventType_ERROR,
                              "LRMH_SEQ: actuator command timed out in Phase=%u step %u",
                              (unsigned)g_SeqApp.Phase, (unsigned)g_SeqApp.PhaseStep);
            g_SeqApp.WaitingForCtrl = 0;
            if (g_SeqApp.Phase < LRMH_PHASE_PROCESSING) {
                LRMH_SEQ_HandleRecovery(0, LRMH_TIER_T2_SKIP);
            }
        }
        LRMH_SEQ_PublishStatus();
        return;
    }

    switch (g_SeqApp.Phase) {

    case LRMH_PHASE_IDLE:
        /* Nothing to do until START_SITE_TEST arrives */
        break;

    case LRMH_PHASE_PRECHECK:
        /* Actuator self-test through lrmh_ctrl (HAL Init). */
        if (g_SeqApp.PhaseStep == 0) {
            LRMH_SEQ_SendCtrlAndWait(LRMH_CTRL_MODE_PRECHECK, 0.0f, 0.0f, 0.0f,
                                     LRMH_ACTUATOR_TIMEOUT_S);
            g_SeqApp.PhaseStep = 1;
        } else {
            LRMH_SEQ_TransitionTo(LRMH_PHASE_POSITIONING);
        }
        break;

    case LRMH_PHASE_POSITIONING:
        /* Start acquisition, then force-mode contact at 5 N with a
         * 3 N/s ramp. RateLimit is N/s in FORCE mode, m/s in
         * POSITION mode (lrmh_ctrl_msg.h LRMH_CtrlCmd_t contract). */
        if (g_SeqApp.PhaseStep == 0) {
            LRMH_SEQ_SendSenseCmd(1);
            LRMH_SEQ_SendProcSegmentStart((uint8)LRMH_PHASE_POSITIONING);
            LRMH_SEQ_SendCtrlAndWait(LRMH_CTRL_MODE_FORCE, 0.0f, 5.0f, 3.0f,
                                     LRMH_ACTUATOR_TIMEOUT_S);
            g_SeqApp.PhaseStep = 1;
        } else {
            LRMH_SEQ_TransitionTo(LRMH_PHASE_BASELINE_PEN);
        }
        break;

    case LRMH_PHASE_BASELINE_PEN:
        /* Position-mode penetration to MaxDepth (RMH-F-002) with
         * MaxLoad_N as the force clamp (ADR-0004: the move ends at
         * depth, at the clamp, or at its deadline). */
        if (g_SeqApp.PhaseStep == 0) {
            LRMH_SEQ_SendProcSegmentStart((uint8)LRMH_PHASE_BASELINE_PEN);
            LRMH_SEQ_SendCtrlAndWait(LRMH_CTRL_MODE_POSITION, g_SeqApp.MaxDepth_m,
                                     g_SeqApp.MaxLoad_N, 0.005f, 120.0f);
            g_SeqApp.PhaseStep = 1;
        } else {
            LRMH_SEQ_SendProcTrigger((uint8)LRMH_PHASE_BASELINE_PEN);
            LRMH_SEQ_TransitionTo(LRMH_PHASE_LOAD_RAMP);
        }
        break;

    case LRMH_PHASE_LOAD_RAMP:
        /* Unload (retract to home), then ramp force to MaxLoad: one
         * load-sinkage cycle (RMH-F-001). The load samples are
         * processed with the repeat cycles, or on their own when
         * repeat load is disabled. */
        if (g_SeqApp.PhaseStep == 0) {
            LRMH_SEQ_SendCtrlAndWait(LRMH_CTRL_MODE_RETRACT, 0.0f, 0.0f, 0.008f,
                                     LRMH_ACTUATOR_TIMEOUT_S);
            g_SeqApp.PhaseStep = 1;
        } else if (g_SeqApp.PhaseStep == 1) {
            LRMH_SEQ_SendProcSegmentStart((uint8)LRMH_PHASE_LOAD_RAMP);
            LRMH_SEQ_SendCtrlAndWait(LRMH_CTRL_MODE_FORCE, 0.0f, g_SeqApp.MaxLoad_N,
                                     3.0f, LRMH_SEQ_RampTimeout(g_SeqApp.MaxLoad_N, 3.0f));
            g_SeqApp.PhaseStep = 2;
        } else if (g_SeqApp.EnableRepeatLoad) {
            LRMH_SEQ_TransitionTo(LRMH_PHASE_REPEAT_LOAD);
        } else {
            LRMH_SEQ_SendProcTrigger((uint8)LRMH_PHASE_LOAD_RAMP);
            LRMH_SEQ_TransitionTo(LRMH_SEQ_AfterLoadPhases());
        }
        break;

    case LRMH_PHASE_REPEAT_LOAD:
        /* RepeatCyclesTarget cycles of unload (retract) and reload
         * to MaxLoad (RMH-F-001 and RMH-F-003). */
        if (g_SeqApp.PhaseStep == 0) {
            LRMH_SEQ_SendCtrlAndWait(LRMH_CTRL_MODE_RETRACT, 0.0f, 0.0f, 0.008f,
                                     LRMH_ACTUATOR_TIMEOUT_S);
            g_SeqApp.PhaseStep = 1;
        } else if (g_SeqApp.PhaseStep == 1) {
            LRMH_SEQ_SendCtrlAndWait(LRMH_CTRL_MODE_FORCE, 0.0f, g_SeqApp.MaxLoad_N,
                                     3.0f, LRMH_SEQ_RampTimeout(g_SeqApp.MaxLoad_N, 3.0f));
            g_SeqApp.PhaseStep = 2;
        } else {
            g_SeqApp.RepeatCyclesDone++;
            if (g_SeqApp.RepeatCyclesDone < g_SeqApp.RepeatCyclesTarget) {
                g_SeqApp.PhaseStep = 0;   /* next cycle, same phase */
            } else {
                LRMH_SEQ_SendProcTrigger((uint8)LRMH_PHASE_REPEAT_LOAD);
                LRMH_SEQ_TransitionTo(LRMH_SEQ_AfterLoadPhases());
            }
        }
        break;

    case LRMH_PHASE_POST_PEN:
        /* Retract to home, then a second penetration to MaxDepth
         * down the same hole (RMH-F-003). */
        if (g_SeqApp.PhaseStep == 0) {
            LRMH_SEQ_SendCtrlAndWait(LRMH_CTRL_MODE_RETRACT, 0.0f, 0.0f, 0.008f,
                                     LRMH_ACTUATOR_TIMEOUT_S);
            g_SeqApp.PhaseStep = 1;
        } else if (g_SeqApp.PhaseStep == 1) {
            LRMH_SEQ_SendProcSegmentStart((uint8)LRMH_PHASE_POST_PEN);
            LRMH_SEQ_SendCtrlAndWait(LRMH_CTRL_MODE_POSITION, g_SeqApp.MaxDepth_m,
                                     g_SeqApp.MaxLoad_N, 0.005f, 120.0f);
            g_SeqApp.PhaseStep = 2;
        } else {
            LRMH_SEQ_SendProcTrigger((uint8)LRMH_PHASE_POST_PEN);
            LRMH_SEQ_TransitionTo(g_SeqApp.EnableTraction ? LRMH_PHASE_TRACTION
                                                          : LRMH_PHASE_PROCESSING);
        }
        break;

    case LRMH_PHASE_TRACTION:
        /* Not implemented (ADR-0004); passes straight through. */
        LRMH_SEQ_TransitionTo(LRMH_PHASE_PROCESSING);
        break;

    case LRMH_PHASE_PROCESSING:
        /* Stop acquisition and retract, then wait until every
         * processing trigger has been answered so REPORTING sees the
         * complete set of phase results. */
        if (g_SeqApp.PhaseStep == 0) {
            LRMH_SEQ_SendSenseCmd(0);
            LRMH_SEQ_SendCtrlAndWait(LRMH_CTRL_MODE_RETRACT, 0.0f, 0.0f, 0.008f,
                                     LRMH_ACTUATOR_TIMEOUT_S);
            g_SeqApp.PhaseStep = 1;
        } else if (g_SeqApp.WaitingForProc == 0) {
            LRMH_SEQ_TransitionTo(LRMH_PHASE_REPORTING);
        }
        break;

    case LRMH_PHASE_REPORTING:
        LRMH_SEQ_SendReportTrigger();
        CFE_EVS_SendEvent(LRMH_SEQ_TEST_COMPLETE_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "LRMH_SEQ: COMPLETE Site=%u Quality=%u",
                          (unsigned)g_SeqApp.SiteId,
                          (unsigned)g_SeqApp.QualityFlag);
        /* Drive lrmh_cmd's mode FSM TESTING → STANDBY (ADR-0004). */
        LRMH_SEQ_SendCmdNotify(LRMH_SEQ_NOTIFY_TEST_COMPLETE);
        LRMH_SEQ_TransitionTo(LRMH_PHASE_IDLE);
        break;

    default:
        break;
    }

    LRMH_SEQ_PublishStatus();
}

/* ============================================================
 * SB send helpers — every command stamped with current token
 * ============================================================ */
static void LRMH_SEQ_SendCtrlCmd(uint8 mode, float target_pos, float target_force,
                                 float rate, float timeout)
{
    LRMH_CtrlCmd_t cmd;
    LRMH_SeqToken_t tok;
    LRMH_SeqToken_Stamp(&tok);

    CFE_MSG_Init(CFE_MSG_PTR(cmd.CmdHeader),
                 CFE_SB_ValueToMsgId(LRMH_SEQ_TO_CTRL_MID), sizeof(cmd));
    cmd.SeqTokenPacked   = LRMH_SEQ_TOKEN_PACK(tok);
    cmd.Mode             = mode;
    cmd.TargetPosition_m = target_pos;
    cmd.TargetForce_N    = target_force;
    cmd.RateLimit        = rate;
    cmd.Timeout_s        = timeout;
    CFE_SB_TransmitMsg(CFE_MSG_PTR(cmd.CmdHeader), true);
}

static void LRMH_SEQ_SendSenseCmd(uint8 start_stop)
{
    LRMH_SenseSeqCmd_t cmd;
    LRMH_SeqToken_t tok;
    LRMH_SeqToken_Stamp(&tok);

    CFE_MSG_Init(CFE_MSG_PTR(cmd.CmdHeader),
                 CFE_SB_ValueToMsgId(LRMH_SEQ_TO_SENSE_MID), sizeof(cmd));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(cmd.CmdHeader),
                       start_stop ? LRMH_SENSE_START_CC : LRMH_SENSE_STOP_CC);
    cmd.SeqTokenPacked = LRMH_SEQ_TOKEN_PACK(tok);
    CFE_SB_TransmitMsg(CFE_MSG_PTR(cmd.CmdHeader), true);
}

static void LRMH_SEQ_SendProcMsg(uint8 fcn, uint8 phase)
{
    LRMH_ProcTriggerCmd_t cmd;
    LRMH_SeqToken_t tok;
    LRMH_SeqToken_Stamp(&tok);

    CFE_MSG_Init(CFE_MSG_PTR(cmd.CmdHeader),
                 CFE_SB_ValueToMsgId(LRMH_SEQ_TO_PROC_MID), sizeof(cmd));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(cmd.CmdHeader), fcn);
    cmd.SeqTokenPacked      = LRMH_SEQ_TOKEN_PACK(tok);
    cmd.SiteId              = g_SeqApp.SiteId;
    cmd.PhaseComplete       = phase;
    cmd.Reserved            = 0;
    cmd.SampleCount         = g_SeqApp.SampleCount;
    CFE_SB_TransmitMsg(CFE_MSG_PTR(cmd.CmdHeader), true);
}

/* Process the samples accumulated since the last segment start as `phase`. */
static void LRMH_SEQ_SendProcTrigger(uint8 phase)
{
    LRMH_SEQ_SendProcMsg(LRMH_PROC_RUN_PHASE_CC, phase);
    g_SeqApp.WaitingForProc++;
}

/* Mark the start of a measured segment: lrmh_proc discards what it
 * has buffered (a retract, for instance) and stamps the phase start
 * time that per-phase durations and stale-sample rejection use. */
static void LRMH_SEQ_SendProcSegmentStart(uint8 phase)
{
    LRMH_SEQ_SendProcMsg(LRMH_PROC_PHASE_START_CC, phase);
}

static void LRMH_SEQ_SendReportTrigger(void)
{
    LRMH_ReportTriggerCmd_t cmd;
    LRMH_SeqToken_t tok;
    LRMH_SeqToken_Stamp(&tok);

    CFE_MSG_Init(CFE_MSG_PTR(cmd.CmdHeader),
                 CFE_SB_ValueToMsgId(LRMH_SEQ_TO_REPORT_MID), sizeof(cmd));
    cmd.SeqTokenPacked = LRMH_SEQ_TOKEN_PACK(tok);
    cmd.SiteId         = g_SeqApp.SiteId;
    cmd.QualityFlag    = g_SeqApp.QualityFlag;
    cmd.Reserved       = 0;
    cmd.SessionId      = g_SeqApp.SessionId;
    cmd.SequenceNumber = g_SeqApp.SequenceNumber;
    cmd.PairedSiteId   = g_SeqApp.PairedSiteId;
    cmd.Reserved16     = 0;
    cmd.TestProfileId  = g_SeqApp.TestProfileId;
    cmd.ToolId         = g_SeqApp.ToolId;
    memset(cmd.Reserved8b, 0, sizeof(cmd.Reserved8b));
    cmd.TestStartTime_us = g_SeqApp.TestStartTime_us;
    CFE_SB_TransmitMsg(CFE_MSG_PTR(cmd.CmdHeader), true);
}

static void LRMH_SEQ_SendSafeBroadcast(void)
{
    LRMH_SenseSeqCmd_t cmd;
    LRMH_SeqToken_t tok;
    LRMH_SeqToken_Stamp(&tok);

    CFE_MSG_Init(CFE_MSG_PTR(cmd.CmdHeader),
                 CFE_SB_ValueToMsgId(LRMH_SEQ_SAFE_MID), sizeof(cmd));
    cmd.SeqTokenPacked = LRMH_SEQ_TOKEN_PACK(tok);
    CFE_SB_TransmitMsg(CFE_MSG_PTR(cmd.CmdHeader), true);
}

static void LRMH_SEQ_SendCmdNotify(uint8 notify_type)
{
    /* Notify lrmh_cmd of an FSM-relevant sequencer event so the
     * mode FSM (ADR-0004, owned by lrmh_cmd) can advance. No token:
     * this is a state event, not a sequenced data command. */
    LRMH_SeqToCmdNotify_t cmd;
    CFE_MSG_Init(CFE_MSG_PTR(cmd.CmdHeader),
                 CFE_SB_ValueToMsgId(LRMH_SEQ_TO_CMD_MID), sizeof(cmd));
    cmd.NotifyType = notify_type;
    cmd.Reserved[0] = cmd.Reserved[1] = cmd.Reserved[2] = 0;
    CFE_SB_TransmitMsg(CFE_MSG_PTR(cmd.CmdHeader), true);
}

/* ============================================================
 * HK status publication
 * ============================================================ */
static void LRMH_SEQ_PublishStatus(void)
{
    LRMH_SeqStatusTlm_t tlm;
    memset(&tlm, 0, sizeof(tlm));
    CFE_MSG_Init(CFE_MSG_PTR(tlm.TlmHeader),
                 CFE_SB_ValueToMsgId(LRMH_SEQ_STATUS_MID), sizeof(tlm));

    tlm.Phase                  = (uint8)g_SeqApp.Phase;
    tlm.PreviousPhase          = (uint8)g_SeqApp.PreviousPhase;
    tlm.QualityFlag            = g_SeqApp.QualityFlag;
    tlm.WaitingForCtrl         = g_SeqApp.WaitingForCtrl;
    tlm.SiteId                 = g_SeqApp.SiteId;
    tlm.SessionId              = g_SeqApp.SessionId;
    tlm.SequenceNumber         = g_SeqApp.SequenceNumber;
    tlm.Epoch                  = LRMH_SeqToken_GetCurrentEpoch();
    tlm.TotalSamples           = g_SeqApp.SampleCount;
    tlm.CmdCount               = g_SeqApp.CmdCount;
    tlm.ErrCount               = g_SeqApp.ErrCount;
    tlm.T1AttemptsCurrentPhase = g_SeqApp.T1AttemptsCurrentPhase;
    tlm.RepeatCyclesDone       = g_SeqApp.RepeatCyclesDone;
    tlm.RepeatCyclesTarget     = g_SeqApp.RepeatCyclesTarget;
    tlm.ElapsedTime_s          = g_SeqApp.ElapsedTime_s;
    tlm.PhaseStartTime_s       = g_SeqApp.PhaseStartTime_s;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(tlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(tlm.TlmHeader), true);
}
