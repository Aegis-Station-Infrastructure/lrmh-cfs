/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_ctrl_app.c
 * @brief LRMH lrmh_ctrl application — implementation
 *
 * Receives high-level motion commands from lrmh_seq, validates the
 * sequence token (ADR-0007), translates the command's Mode field
 * into the appropriate LRMH_HAL_* call (ADR-0010), and publishes
 * status TLM that lrmh_seq watches for completion.
 *
 * Every command arriving on LRMH_SEQ_TO_CTRL_MID is run through
 * LRMH_SeqToken_Validate before any HAL call; a stale command is
 * dropped with an INFO event and counted in StaleCount.
 *
 * LRMH_CTRL_CMD_MID (ground) carries NOOP and RESET only; ground
 * actuator commands go through lrmh_seq (ADR-0018).
 * LRMH_FAULT_TO_CTRL_MID is subscribed but reserved: nothing sends
 * it today (recovery goes through lrmh_seq).
 */

#include "lrmh_ctrl_app.h"

#include <string.h>

LRMH_CTRL_AppData_t g_CtrlApp;

/* ============================================================
 * Forward declarations
 * ============================================================ */
static int32 LRMH_CTRL_Init(void);
static void  LRMH_CTRL_ProcessMsg(const CFE_SB_Buffer_t *BufPtr);

/* ============================================================
 * App main
 * ============================================================ */
void LRMH_CTRL_AppMain(void)
{
    int32 Status;
    CFE_SB_Buffer_t *BufPtr = NULL;
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = LRMH_CTRL_Init();
    if (Status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog("LRMH_CTRL: Init failed\n");
        CFE_ES_ExitApp(CFE_ES_RunStatus_APP_ERROR);
        return;
    }

    CFE_EVS_SendEvent(LRMH_CTRL_INIT_INF_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_CTRL: initialized");

    while (CFE_ES_RunLoop(&RunStatus) == true) {
        Status = CFE_SB_ReceiveBuffer(&BufPtr, g_CtrlApp.CmdPipe, 100);
        if (Status == CFE_SUCCESS && BufPtr != NULL) {
            LRMH_CTRL_ProcessMsg(BufPtr);
        }
        else if (Status != CFE_SB_TIME_OUT) {
            /* A software-bus receive error is not something the app can
             * recover from in place: event it and let ES restart the app
             * (HS policy) rather than spin on a broken pipe. */
            g_CtrlApp.ErrCount++;
            CFE_EVS_SendEvent(LRMH_CTRL_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "LRMH_CTRL: SB receive error 0x%08lX, exiting",
                              (unsigned long)Status);
            RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
        LRMH_CTRL_PublishStatus();
    }

    CFE_ES_ExitApp(RunStatus);
}

/* ============================================================
 * Initialization
 * ============================================================ */
static int32 LRMH_CTRL_Init(void)
{
    int32 Status;

    memset(&g_CtrlApp, 0, sizeof(g_CtrlApp));

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) return Status;

    Status = CFE_SB_CreatePipe(&g_CtrlApp.CmdPipe,
                               LRMH_CTRL_PIPE_DEPTH, "LRMH_CTRL_PIPE");
    if (Status != CFE_SUCCESS) return Status;

    /* Subscriptions */
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_CTRL_CMD_MID),
                     g_CtrlApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEQ_TO_CTRL_MID),
                     g_CtrlApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_FAULT_TO_CTRL_MID),
                     g_CtrlApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEQ_SAFE_MID),
                     g_CtrlApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_TOKEN_RESET_MID),
                     g_CtrlApp.CmdPipe);

    return CFE_SUCCESS;
}

/* ============================================================
 * Token reset handler — clear accepted token state
 * ============================================================ */
void LRMH_CTRL_OnTokenReset(void)
{
    memset(&g_CtrlApp.AcceptedToken, 0, sizeof(g_CtrlApp.AcceptedToken));
    g_CtrlApp.ResetObserved = true;
    CFE_EVS_SendEvent(LRMH_CTRL_TOKEN_RESET_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_CTRL: token reset observed");
}

/* ============================================================
 * Token validation + dispatch (ADR-0007 receiver-side)
 *
 * Returns the validation result so tests can assert on it
 * directly. Accept-and-update of g_CtrlApp.AcceptedToken happens
 * inside LRMH_SeqToken_Validate per the ADR.
 * ============================================================ */
LRMH_TokenValidationResult_t LRMH_CTRL_ValidateAndDispatch(
    const LRMH_CtrlCmd_t *cmd)
{
    LRMH_SeqToken_t incoming;
    incoming.Epoch    = LRMH_SEQ_TOKEN_UNPACK_EPOCH(cmd->SeqTokenPacked);
    incoming.SeqState = LRMH_SEQ_TOKEN_UNPACK_STATE(cmd->SeqTokenPacked);
    incoming.SeqCount = LRMH_SEQ_TOKEN_UNPACK_COUNT(cmd->SeqTokenPacked);

    /* If we just observed a token reset, the next command is
     * accepted unconditionally and re-baselines the accepted
     * state (ADR-0007 rule 2). */
    if (g_CtrlApp.ResetObserved) {
        g_CtrlApp.AcceptedToken = incoming;
        g_CtrlApp.ResetObserved = false;
        LRMH_CTRL_HandleCommand(cmd, true);
        return LRMH_TOKEN_ACCEPT;
    }

    LRMH_TokenValidationResult_t result =
        LRMH_SeqToken_Validate(&incoming, &g_CtrlApp.AcceptedToken,
                               cmd->Mode == LRMH_CTRL_MODE_ESTOP ||
                               cmd->Mode == LRMH_CTRL_MODE_RETRACT);

    switch (result) {
    case LRMH_TOKEN_ACCEPT:
    case LRMH_TOKEN_UNCONDITIONAL_ACCEPT:
        LRMH_CTRL_HandleCommand(cmd, true);
        return result;

    case LRMH_TOKEN_REJECT_STALE_EPOCH:
    case LRMH_TOKEN_REJECT_STALE_STATE:
    case LRMH_TOKEN_REJECT_REORDER:
    default:
        g_CtrlApp.StaleCount++;
        CFE_EVS_SendEvent(LRMH_CTRL_STALE_CMD_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "LRMH_CTRL: stale cmd dropped (rule=%d, "
                          "incoming E=%u S=%u C=%u, accepted E=%u S=%u C=%u)",
                          (int)result,
                          (unsigned)incoming.Epoch,
                          (unsigned)incoming.SeqState,
                          (unsigned)incoming.SeqCount,
                          (unsigned)g_CtrlApp.AcceptedToken.Epoch,
                          (unsigned)g_CtrlApp.AcceptedToken.SeqState,
                          (unsigned)g_CtrlApp.AcceptedToken.SeqCount);
        return result;
    }
}

/* ============================================================
 * Command handler — translate Mode → HAL call
 *
 * Per ADR-0010: each mode maps to one HAL primitive. The
 * sequencer's command struct doesn't carry explicit safety
 * clamps; we use the platform defaults from lrmh_platform_cfg.h.
 * The motor controller below the HAL line is responsible for
 * enforcing the clamps in single-cycle time.
 * ============================================================ */
void LRMH_CTRL_HandleCommand(const LRMH_CtrlCmd_t *cmd, bool token_validated)
{
    int32 hal_status = LRMH_HAL_OK;
    (void)token_validated;  /* receivers may differ in handling later */

    switch (cmd->Mode) {

    case LRMH_CTRL_MODE_POSITION: {
        /* The force clamp is the command's TargetForce_N when it gives
         * one, else just inside the LC force watchpoint: the clamp is the
         * inner defence (ADR-0012), so a position move into hard ground
         * ends at the clamp, not as a FORCE_SATURATION trip. */
        LRMH_HAL_PositionCmd_t pos_cmd = {
            .target_stroke_m = cmd->TargetPosition_m,
            .max_force_n     = (cmd->TargetForce_N > 0.0f)
                               ? cmd->TargetForce_N
                               : LRMH_LC_FORCE_THRESHOLD_N - 20.0f,
            .max_speed_mps   = (cmd->RateLimit > 0.0f)
                               ? cmd->RateLimit : 0.01f,
            .reserved        = 0,
        };
        hal_status = LRMH_HAL_CommandPosition(&pos_cmd);
        g_CtrlApp.LastCommandedPos_m = cmd->TargetPosition_m;
        break;
    }

    case LRMH_CTRL_MODE_FORCE: {
        /* RateLimit is N/s in FORCE mode (m/s in POSITION mode) per
         * the LRMH_CtrlCmd_t contract in lrmh_ctrl_msg.h. */
        /* The stroke clamp is the inner defence and the LC watchpoint
         * the outer one (ADR-0012), so the clamp sits inside the
         * watchpoint: a force command on soft ground that runs out of
         * stroke ends at the clamp as a normal completion, not as a
         * HARD_STOP trip at LRMH_LC_STROKE_THRESHOLD_M. */
        LRMH_HAL_ForceCmd_t force_cmd = {
            .target_force_n = cmd->TargetForce_N,
            .max_stroke_m   = LRMH_LC_STROKE_THRESHOLD_M - 0.01f,
            .ramp_rate_nps  = cmd->RateLimit,
            .reserved       = 0,
        };
        hal_status = LRMH_HAL_CommandForce(&force_cmd);
        g_CtrlApp.LastCommandedForce_N = cmd->TargetForce_N;
        break;
    }

    case LRMH_CTRL_MODE_RETRACT:
        hal_status = LRMH_HAL_Retract();
        g_CtrlApp.LastCommandedPos_m = 0.0f;
        break;

    case LRMH_CTRL_MODE_HOLD:
        hal_status = LRMH_HAL_Hold();
        break;

    case LRMH_CTRL_MODE_ESTOP:
        hal_status = LRMH_HAL_EStop();
        break;

    case LRMH_CTRL_MODE_PRECHECK:
        /* Precheck is the actuator self-test per ADR-0010.
         * LRMH_HAL_Init runs the self-test and returns OK on
         * success or LRMH_HAL_SELFTEST_FAIL with a specific
         * error code. */
        hal_status = LRMH_HAL_Init();
        break;

    default:
        g_CtrlApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_CTRL_CMD_BAD_MODE_EID,
                          CFE_EVS_EventType_ERROR,
                          "LRMH_CTRL: unknown mode %u",
                          (unsigned)cmd->Mode);
        return;
    }

    if (hal_status != LRMH_HAL_OK) {
        g_CtrlApp.HalErrCount++;
        g_CtrlApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_CTRL_CMD_HAL_ERR_EID,
                          CFE_EVS_EventType_ERROR,
                          "LRMH_CTRL: HAL error 0x%08lX on mode %u",
                          (unsigned long)hal_status,
                          (unsigned)cmd->Mode);
        return;
    }

    g_CtrlApp.ActiveMode      = cmd->Mode;
    g_CtrlApp.CommandPending  = 1;
    g_CtrlApp.CmdCount++;

    CFE_EVS_SendEvent(LRMH_CTRL_CMD_OK_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_CTRL: mode=%u accepted (Pos=%.3f Force=%.1f)",
                      (unsigned)cmd->Mode,
                      (double)cmd->TargetPosition_m,
                      (double)cmd->TargetForce_N);
}

/* ============================================================
 * Message router
 * ============================================================ */
static void LRMH_CTRL_ProcessMsg(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_SB_MsgId_t MsgId;
    CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId)) {

    case LRMH_SEQ_TO_CTRL_MID:
        /* Sequencer command — token-validated */
        LRMH_CTRL_ValidateAndDispatch((const LRMH_CtrlCmd_t *)BufPtr);
        break;

    case LRMH_CTRL_CMD_MID: {
        /* Ground command surface: NOOP and RESET_COUNTERS only, by
         * function code. Ground actuator commands route through
         * lrmh_seq (LRMH_SEQ_MANUAL_CTRL_CC, ADR-0018) so the IDLE-only
         * gate applies; a raw packet here must never move the tool. */
        CFE_MSG_FcnCode_t fc = 0;
        CFE_MSG_GetFcnCode(&BufPtr->Msg, &fc);
        switch (fc) {
        case LRMH_CTRL_NOOP_CC:
            g_CtrlApp.CmdCount++;
            CFE_EVS_SendEvent(LRMH_CTRL_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "LRMH_CTRL: NOOP Cmd=%u Err=%u",
                              (unsigned)g_CtrlApp.CmdCount, (unsigned)g_CtrlApp.ErrCount);
            break;
        case LRMH_CTRL_RESET_COUNTERS_CC:
            g_CtrlApp.CmdCount = 0;
            g_CtrlApp.ErrCount = 0;
            g_CtrlApp.HalErrCount = 0;
            CFE_EVS_SendEvent(LRMH_CTRL_RESET_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "LRMH_CTRL: counters reset");
            break;
        default:
            g_CtrlApp.ErrCount++;
            CFE_EVS_SendEvent(LRMH_CTRL_CMD_BAD_MODE_EID, CFE_EVS_EventType_ERROR,
                              "LRMH_CTRL: ground FcnCode %u rejected; actuator "
                              "commands go through lrmh_seq manual control",
                              (unsigned)fc);
            break;
        }
        break;
    }

    case LRMH_FAULT_TO_CTRL_MID:
        /* Fault orchestrator — bypasses token validation */
        LRMH_CTRL_HandleCommand((const LRMH_CtrlCmd_t *)BufPtr, false);
        break;

    case LRMH_SEQ_SAFE_MID:
        /* SAFE broadcast: retract. lrmh_seq's T3 path also sends an
         * explicit RETRACT command; the two are idempotent. */
        LRMH_HAL_Retract();
        break;

    case LRMH_TOKEN_RESET_MID:
        LRMH_CTRL_OnTokenReset();
        break;

    default:
        g_CtrlApp.ErrCount++;
        break;
    }
}

/* ============================================================
 * Status TLM publication
 *
 * Polls HAL health for command completion and current state, then
 * publishes LRMH_CtrlStatusTlm_t. lrmh_seq subscribes to this and
 * uses CommandComplete to clear its WaitingForCtrl flag.
 * ============================================================ */
void LRMH_CTRL_PublishStatus(void)
{
    LRMH_HAL_Health_t health;
    int32 health_status = LRMH_HAL_GetHealth(&health);

    bool just_completed = false;
    if (g_CtrlApp.CommandPending && health_status == LRMH_HAL_OK) {
        /* IDLE means a position move completed; HOLDING means a
         * force command completed and is being held. Both are
         * "command done" from the FSW's perspective. */
        if (health.state == LRMH_HAL_STATE_IDLE ||
            health.state == LRMH_HAL_STATE_HOLDING) {
            g_CtrlApp.CommandPending = 0;
            just_completed = true;
        }
    }

    LRMH_CtrlStatusTlm_t tlm;
    memset(&tlm, 0, sizeof(tlm));
    CFE_MSG_Init(CFE_MSG_PTR(tlm.TlmHeader),
                 CFE_SB_ValueToMsgId(LRMH_CTRL_STATUS_MID), sizeof(tlm));

    tlm.ActiveMode          = g_CtrlApp.ActiveMode;
    tlm.CommandPending      = g_CtrlApp.CommandPending;
    tlm.CommandComplete     = just_completed ? 1 : 0;
    tlm.HalState            = (health_status == LRMH_HAL_OK)
                              ? health.state : (uint8)LRMH_HAL_STATE_FAULT;
    tlm.HalFlags            = (health_status == LRMH_HAL_OK)
                              ? health.flags : 0;
    tlm.StaleCount          = (uint16)g_CtrlApp.StaleCount;
    tlm.CmdCount            = g_CtrlApp.CmdCount;
    tlm.ErrCount            = g_CtrlApp.ErrCount;
    tlm.AcceptedEpoch       = g_CtrlApp.AcceptedToken.Epoch;
    tlm.AcceptedSeqState    = g_CtrlApp.AcceptedToken.SeqState;
    tlm.AcceptedSeqCount    = g_CtrlApp.AcceptedToken.SeqCount;
    tlm.LastCommandedPos_m  = g_CtrlApp.LastCommandedPos_m;
    tlm.LastCommandedForce_N = g_CtrlApp.LastCommandedForce_N;
    tlm.CurrentStroke_m     = (health_status == LRMH_HAL_OK)
                              ? health.stroke_m : 0.0f;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(tlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(tlm.TlmHeader), true);

    if (just_completed) {
        CFE_EVS_SendEvent(LRMH_CTRL_CMD_COMPLETE_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "LRMH_CTRL: command complete (mode=%u stroke=%.4f)",
                          (unsigned)g_CtrlApp.ActiveMode,
                          (double)tlm.CurrentStroke_m);
    }
}
