/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_hk_app.c
 * @brief LRMH lrmh_hk application — implementation
 *
 * Aggregates per-app status TLMs into the instrument-level HK
 * packet. Caches the latest values from each source on receipt
 * and publishes the aggregated packet on the 1 Hz wakeup tick
 * and on demand via LRMH_SEND_HK_MID.
 *
 * No HAL calls: lrmh_hk reads the controller and sense apps'
 * status TLMs instead.
 */

#include "lrmh_hk_app.h"

#include <string.h>

LRMH_HK_AppData_t g_HkApp;

/* ============================================================
 * Forward declarations
 * ============================================================ */
static int32 LRMH_HK_Init(void);
static void  LRMH_HK_ProcessMsg(const CFE_SB_Buffer_t *BufPtr);

/* ============================================================
 * Cache update handlers — called on every TLM arrival
 * ============================================================ */
void LRMH_HK_OnSeqStatus(const LRMH_SeqStatusTlm_t *seq)
{
    g_HkApp.SeqCache.valid             = true;
    g_HkApp.SeqCache.phase             = seq->Phase;
    g_HkApp.SeqCache.quality           = seq->QualityFlag;
    g_HkApp.SeqCache.waiting_for_ctrl  = seq->WaitingForCtrl;
    g_HkApp.SeqCache.t1_attempts       = (uint8)seq->T1AttemptsCurrentPhase;
    g_HkApp.SeqCache.site_id           = seq->SiteId;
    g_HkApp.SeqCache.session_id        = seq->SessionId;
    g_HkApp.SeqCache.epoch             = seq->Epoch;
    g_HkApp.SeqCache.cmd_count         = seq->CmdCount;
    g_HkApp.SeqCache.err_count         = seq->ErrCount;
}

void LRMH_HK_OnCtrlStatus(const LRMH_CtrlStatusTlm_t *ctrl)
{
    g_HkApp.CtrlCache.valid           = true;
    g_HkApp.CtrlCache.active_mode     = ctrl->ActiveMode;
    g_HkApp.CtrlCache.command_pending = ctrl->CommandPending;
    g_HkApp.CtrlCache.hal_state       = ctrl->HalState;
    g_HkApp.CtrlCache.hal_flags       = ctrl->HalFlags;
    g_HkApp.CtrlCache.stale_count     = ctrl->StaleCount;
    g_HkApp.CtrlCache.cmd_count       = ctrl->CmdCount;
    g_HkApp.CtrlCache.err_count       = ctrl->ErrCount;
    g_HkApp.CtrlCache.stroke_m        = ctrl->CurrentStroke_m;
    g_HkApp.CtrlCache.last_force_n    = ctrl->LastCommandedForce_N;
}

void LRMH_HK_OnSenseSample(const LRMH_SampleTlm_t *sense)
{
    g_HkApp.SenseCache.valid          = true;
    g_HkApp.SenseCache.channel_valid  = sense->channel_valid;
    g_HkApp.SenseCache.force_n        = sense->force_n;
    g_HkApp.SenseCache.depth_m        = sense->depth_m;
    g_HkApp.SenseCache.current_a      = sense->current_a;
    g_HkApp.SenseCache.bus_v          = sense->bus_v;
    g_HkApp.SenseCache.temp_c         = sense->temp_c;
    g_HkApp.SenseCache.force_rate_nps = sense->force_rate_nps;
}

void LRMH_HK_OnCmdStatus(const LRMH_CmdTlm_t *cmd)
{
    g_HkApp.CmdCache.valid               = true;
    g_HkApp.CmdCache.mode                = cmd->Mode;
    g_HkApp.CmdCache.cmd_count           = cmd->CmdCount;
    g_HkApp.CmdCache.err_count           = cmd->ErrCount;
    g_HkApp.CmdCache.rejected_count      = cmd->RejectedCount;
    g_HkApp.CmdCache.dispatch_cache_hits = cmd->DispatchCacheHits;
    g_HkApp.InstrumentMode               = cmd->Mode;
    g_HkApp.TotalRejectedCount           = cmd->RejectedCount;
}

void LRMH_HK_OnFaultStatus(const LRMH_FaultTlm_t *fault)
{
    g_HkApp.FaultFlags    = fault->FaultFlags;
    g_HkApp.FaultActive   = fault->FaultActive;
    g_HkApp.RecoveryTier  = fault->RecoveryTier;
    g_HkApp.TimeoutCount  = fault->TimeoutCount;
}

/* ============================================================
 * Publish — assemble HK packet from caches and transmit
 * ============================================================ */
void LRMH_HK_PublishHk(void)
{
    LRMH_HkTlm_t *p = &g_HkApp.HkPacket;
    memset(p, 0, sizeof(*p));
    CFE_MSG_Init(CFE_MSG_PTR(p->TlmHeader),
                 CFE_SB_ValueToMsgId(LRMH_HK_TLM_MID), sizeof(*p));

    /* ── Mode + tool (read from lrmh_cmd status TLM) ─────── */
    p->InstrumentMode = g_HkApp.InstrumentMode;
    p->ActiveTool     = g_HkApp.ActiveTool;

    /* ── Sequencer state ────────────────────────────────── */
    if (g_HkApp.SeqCache.valid) {
        p->SeqPhase                = g_HkApp.SeqCache.phase;
        p->SeqQuality              = g_HkApp.SeqCache.quality;
        p->SeqWaitingForCtrl       = g_HkApp.SeqCache.waiting_for_ctrl;
        p->T1AttemptsCurrentPhase  = g_HkApp.SeqCache.t1_attempts;
        p->CurrentSiteId           = g_HkApp.SeqCache.site_id;
        p->CurrentSessionId        = g_HkApp.SeqCache.session_id;
        p->Epoch                   = g_HkApp.SeqCache.epoch;
    }

    /* ── Controller state ───────────────────────────────── */
    if (g_HkApp.CtrlCache.valid) {
        p->CtrlActiveMode      = g_HkApp.CtrlCache.active_mode;
        p->CtrlCommandPending  = g_HkApp.CtrlCache.command_pending;
        p->HalState            = g_HkApp.CtrlCache.hal_state;
        p->HalFlags            = g_HkApp.CtrlCache.hal_flags;
        p->CtrlStaleCount      = g_HkApp.CtrlCache.stale_count;
        p->StrokePosition_m    = g_HkApp.CtrlCache.stroke_m;
        p->LastCommandedForce_N = g_HkApp.CtrlCache.last_force_n;
    }

    /* ── Sensor state ───────────────────────────────────── */
    if (g_HkApp.SenseCache.valid) {
        p->SensorChannelValid = g_HkApp.SenseCache.channel_valid;
        p->Force_N            = g_HkApp.SenseCache.force_n;
        p->Depth_m            = g_HkApp.SenseCache.depth_m;
        p->MotorCurrent_A     = g_HkApp.SenseCache.current_a;
        p->BusVoltage_V       = g_HkApp.SenseCache.bus_v;
        p->ActuatorTemp_C     = g_HkApp.SenseCache.temp_c;
        p->ForceRate_NpS      = g_HkApp.SenseCache.force_rate_nps;
    }

    /* ── Fault state ────────────────────────────────────── */
    p->FaultFlags     = g_HkApp.FaultFlags;
    p->FaultActive    = g_HkApp.FaultActive;
    p->RecoveryTier   = g_HkApp.RecoveryTier;
    p->TimeoutCount   = g_HkApp.TimeoutCount;

    /* ── Aggregated counters ────────────────────────────── */
    p->TotalCmdCount  = g_HkApp.SeqCache.cmd_count +
                        g_HkApp.CtrlCache.cmd_count;
    p->TotalErrCount  = g_HkApp.SeqCache.err_count +
                        g_HkApp.CtrlCache.err_count;
    p->TotalRejectedCount = g_HkApp.TotalRejectedCount;

    g_HkApp.HkCycleCount++;
    p->HkCycleCount = g_HkApp.HkCycleCount;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(p->TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(p->TlmHeader), true);
}

/* ============================================================
 * App main
 * ============================================================ */
void LRMH_HK_AppMain(void)
{
    int32 Status;
    CFE_SB_Buffer_t *BufPtr = NULL;
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = LRMH_HK_Init();
    if (Status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog("LRMH_HK: Init failed\n");
        CFE_ES_ExitApp(CFE_ES_RunStatus_APP_ERROR);
        return;
    }

    CFE_EVS_SendEvent(LRMH_HK_INIT_INF_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_HK: initialized");

    while (CFE_ES_RunLoop(&RunStatus) == true) {
        /* Block until the next message arrives. Each subscribed status
         * TLM (LRMH_SEQ_STATUS_MID, LRMH_CTRL_STATUS_MID,
         * LRMH_SENSE_TLM_MID, LRMH_FAULT_TLM_MID, LRMH_CMD_STATUS_MID)
         * just updates the cached state for the next HK publication;
         * only LRMH_SEND_HK_MID actually triggers PublishHk.
         *
         * The pipe-timeout fallback is a heartbeat in case SCH_LAB
         * stops feeding LRMH_SEND_HK_MID — at which point we publish
         * once anyway so the dashboard's connection-staleness
         * indicator doesn't flatline. */
        Status = CFE_SB_ReceiveBuffer(&BufPtr, g_HkApp.CmdPipe, 1000);
        if (Status == CFE_SUCCESS && BufPtr != NULL) {
            LRMH_HK_ProcessMsg(BufPtr);
        } else if (Status == CFE_SB_TIME_OUT) {
            LRMH_HK_PublishHk();
        }
        else if (Status != CFE_SB_TIME_OUT) {
            /* A software-bus receive error is not something the app can
             * recover from in place: event it and let ES restart the app
             * (HS policy) rather than spin on a broken pipe. */
            CFE_EVS_SendEvent(LRMH_HK_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "LRMH_HK: SB receive error 0x%08lX, exiting",
                              (unsigned long)Status);
            RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    CFE_ES_ExitApp(RunStatus);
}

/* ============================================================
 * Initialization
 * ============================================================ */
static int32 LRMH_HK_Init(void)
{
    int32 Status;
    memset(&g_HkApp, 0, sizeof(g_HkApp));

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) return Status;

    Status = CFE_SB_CreatePipe(&g_HkApp.CmdPipe,
                               LRMH_HK_PIPE_DEPTH, "LRMH_HK_PIPE");
    if (Status != CFE_SUCCESS) return Status;

    /* Subscribe to per-app status TLMs */
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEND_HK_MID),
                     g_HkApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEQ_STATUS_MID),
                     g_HkApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_CTRL_STATUS_MID),
                     g_HkApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SENSE_TLM_MID),
                     g_HkApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_FAULT_TLM_MID),
                     g_HkApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_CMD_STATUS_MID),
                     g_HkApp.CmdPipe);

    return CFE_SUCCESS;
}

/* ============================================================
 * Message router
 * ============================================================ */
static void LRMH_HK_ProcessMsg(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_SB_MsgId_t MsgId;
    CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId)) {
    case LRMH_SEND_HK_MID:
        LRMH_HK_PublishHk();
        break;
    case LRMH_SEQ_STATUS_MID:
        LRMH_HK_OnSeqStatus((const LRMH_SeqStatusTlm_t *)BufPtr);
        break;
    case LRMH_CTRL_STATUS_MID:
        LRMH_HK_OnCtrlStatus((const LRMH_CtrlStatusTlm_t *)BufPtr);
        break;
    case LRMH_SENSE_TLM_MID:
        LRMH_HK_OnSenseSample((const LRMH_SampleTlm_t *)BufPtr);
        break;
    case LRMH_CMD_STATUS_MID:
        LRMH_HK_OnCmdStatus((const LRMH_CmdTlm_t *)BufPtr);
        break;
    case LRMH_FAULT_TLM_MID:
        LRMH_HK_OnFaultStatus((const LRMH_FaultTlm_t *)BufPtr);
        break;
    default:
        break;
    }
}
