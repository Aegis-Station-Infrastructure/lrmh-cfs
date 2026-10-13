/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_cmd_app.c
 * @brief LRMH lrmh_cmd application — implementation
 *
 * Front door for the instrument. Owns the mode FSM (ADR-0004),
 * the dispatch cache (ADR-0014), and the token epoch bumps for
 * INIT/SAFE/restart paths (ADR-0007).
 *
 * Per ADR-0006, lrmh_cmd is deliberately separate from lrmh_seq
 * so a sequencer crash never takes down the command surface.
 * Per the HS table in ADR-0017, lrmh_cmd has NO restart action —
 * losing the command surface requires ground intervention.
 */

#include "lrmh_cmd_app.h"

#include <string.h>
#include <stddef.h>

/* Host-vehicle pose TLM, the subset LRMH reads (ADR-0011). The same
 * view is declared in lrmh_sense; both must track the host's layout. */
typedef struct {
    CFE_MSG_TelemetryHeader_t  TelemetryHeader;
    float    X;
    float    Y;
    float    Z;
    float    Roll;
    float    Pitch;
    float    Yaw;
    uint32_t CmdCount;
    uint32_t ErrCount;
    uint32_t HalTimeoutCount;
    uint8_t  NavState;
    uint8_t  ActiveWpId;
    uint16_t Padding16;
    /* remaining rover-internal fields ignored */
} LRMH_HostPoseTlm_t;

#define AEGIS_NAV_STATE_IDLE  0

/* ============================================================
 * Global app data (visible to standalone test)
 * ============================================================ */
LRMH_CMD_AppData_t g_CmdApp;

/* ============================================================
 * Forward declarations (file-static functions)
 * ============================================================ */
static int32 LRMH_CMD_Init(void);
static void  LRMH_CMD_ProcessPacket(const CFE_SB_Buffer_t *BufPtr);
static void  LRMH_CMD_HandleGroundCommand(const CFE_SB_Buffer_t *BufPtr);
static void  LRMH_CMD_HandleHostPose(const CFE_SB_Buffer_t *BufPtr);
static void  LRMH_CMD_HandleSeqNotify(const CFE_SB_Buffer_t *BufPtr);

static void  LRMH_CMD_Noop(void);
static void  LRMH_CMD_ResetCounters(void);
static void  LRMH_CMD_InitInstrument(void);
static void  LRMH_CMD_Deploy(void);
static void  LRMH_CMD_Stow(void);
static void  LRMH_CMD_Abort(void);
static void  LRMH_CMD_Safe(void);
static void  LRMH_CMD_ResetFault(void);
static void  LRMH_CMD_PowerOff(void);
static void  LRMH_CMD_StartSiteTest(const LRMH_StartSiteTestCmd_t *cmd);
static void  LRMH_CMD_SetMode(const LRMH_SetModeCmd_t *cmd);

static bool  LRMH_CMD_AdvanceMode(LRMH_ModeTrigger_t trigger);
static bool  LRMH_CMD_HostNavOk(uint8_t dispatch_flags);
static uint64_t LRMH_CMD_NowUs(void);

/* ============================================================
 * Mode FSM transition matrix (ADR-0004)
 *
 * (from, trigger) → to. LRMH_MODE_COUNT means "illegal".
 *
 * This matrix is the AUTHORITATIVE source for which transitions
 * are allowed. The transition handler in LRMH_CMD_AdvanceMode()
 * is the only place that mutates g_CmdApp.Mode.
 * ============================================================ */
static const uint8 mode_matrix[LRMH_MODE_COUNT][LRMH_TRIGGER_COUNT] = {
    /*               POWER  DEPLOY READY  STOW   START  COMP   T12    T3     ABORT  INIT   RST_F  EMER   PWROFF */
    /* OFF     */  { LRMH_MODE_STANDBY, LRMH_MODE_STANDBY, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_OFF, LRMH_MODE_COUNT },
    /* STANDBY */  { LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_READY, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_SAFE, LRMH_MODE_SAFE, LRMH_MODE_SAFE, LRMH_MODE_STANDBY, LRMH_MODE_COUNT, LRMH_MODE_OFF, LRMH_MODE_OFF },
    /* READY   */  { LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_STANDBY, LRMH_MODE_TESTING, LRMH_MODE_COUNT, LRMH_MODE_SAFE, LRMH_MODE_SAFE, LRMH_MODE_SAFE, LRMH_MODE_STANDBY, LRMH_MODE_COUNT, LRMH_MODE_OFF, LRMH_MODE_COUNT },
    /* TESTING */  { LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_STANDBY, LRMH_MODE_TESTING, LRMH_MODE_SAFE, LRMH_MODE_SAFE, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_OFF, LRMH_MODE_COUNT },
    /* SAFE    */  { LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_FAULT, LRMH_MODE_COUNT, LRMH_MODE_STANDBY, LRMH_MODE_COUNT, LRMH_MODE_OFF, LRMH_MODE_COUNT },
    /* FAULT   */  { LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_COUNT, LRMH_MODE_SAFE, LRMH_MODE_OFF, LRMH_MODE_COUNT }
};

/*
 * Note on TESTING + FAULT_T12: a T1/T2 fault during TESTING does
 * NOT transition the mode FSM — recovery is handled by lrmh_seq's
 * phase machine, the mode stays TESTING until either the test
 * completes (TEST_COMPLETE → STANDBY) or escalates to T3 (→ SAFE).
 * The matrix encodes this as TESTING + FAULT_T12 → TESTING (no-op).
 */

bool LRMH_CMD_TransitionAllowed(LRMH_InstrumentMode_t from,
                                LRMH_ModeTrigger_t trigger,
                                LRMH_InstrumentMode_t *to_out)
{
    if (from >= LRMH_MODE_COUNT || trigger >= LRMH_TRIGGER_COUNT) return false;
    uint8 to = mode_matrix[from][trigger];
    if (to >= LRMH_MODE_COUNT) return false;
    if (to_out) *to_out = (LRMH_InstrumentMode_t)to;
    return true;
}

static bool LRMH_CMD_AdvanceMode(LRMH_ModeTrigger_t trigger)
{
    LRMH_InstrumentMode_t new_mode;
    if (!LRMH_CMD_TransitionAllowed(g_CmdApp.Mode, trigger, &new_mode)) {
        g_CmdApp.RejectedCount++;
        CFE_EVS_SendEvent(LRMH_CMD_MODE_REJECT_EID, CFE_EVS_EventType_ERROR,
                          "LRMH_CMD: illegal transition Mode=%u Trigger=%u",
                          (unsigned)g_CmdApp.Mode, (unsigned)trigger);
        return false;
    }

    LRMH_InstrumentMode_t old_mode = g_CmdApp.Mode;

    /* SAFE entry side effect: bump token epoch + broadcast (ADR-0007) */
    if (new_mode == LRMH_MODE_SAFE && old_mode != LRMH_MODE_SAFE) {
        LRMH_CMD_BroadcastTokenReset(LRMH_TOKEN_RESET_SAFE_ENTRY);
        CFE_EVS_SendEvent(LRMH_CMD_SAFE_ENTRY_EID,
                          CFE_EVS_EventType_CRITICAL,
                          "LRMH_CMD: SAFE entry, token epoch bumped");
    }

    /* SAFE → FAULT terminal */
    if (new_mode == LRMH_MODE_FAULT) {
        CFE_EVS_SendEvent(LRMH_CMD_FAULT_ESCALATE_EID,
                          CFE_EVS_EventType_CRITICAL,
                          "LRMH_CMD: SAFE → FAULT terminal");
    }

    g_CmdApp.Mode = new_mode;
    CFE_EVS_SendEvent(LRMH_CMD_MODE_OK_EID, CFE_EVS_EventType_INFORMATION,
                      "LRMH_CMD: Mode %u → %u (trigger %u)",
                      (unsigned)old_mode, (unsigned)new_mode,
                      (unsigned)trigger);
    /* Publish status TLM so lrmh_hk picks up the new mode */
    LRMH_CMD_PublishStatus();
    return true;
}

/* ============================================================
 * Token epoch broadcast (ADR-0007)
 * ============================================================ */
void LRMH_CMD_BroadcastTokenReset(LRMH_TokenResetReason_t reason)
{
    LRMH_SeqToken_BumpEpoch(reason);

    LRMH_TokenResetTlm_t reset;
    CFE_MSG_Init(CFE_MSG_PTR(reset.TelemetryHeader),
                 CFE_SB_ValueToMsgId(LRMH_TOKEN_RESET_MID),
                 sizeof(reset));
    reset.NewEpoch        = LRMH_SeqToken_GetCurrentEpoch();
    reset.ResetReason     = (uint8)reason;
    memset(reset.Reserved, 0, sizeof(reset.Reserved));
    reset.PhaseStartTime_us = LRMH_CMD_NowUs();
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(reset.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(reset.TelemetryHeader), true);
}

/* ============================================================
 * Dispatch cache (ADR-0014)
 *
 * 8-entry FIFO. Lookup is linear (8 entries, trivially fast).
 * Insertion overwrites the oldest entry.
 * ============================================================ */
LRMH_DispatchOutcome_t LRMH_CMD_DispatchCacheLookup(uint16 session, uint16 seq)
{
    for (int i = 0; i < LRMH_DISPATCH_CACHE_DEPTH; i++) {
        const LRMH_DispatchCacheEntry_t *e = &g_CmdApp.DispatchCache[i];
        if (e->Outcome != LRMH_DISPATCH_OUTCOME_NONE &&
            e->SessionId == session && e->SequenceNumber == seq) {
            return (LRMH_DispatchOutcome_t)e->Outcome;
        }
    }
    return LRMH_DISPATCH_OUTCOME_NONE;
}

void LRMH_CMD_DispatchCacheRecord(uint16 session, uint16 seq,
                                  LRMH_DispatchOutcome_t outcome)
{
    /* Update existing entry if present */
    for (int i = 0; i < LRMH_DISPATCH_CACHE_DEPTH; i++) {
        LRMH_DispatchCacheEntry_t *e = &g_CmdApp.DispatchCache[i];
        if (e->Outcome != LRMH_DISPATCH_OUTCOME_NONE &&
            e->SessionId == session && e->SequenceNumber == seq) {
            e->Outcome = (uint8)outcome;
            return;
        }
    }
    /* Insert at FIFO head, overwriting oldest */
    LRMH_DispatchCacheEntry_t *slot =
        &g_CmdApp.DispatchCache[g_CmdApp.DispatchCacheHead];
    slot->SessionId      = session;
    slot->SequenceNumber = seq;
    slot->Outcome        = (uint8)outcome;
    g_CmdApp.DispatchCacheHead =
        (g_CmdApp.DispatchCacheHead + 1) % LRMH_DISPATCH_CACHE_DEPTH;
}

/* ============================================================
 * Status TLM publication
 *
 * Publishes LRMH_CmdTlm_t on LRMH_CMD_STATUS_MID so lrmh_hk
 * can read instrument mode, command counters, and dispatch
 * cache hit count.
 * ============================================================ */
void LRMH_CMD_PublishStatus(void)
{
    LRMH_CmdTlm_t tlm;
    memset(&tlm, 0, sizeof(tlm));
    CFE_MSG_Init(CFE_MSG_PTR(tlm.TelemetryHeader),
                 CFE_SB_ValueToMsgId(LRMH_CMD_STATUS_MID), sizeof(tlm));
    tlm.Mode                  = (uint8)g_CmdApp.Mode;
    tlm.CmdCount              = g_CmdApp.CmdCount;
    tlm.ErrCount              = g_CmdApp.ErrCount;
    tlm.RejectedCount         = g_CmdApp.RejectedCount;
    tlm.DispatchCacheHits     = g_CmdApp.DispatchCacheHits;
    tlm.CurrentSessionId      = g_CmdApp.CurrentSessionId;
    tlm.CurrentSequenceNumber = g_CmdApp.CurrentSequenceNumber;
    tlm.CurrentSiteId         = g_CmdApp.CurrentSiteId;
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(tlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(tlm.TelemetryHeader), true);
}

/* ============================================================
 * Host nav precondition check (ADR-0014 §"Two preconditions")
 * ============================================================ */
static bool LRMH_CMD_HostNavOk(uint8_t dispatch_flags)
{
    if (dispatch_flags & LRMH_DISPATCH_WITNESS_TARGET) return true;
    if (dispatch_flags & LRMH_DISPATCH_OVERRIDE_HOST)  return true;

    if (!g_CmdApp.HostPose.valid) return false;

    uint64_t now = LRMH_CMD_NowUs();
    uint64_t age_ms = (now - g_CmdApp.HostPose.receipt_time_us) / 1000ULL;
    if (age_ms > LRMH_HOST_NAV_MAX_AGE_MS) return false;

    if (dispatch_flags & LRMH_DISPATCH_STRICT_NAV) {
        if (g_CmdApp.HostPose.nav_state != AEGIS_NAV_STATE_IDLE) return false;
    }
    return true;
}

/* ============================================================
 * Time helper — wraps the cFE time API to give microseconds
 * ============================================================ */
static uint64_t LRMH_CMD_NowUs(void)
{
    CFE_TIME_SysTime_t t = CFE_TIME_GetTime();
    /* CFE_TIME_SysTime_t is seconds + subseconds (32-bit fraction).
     * Convert to microseconds. */
    uint64_t us = (uint64_t)t.Seconds * 1000000ULL;
    us += ((uint64_t)t.Subseconds * 1000000ULL) >> 32;
    return us;
}

/* ============================================================
 * App main
 * ============================================================ */
void LRMH_CMD_AppMain(void)
{
    int32 Status;
    CFE_SB_Buffer_t *BufPtr = NULL;
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = LRMH_CMD_Init();
    if (Status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog("LRMH_CMD: Init failed (0x%08lX)\n",
                             (unsigned long)Status);
        return;
    }

    CFE_EVS_SendEvent(LRMH_CMD_INIT_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "LRMH_CMD: initialized, Mode=STANDBY");

    while (CFE_ES_RunLoop(&RunStatus) == true) {
        Status = CFE_SB_ReceiveBuffer(&BufPtr, g_CmdApp.CmdPipe,
                                      CFE_SB_PEND_FOREVER);
        if (Status == CFE_SUCCESS && BufPtr != NULL) {
            LRMH_CMD_ProcessPacket(BufPtr);
        }
        else if (Status != CFE_SB_TIME_OUT) {
            /* A software-bus receive error is not something the app can
             * recover from in place: event it and let ES restart the app
             * (HS policy) rather than spin on a broken pipe. */
            g_CmdApp.ErrCount++;
            CFE_EVS_SendEvent(LRMH_CMD_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "LRMH_CMD: SB receive error 0x%08lX, exiting",
                              (unsigned long)Status);
            RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    CFE_ES_ExitApp(RunStatus);
}

/* ============================================================
 * Initialization
 * ============================================================ */
static int32 LRMH_CMD_Init(void)
{
    int32 Status;

    memset(&g_CmdApp, 0, sizeof(g_CmdApp));
    g_CmdApp.Mode = LRMH_MODE_STANDBY;

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) return Status;

    Status = CFE_SB_CreatePipe(&g_CmdApp.CmdPipe,
                               LRMH_CMD_PIPE_DEPTH, "LRMH_CMD_PIPE");
    if (Status != CFE_SUCCESS) return Status;

    Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_CMD_MID),
                              g_CmdApp.CmdPipe);
    if (Status != CFE_SUCCESS) return Status;

    Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEND_HK_MID),
                              g_CmdApp.CmdPipe);
    if (Status != CFE_SUCCESS) return Status;

    /* Subscribe to host pose for the dispatch precondition check
     * (ADR-0011 / ADR-0014). lrmh_sense maintains its own richer
     * cache for sample alignment; this one is single-purpose. */
    Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_HOST_POSE_MID_VALUE),
                              g_CmdApp.CmdPipe);
    if (Status != CFE_SUCCESS) {
        /* Non-fatal — host pose subscription is for the dispatch
         * precondition; if the host vehicle isn't publishing,
         * lrmh_cmd accepts commands without the precondition. */
        CFE_EVS_SendEvent(LRMH_CMD_HOST_POSE_SUB_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "LRMH_CMD: host pose subscription unavailable");
    }

    /* Subscribe to seq → cmd FSM notifications. lrmh_seq publishes on
     * this MID at end-of-test (ADR-0004 TESTING → STANDBY). */
    Status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEQ_TO_CMD_MID),
                              g_CmdApp.CmdPipe);
    if (Status != CFE_SUCCESS) return Status;

    /* Cold-start token reset broadcast (ADR-0007) */
    LRMH_CMD_BroadcastTokenReset(LRMH_TOKEN_RESET_COLD_START);

    return CFE_SUCCESS;
}

/* ============================================================
 * Packet router
 * ============================================================ */
static void LRMH_CMD_ProcessPacket(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_SB_MsgId_t MsgId;
    CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId)) {
    case LRMH_CMD_MID:
        LRMH_CMD_HandleGroundCommand(BufPtr);
        break;
    case LRMH_SEND_HK_MID:
        /* lrmh_hk handles HK publication; this is a no-op forward */
        break;
    case LRMH_HOST_POSE_MID_VALUE:
        LRMH_CMD_HandleHostPose(BufPtr);
        break;
    case LRMH_SEQ_TO_CMD_MID:
        LRMH_CMD_HandleSeqNotify(BufPtr);
        break;
    default:
        g_CmdApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_CMD_MID_ERR_EID, CFE_EVS_EventType_ERROR,
                          "LRMH_CMD: Unknown MID 0x%04X",
                          (unsigned)CFE_SB_MsgIdToValue(MsgId));
        break;
    }
}

/* ============================================================
 * Host pose cache update (ADR-0011)
 *
 * The rover's pose TLM struct is owned by the rover stack; we
 * see only the prefix we care about (LRMH_HostPoseTlm_t). If
 * the rover ever shrinks the struct, NavState's offset shifts
 * and reading it would return garbage — so verify the message
 * size first and reject anything shorter than our view.
 * ============================================================ */
static void LRMH_CMD_HandleHostPose(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_Size_t msg_size = 0;
    CFE_MSG_GetSize(&BufPtr->Msg, &msg_size);
    if (msg_size < sizeof(LRMH_HostPoseTlm_t)) {
        g_CmdApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_CMD_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "LRMH_CMD: host pose TLM too short (%u < %u)",
                          (unsigned)msg_size,
                          (unsigned)sizeof(LRMH_HostPoseTlm_t));
        return;
    }
    const LRMH_HostPoseTlm_t *pose = (const LRMH_HostPoseTlm_t *)BufPtr;
    g_CmdApp.HostPose.valid           = true;
    g_CmdApp.HostPose.receipt_time_us = LRMH_CMD_NowUs();
    g_CmdApp.HostPose.nav_state       = pose->NavState;
}

/* ============================================================
 * Sequencer → cmd FSM notification (ADR-0004)
 *
 * lrmh_seq publishes on LRMH_SEQ_TO_CMD_MID for FSM-relevant events
 * the sequencer alone observes. Currently only TEST_COMPLETE drives
 * a transition (TESTING → STANDBY); future notify types can be
 * added without changing the wire shape.
 * ============================================================ */
static void LRMH_CMD_HandleSeqNotify(const CFE_SB_Buffer_t *BufPtr)
{
    const LRMH_SeqToCmdNotify_t *n = (const LRMH_SeqToCmdNotify_t *)BufPtr;
    switch (n->NotifyType) {
    case LRMH_SEQ_NOTIFY_TEST_COMPLETE:
        /* Only legal from TESTING. AdvanceMode emits the standard
         * Mode-OK / Mode-REJECT EVS event so we don't need a
         * dedicated event here. */
        (void)LRMH_CMD_AdvanceMode(LRMH_TRIGGER_TEST_COMPLETE);
        break;
    default:
        g_CmdApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_CMD_NOTIFY_ERR_EID, CFE_EVS_EventType_ERROR,
                          "LRMH_CMD: unknown SEQ notify type %u",
                          (unsigned)n->NotifyType);
        break;
    }
}

/* ============================================================
 * Ground command dispatch
 * ============================================================ */
static void LRMH_CMD_HandleGroundCommand(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_MSG_FcnCode_t FcnCode;
    CFE_MSG_GetFcnCode(&BufPtr->Msg, &FcnCode);

    switch (FcnCode) {
    case LRMH_NOOP_CC:           LRMH_CMD_Noop();           break;
    case LRMH_RESET_COUNTERS_CC: LRMH_CMD_ResetCounters();  break;
    case LRMH_INIT_INSTRUMENT_CC: LRMH_CMD_InitInstrument(); break;
    case LRMH_DEPLOY_CC:         LRMH_CMD_Deploy();         break;
    case LRMH_STOW_CC:           LRMH_CMD_Stow();           break;
    case LRMH_ABORT_CC:          LRMH_CMD_Abort();          break;
    case LRMH_SAFE_CC:           LRMH_CMD_Safe();           break;
    case LRMH_RESET_FAULT_CC:    LRMH_CMD_ResetFault();     break;
    case LRMH_POWER_OFF_CC:      LRMH_CMD_PowerOff();       break;
    case LRMH_START_SITE_TEST_CC:
        LRMH_CMD_StartSiteTest((const LRMH_StartSiteTestCmd_t *)BufPtr);
        break;
    case LRMH_SET_MODE_CC:
        LRMH_CMD_SetMode((const LRMH_SetModeCmd_t *)BufPtr);
        break;
    default:
        g_CmdApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_CMD_FCN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "LRMH_CMD: Unknown FcnCode %u",
                          (unsigned)FcnCode);
        break;
    }
}

/* ============================================================
 * Command implementations
 * ============================================================ */

static void LRMH_CMD_Noop(void)
{
    g_CmdApp.CmdCount++;
    CFE_EVS_SendEvent(LRMH_CMD_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "LRMH_CMD: NOOP Cmd=%u Err=%u Mode=%u",
                      (unsigned)g_CmdApp.CmdCount,
                      (unsigned)g_CmdApp.ErrCount,
                      (unsigned)g_CmdApp.Mode);
}

static void LRMH_CMD_ResetCounters(void)
{
    g_CmdApp.CmdCount = 0;
    g_CmdApp.ErrCount = 0;
    g_CmdApp.RejectedCount = 0;
    g_CmdApp.DispatchCacheHits = 0;
    CFE_EVS_SendEvent(LRMH_CMD_RESET_INF_EID, CFE_EVS_EventType_INFORMATION,
                      "LRMH_CMD: counters reset");
}

static void LRMH_CMD_InitInstrument(void)
{
    int32 Status = LRMH_HAL_Init();
    if (Status != LRMH_HAL_OK) {
        g_CmdApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_CMD_HAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "LRMH_CMD: HAL Init failed (0x%08lX)",
                          (unsigned long)Status);
        /* Failed init does NOT advance the FSM — leave the mode
         * where it was so ground can retry. */
        return;
    }

    /* Successful init drives any recoverable state to STANDBY so
     * INIT is the universal "bring the instrument up" command:
     *   OFF        → STANDBY via POWER_ON
     *   SAFE/FAULT → STANDBY via INIT_INSTRUMENT
     *   STANDBY    → no-op (epoch bump only)
     *   READY/TESTING → no-op (do not interrupt a live test) */
    if (g_CmdApp.Mode == LRMH_MODE_OFF) {
        if (!LRMH_CMD_AdvanceMode(LRMH_TRIGGER_POWER_ON)) return;
    } else if (g_CmdApp.Mode == LRMH_MODE_SAFE ||
               g_CmdApp.Mode == LRMH_MODE_FAULT) {
        if (!LRMH_CMD_AdvanceMode(LRMH_TRIGGER_INIT_INSTRUMENT)) return;
    }

    LRMH_CMD_BroadcastTokenReset(LRMH_TOKEN_RESET_INIT_INSTRUMENT);

    g_CmdApp.CmdCount++;
    CFE_EVS_SendEvent(LRMH_CMD_INIT_INSTRUMENT_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_CMD: instrument initialized, token epoch bumped");
}

static void LRMH_CMD_Deploy(void)
{
    /* The ground DEPLOY command is the operator's "get to READY"
     * intent. From STANDBY, that maps to the READY_FOR_TEST
     * trigger in the ADR-0004 mode FSM. The LRMH_TRIGGER_DEPLOY
     * enum value is the OFF → STANDBY power-on path and is the
     * wrong trigger here. From OFF the ground command is
     * POWER_ON, not DEPLOY — DEPLOY at the ground level only
     * makes sense after the instrument is powered. */
    LRMH_ModeTrigger_t trig = (g_CmdApp.Mode == LRMH_MODE_OFF)
                              ? LRMH_TRIGGER_DEPLOY
                              : LRMH_TRIGGER_READY_FOR_TEST;
    if (!LRMH_CMD_AdvanceMode(trig)) return;
    g_CmdApp.CmdCount++;
    CFE_EVS_SendEvent(LRMH_CMD_DEPLOY_EID, CFE_EVS_EventType_INFORMATION,
                      "LRMH_CMD: DEPLOY → Mode=%u", (unsigned)g_CmdApp.Mode);
}

static void LRMH_CMD_Stow(void)
{
    /* STOW is "retract tool to home" — valid from READY (→ STANDBY).
     * STANDBY → OFF is NOT a STOW; it's POWER_OFF (ADR-0004). Gate on
     * transition legality first so we don't retract HAL only to then
     * reject the mode change. */
    if (!LRMH_CMD_TransitionAllowed(g_CmdApp.Mode, LRMH_TRIGGER_STOW, NULL)) {
        g_CmdApp.RejectedCount++;
        if (g_CmdApp.Mode == LRMH_MODE_STANDBY) {
            CFE_EVS_SendEvent(LRMH_CMD_MODE_REJECT_EID,
                              CFE_EVS_EventType_ERROR,
                              "LRMH_CMD: STOW rejected from STANDBY "
                              "(tool already stowed; use POWER_OFF to "
                              "power down)");
        } else {
            CFE_EVS_SendEvent(LRMH_CMD_MODE_REJECT_EID,
                              CFE_EVS_EventType_ERROR,
                              "LRMH_CMD: STOW rejected, Mode=%u",
                              (unsigned)g_CmdApp.Mode);
        }
        return;
    }

    int32 Status = LRMH_HAL_Retract();
    if (Status != LRMH_HAL_OK) {
        g_CmdApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_CMD_HAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "LRMH_CMD: STOW HAL_Retract failed (0x%08lX)",
                          (unsigned long)Status);
        return;
    }

    if (!LRMH_CMD_AdvanceMode(LRMH_TRIGGER_STOW)) return;
    g_CmdApp.CmdCount++;
    CFE_EVS_SendEvent(LRMH_CMD_STOW_EID, CFE_EVS_EventType_INFORMATION,
                      "LRMH_CMD: STOW commanded");
}

static void LRMH_CMD_PowerOff(void)
{
    /* Explicit power-down from STANDBY (ADR-0004). Split from STOW
     * to remove the foot-gun where a STOW command from STANDBY
     * silently powered the instrument off. */
    if (!LRMH_CMD_AdvanceMode(LRMH_TRIGGER_POWER_OFF)) return;
    g_CmdApp.CmdCount++;
    CFE_EVS_SendEvent(LRMH_CMD_POWER_OFF_EID, CFE_EVS_EventType_INFORMATION,
                      "LRMH_CMD: POWER_OFF commanded");
}

static void LRMH_CMD_Abort(void)
{
    /* Unconditional T3 (ADR-0008): EStop the actuator and SAFE the
     * mode. The unconditional accept rule (ADR-0007 rule 1) means
     * this command bypasses token validation in receiving apps. */
    LRMH_HAL_EStop();
    LRMH_CMD_AdvanceMode(LRMH_TRIGGER_ABORT);

    /* Tell the sequencer so the phase machine leaves the test rather
     * than resuming it after the next INIT. LRMH_SEQ_ABORT_CC is on
     * the unconditional-accept list, so no token is needed. */
    LRMH_NoArgsCmd_t seq_abort;
    CFE_MSG_Init(CFE_MSG_PTR(seq_abort.CmdHeader),
                 CFE_SB_ValueToMsgId(LRMH_SEQ_CMD_MID), sizeof(seq_abort));
    /* lrmh_seq's ABORT function code mirrors LRMH_CMD_ABORT_CC by design
     * (see lrmh_seq_msg.h), so the shared value is used here. */
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(seq_abort.CmdHeader), LRMH_CMD_ABORT_CC);
    CFE_SB_TransmitMsg(CFE_MSG_PTR(seq_abort.CmdHeader), true);

    g_CmdApp.CmdCount++;
    CFE_EVS_SendEvent(LRMH_CMD_ABORT_EID, CFE_EVS_EventType_CRITICAL,
                      "LRMH_CMD: ABORT executed, sequencer notified");
}

static void LRMH_CMD_Safe(void)
{
    /* Same path as ABORT but without the EStop: used by lrmh_fault
     * dispatching a T3. The retract comes from lrmh_seq's T3 handler
     * (and lrmh_ctrl's SAFE-broadcast handler). */
    LRMH_CMD_AdvanceMode(LRMH_TRIGGER_FAULT_T3);
    g_CmdApp.CmdCount++;
}

static void LRMH_CMD_ResetFault(void)
{
    if (!LRMH_CMD_AdvanceMode(LRMH_TRIGGER_RESET_FAULT)) return;
    g_CmdApp.CmdCount++;
    CFE_EVS_SendEvent(LRMH_CMD_FAULT_RESET_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_CMD: FAULT mode reset by ground");
}

static void LRMH_CMD_StartSiteTest(const LRMH_StartSiteTestCmd_t *cmd)
{
    /* ADR-0014 idempotency check (precondition zero) */
    if (cmd->DispatchFlags & LRMH_DISPATCH_IDEMPOTENT) {
        LRMH_DispatchOutcome_t prior =
            LRMH_CMD_DispatchCacheLookup(cmd->SessionId, cmd->SequenceNumber);
        if (prior == LRMH_DISPATCH_OUTCOME_ACCEPTED ||
            prior == LRMH_DISPATCH_OUTCOME_IN_PROGRESS ||
            prior == LRMH_DISPATCH_OUTCOME_COMPLETE) {
            g_CmdApp.DispatchCacheHits++;
            CFE_EVS_SendEvent(LRMH_CMD_DISPATCH_RETRY_EID,
                              CFE_EVS_EventType_INFORMATION,
                              "LRMH_CMD: idempotent retry absorbed "
                              "(Session=%u Seq=%u)",
                              (unsigned)cmd->SessionId,
                              (unsigned)cmd->SequenceNumber);
            return;
        }
    }

    /* ADR-0014 precondition A: mode FSM is in READY */
    if (g_CmdApp.Mode != LRMH_MODE_READY) {
        g_CmdApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_CMD_DISPATCH_REJECT_EID,
                          CFE_EVS_EventType_ERROR,
                          "LRMH_CMD: START_SITE_TEST rejected, "
                          "Mode=%u (need READY)",
                          (unsigned)g_CmdApp.Mode);
        if (cmd->DispatchFlags & LRMH_DISPATCH_IDEMPOTENT) {
            LRMH_CMD_DispatchCacheRecord(cmd->SessionId, cmd->SequenceNumber,
                                         LRMH_DISPATCH_OUTCOME_REJECTED);
        }
        return;
    }

    /* ADR-0014 precondition B: host nav state */
    if (!LRMH_CMD_HostNavOk(cmd->DispatchFlags)) {
        g_CmdApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_CMD_DISPATCH_NAV_EID,
                          CFE_EVS_EventType_ERROR,
                          "LRMH_CMD: START_SITE_TEST rejected, "
                          "host nav stale or invalid");
        if (cmd->DispatchFlags & LRMH_DISPATCH_IDEMPOTENT) {
            LRMH_CMD_DispatchCacheRecord(cmd->SessionId, cmd->SequenceNumber,
                                         LRMH_DISPATCH_OUTCOME_REJECTED);
        }
        return;
    }

    /* Forward to lrmh_seq with the full 2.0 payload. The token
     * epoch bump is the LAST step before the SB issue, so a
     * precondition rejection above does not waste an epoch. */
    LRMH_CMD_BroadcastTokenReset(LRMH_TOKEN_RESET_PHASE_ADVANCE);

    /* CFE_MSG_Init zeroes the buffer before populating the header:
     * initialize first, then copy the body fields. */
    LRMH_StartSiteTestCmd_t fwd;
    CFE_MSG_Init(CFE_MSG_PTR(fwd.CmdHeader),
                 CFE_SB_ValueToMsgId(LRMH_SEQ_CMD_MID), sizeof(fwd));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(fwd.CmdHeader),
                       LRMH_START_SITE_TEST_CC);
    fwd.SiteId              = cmd->SiteId;
    fwd.TestProfileId       = cmd->TestProfileId;
    fwd.ToolId              = cmd->ToolId;
    fwd.MaxDepth_m          = cmd->MaxDepth_m;
    fwd.MaxLoad_N           = cmd->MaxLoad_N;
    fwd.EnableRepeatLoad    = cmd->EnableRepeatLoad;
    fwd.EnablePostProfile   = cmd->EnablePostProfile;
    fwd.EnableTraction      = cmd->EnableTraction;
    fwd.DispatchFlags       = cmd->DispatchFlags;
    fwd.SessionId           = cmd->SessionId;
    fwd.SequenceNumber      = cmd->SequenceNumber;
    fwd.PairedSiteId        = cmd->PairedSiteId;
    fwd.MicroSiteOffsetX_mm = cmd->MicroSiteOffsetX_mm;
    fwd.MicroSiteOffsetY_mm = cmd->MicroSiteOffsetY_mm;
    fwd.RepeatLoadCount     = cmd->RepeatLoadCount;
    CFE_SB_TransmitMsg(CFE_MSG_PTR(fwd.CmdHeader), true);

    /* Mode transition READY → TESTING */
    LRMH_CMD_AdvanceMode(LRMH_TRIGGER_START_SITE_TEST);

    /* Record the dispatch in the cache and the HK fields */
    if (cmd->DispatchFlags & LRMH_DISPATCH_IDEMPOTENT) {
        LRMH_CMD_DispatchCacheRecord(cmd->SessionId, cmd->SequenceNumber,
                                     LRMH_DISPATCH_OUTCOME_IN_PROGRESS);
    }
    g_CmdApp.CurrentSessionId      = cmd->SessionId;
    g_CmdApp.CurrentSequenceNumber = cmd->SequenceNumber;
    g_CmdApp.CurrentSiteId         = cmd->SiteId;

    g_CmdApp.CmdCount++;
    CFE_EVS_SendEvent(LRMH_CMD_DISPATCH_OK_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_CMD: START_SITE_TEST Session=%u Seq=%u Site=%u "
                      "Profile=%u Tool=%u",
                      (unsigned)cmd->SessionId,
                      (unsigned)cmd->SequenceNumber,
                      (unsigned)cmd->SiteId,
                      (unsigned)cmd->TestProfileId,
                      (unsigned)cmd->ToolId);
}

static void LRMH_CMD_SetMode(const LRMH_SetModeCmd_t *cmd)
{
    /* SET_MODE is a debugging affordance — ground forcing a mode
     * transition. It still goes through the transition matrix; it
     * does not bypass the FSM. The matrix lookup needs a synthetic
     * trigger to express "ground forcing this mode," which we
     * approximate by mapping the requested target to the
     * appropriate trigger. */
    (void)cmd;  /* not implemented: the test profile space is owned by
                 * lrmh_seq and SET_MODE has no defined semantics yet */
    g_CmdApp.ErrCount++;
    CFE_EVS_SendEvent(LRMH_CMD_FCN_ERR_EID, CFE_EVS_EventType_ERROR,
                      "LRMH_CMD: SET_MODE not implemented");
}
