/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_fault_app.c
 * @brief LRMH lrmh_fault application — implementation
 *
 * Thin orchestrator per ADR-0017: threshold detection lives in
 * the LC tables (tables/lc_watchpoints.c, thresholds from
 * lrmh_platform_cfg.h), this app makes no HAL calls, and the
 * per-fault tier table is the only policy.
 *
 * The app does three things:
 *
 *   1. Receive LC notifications via LRMH_FAULT_LC_NOTIFY_CC
 *      (an SC notify RTS delivers the fault flag)
 *   2. Run the polled path at 10 Hz for the one state-watching
 *      category implemented so far (SENSOR_TIMEOUT)
 *   3. Look up the recovery tier from a static array indexed
 *      by fault flag bit position, and dispatch the
 *      corresponding cross-app commands
 *
 * Coalescing per ADR-0013: if a fault category is already in
 * `FaultFlags`, further raises are silently absorbed until the
 * recovery completes (clearing the bit).
 */

#include "lrmh_fault_app.h"

/* LC re-arm. LC sets an actionpoint PASSIVE after it requests its RTS,
 * so without this each fault category could fire only once per boot.
 * The command shape mirrors LC_SetAPStateCmd_t (lc_msgstruct.h); the
 * values are LC's SET_AP_STATE function code, LC_ALL_ACTIONPOINTS and
 * LC_APSTATE_ACTIVE. Defined locally so this app does not depend on
 * LC's headers. */
#define LRMH_LC_SET_AP_STATE_CC   3
#define LRMH_LC_ALL_ACTIONPOINTS  0xFFFF
#define LRMH_LC_APSTATE_ACTIVE    1

typedef struct {
    CFE_MSG_CommandHeader_t CommandHeader;
    uint16 APNumber;
    uint16 NewAPState;
} LRMH_LcSetApStateCmd_t;

static void LRMH_FAULT_RearmActionpoints(void)
{
    LRMH_LcSetApStateCmd_t cmd;
    CFE_MSG_Init(CFE_MSG_PTR(cmd.CommandHeader),
                 CFE_SB_ValueToMsgId(LRMH_LC_CMD_MID), sizeof(cmd));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(cmd.CommandHeader), LRMH_LC_SET_AP_STATE_CC);
    cmd.APNumber   = LRMH_LC_ALL_ACTIONPOINTS;
    cmd.NewAPState = LRMH_LC_APSTATE_ACTIVE;
    CFE_SB_TransmitMsg(CFE_MSG_PTR(cmd.CommandHeader), true);
    CFE_EVS_SendEvent(LRMH_FAULT_AP_REARM_EID, CFE_EVS_EventType_INFORMATION,
                      "LRMH_FAULT: LC actionpoints re-armed (all ACTIVE)");
}

#include <string.h>

LRMH_FAULT_AppData_t g_FaultApp;

/* ============================================================
 * Per-fault tier table (ADR-0008)
 *
 * Indexed by fault flag bit position (0..10). The table is a
 * static const array because the policy never changes at
 * runtime — ADR-0008 pins each fault to one tier.
 * ============================================================ */
static const LRMH_RecoveryTier_t fault_tier_table[LRMH_FAULT_FLAG_COUNT] = {
    [0]  = LRMH_TIER_T2_SKIP,    /* STALL */
    [1]  = LRMH_TIER_T3_SAFE,    /* OVERCURRENT */
    [2]  = LRMH_TIER_T2_SKIP,    /* FORCE_SATURATION */
    [3]  = LRMH_TIER_T1_RESUME,  /* ENCODER_LOSS */
    [4]  = LRMH_TIER_T3_SAFE,    /* THERMAL */
    [5]  = LRMH_TIER_T1_RESUME,  /* FORCE_RATE */
    [6]  = LRMH_TIER_T2_SKIP,    /* HARD_STOP */
    [7]  = LRMH_TIER_T1_RESUME,  /* SENSOR_TIMEOUT */
    [8]  = LRMH_TIER_T2_SKIP,    /* DEPTH_DISAGREE */
    [9]  = LRMH_TIER_T3_SAFE,    /* BUS_VOLTAGE */
    [10] = LRMH_TIER_T2_SKIP,    /* HOST_NAV_TIMEOUT */
};

/* ============================================================
 * Forward declarations
 * ============================================================ */
static int32 LRMH_FAULT_Init(void);
static void  LRMH_FAULT_ProcessMsg(const CFE_SB_Buffer_t *BufPtr);
static uint64 LRMH_FAULT_NowUs(void);
static int    LRMH_FAULT_BitPosition(uint16 fault_flag);
static void   LRMH_FAULT_DispatchT1(uint16 fault_flag);
static void   LRMH_FAULT_DispatchT2(uint16 fault_flag);
static void   LRMH_FAULT_DispatchT3(uint16 fault_flag);
static void   LRMH_FAULT_SendSeqRecovery(uint8 fault_code, uint8 tier);

/* ============================================================
 * Helpers
 * ============================================================ */
static int LRMH_FAULT_BitPosition(uint16 fault_flag)
{
    /* Return the bit index 0..10 of a single-bit fault flag,
     * or -1 if the input is not a single bit set. */
    if (fault_flag == 0 || (fault_flag & (fault_flag - 1)) != 0) return -1;
    int pos = 0;
    while ((fault_flag & 1) == 0) {
        fault_flag >>= 1;
        pos++;
    }
    return pos < LRMH_FAULT_FLAG_COUNT ? pos : -1;
}

static uint64 LRMH_FAULT_NowUs(void)
{
    CFE_TIME_SysTime_t t = CFE_TIME_GetTime();
    uint64 us = (uint64)t.Seconds * 1000000ULL;
    us += ((uint64)t.Subseconds * 1000000ULL) >> 32;
    return us;
}

LRMH_RecoveryTier_t LRMH_FAULT_LookupTier(uint16 fault_flag)
{
    int pos = LRMH_FAULT_BitPosition(fault_flag);
    if (pos < 0) return LRMH_TIER_NONE;
    return fault_tier_table[pos];
}

/* ============================================================
 * Cross-app dispatch helpers
 * ============================================================ */
static void LRMH_FAULT_SendSeqRecovery(uint8 fault_code, uint8 tier)
{
    /* Send a recovery command to lrmh_seq on
     * LRMH_FAULT_TO_SEQ_MID. The carrier struct mirrors
     * apps/lrmh_seq/fsw/inc/lrmh_seq_msg.h
     * LRMH_SeqRecoveryCmd_t. */
    struct {
        CFE_MSG_CommandHeader_t CmdHeader;
        uint8  FaultCode;
        uint8  Tier;
        uint8  Reserved[2];
    } cmd;
    memset(&cmd, 0, sizeof(cmd));
    CFE_MSG_Init(CFE_MSG_PTR(cmd.CmdHeader),
                 CFE_SB_ValueToMsgId(LRMH_FAULT_TO_SEQ_MID), sizeof(cmd));
    cmd.FaultCode = fault_code;
    cmd.Tier      = tier;
    CFE_SB_TransmitMsg(CFE_MSG_PTR(cmd.CmdHeader), true);
}

/* ============================================================
 * Tier dispatchers (ADR-0008)
 * ============================================================ */
static void LRMH_FAULT_DispatchT1(uint16 fault_flag)
{
    g_FaultApp.T1AttemptsCurrentPhase++;
    g_FaultApp.TotalT1Events++;

    if (g_FaultApp.T1AttemptsCurrentPhase >
        LRMH_T1_MAX_ATTEMPTS_PER_PHASE) {
        /* T1 → T2 escalation per ADR-0008 */
        CFE_EVS_SendEvent(LRMH_FAULT_ESCALATE_T1_T2_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "LRMH_FAULT: T1→T2 escalation (fault=0x%04X, "
                          "%u T1 attempts)",
                          (unsigned)fault_flag,
                          (unsigned)g_FaultApp.T1AttemptsCurrentPhase);
        LRMH_FAULT_DispatchT2(fault_flag);
        return;
    }

    LRMH_FAULT_SendSeqRecovery((uint8)fault_flag, LRMH_TIER_T1_RESUME);

    /* The resume is the whole recovery: clear the bit so a repeat of
     * the same condition counts as a new attempt (and escalates), and
     * re-arm LC so the watchpoint can fire again. */
    g_FaultApp.FaultFlags &= (uint16)~fault_flag;
    g_FaultApp.FaultActive = (g_FaultApp.FaultFlags != 0) ? 1 : 0;
    LRMH_FAULT_RearmActionpoints();

    g_FaultApp.LastRecoveryTier = LRMH_TIER_T1_RESUME;
    CFE_EVS_SendEvent(LRMH_FAULT_T1_DISPATCH_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_FAULT: T1 RESUME (fault=0x%04X, attempt %u)",
                      (unsigned)fault_flag,
                      (unsigned)g_FaultApp.T1AttemptsCurrentPhase);
}

static uint8 LRMH_FAULT_CountBits(uint8 v)
{
    uint8 n = 0;
    while (v) { n += (uint8)(v & 1u); v >>= 1; }
    return n;
}

static void LRMH_FAULT_DispatchT2(uint16 fault_flag)
{
    /* One mark per site: the escalation rule is "T2 skips in the
     * last LRMH_T2_ESCALATION_WINDOW sites", not T2 events. */
    g_FaultApp.T2SiteHistory |= 1u;
    g_FaultApp.T2EventsInWindow = LRMH_FAULT_CountBits(
        (uint8)(g_FaultApp.T2SiteHistory & ((1u << LRMH_T2_ESCALATION_WINDOW) - 1u)));
    g_FaultApp.TotalT2Events++;

    if (g_FaultApp.T2EventsInWindow >=
        LRMH_T2_ESCALATION_THRESHOLD) {
        /* T2 → T3 escalation per ADR-0008 */
        CFE_EVS_SendEvent(LRMH_FAULT_ESCALATE_T2_T3_EID,
                          CFE_EVS_EventType_CRITICAL,
                          "LRMH_FAULT: T2→T3 escalation (fault=0x%04X, "
                          "%u T2 events in window)",
                          (unsigned)fault_flag,
                          (unsigned)g_FaultApp.T2EventsInWindow);
        LRMH_FAULT_DispatchT3(fault_flag);
        return;
    }

    LRMH_FAULT_SendSeqRecovery((uint8)fault_flag, LRMH_TIER_T2_SKIP);
    LRMH_FAULT_RearmActionpoints();

    g_FaultApp.LastRecoveryTier = LRMH_TIER_T2_SKIP;
    CFE_EVS_SendEvent(LRMH_FAULT_T2_DISPATCH_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_FAULT: T2 SKIP_SITE (fault=0x%04X)",
                      (unsigned)fault_flag);
}

static void LRMH_FAULT_DispatchT3(uint16 fault_flag)
{
    g_FaultApp.TotalT3Events++;

    LRMH_FAULT_SendSeqRecovery((uint8)fault_flag, LRMH_TIER_T3_SAFE);

    /* Also command lrmh_cmd to enter SAFE mode. The carrier is
     * a no-args command with cc = LRMH_CMD_SAFE_CC. Per ADR-0007
     * rule 1, lrmh_cmd accepts SAFE unconditionally. */
    LRMH_NoArgsCmd_t safe_cmd;
    memset(&safe_cmd, 0, sizeof(safe_cmd));
    CFE_MSG_Init(CFE_MSG_PTR(safe_cmd.CmdHeader),
                 CFE_SB_ValueToMsgId(LRMH_CMD_MID), sizeof(safe_cmd));
    CFE_MSG_SetFcnCode(CFE_MSG_PTR(safe_cmd.CmdHeader), LRMH_CMD_SAFE_CC);
    CFE_SB_TransmitMsg(CFE_MSG_PTR(safe_cmd.CmdHeader), true);

    g_FaultApp.LastRecoveryTier = LRMH_TIER_T3_SAFE;
    CFE_EVS_SendEvent(LRMH_FAULT_T3_DISPATCH_EID,
                      CFE_EVS_EventType_CRITICAL,
                      "LRMH_FAULT: T3 SAFE (fault=0x%04X)",
                      (unsigned)fault_flag);
}

/* ============================================================
 * Public: raise a fault
 * ============================================================ */
void LRMH_FAULT_Raise(uint16 fault_flag)
{
    if (!g_FaultApp.FaultEnabled) return;

    /* Coalescing: if already active, suppress (ADR-0013) */
    if (g_FaultApp.FaultFlags & fault_flag) {
        CFE_EVS_SendEvent(LRMH_FAULT_COALESCED_EID,
                          CFE_EVS_EventType_DEBUG,
                          "LRMH_FAULT: coalesced raise of 0x%04X",
                          (unsigned)fault_flag);
        return;
    }

    g_FaultApp.FaultFlags |= fault_flag;
    g_FaultApp.FaultActive = 1;

    /* Per-mode evaluation rules (ADR-0013) */
    if (g_FaultApp.CurrentMode != LRMH_MODE_TESTING) {
        /* In STANDBY/READY/SAFE/FAULT there is no test to recover,
         * so every category dispatches as T3. */
        LRMH_FAULT_DispatchT3(fault_flag);
        return;
    }

    /* In TESTING, dispatch the per-fault tier from the table */
    LRMH_RecoveryTier_t tier = LRMH_FAULT_LookupTier(fault_flag);
    switch (tier) {
    case LRMH_TIER_T1_RESUME:
        if (g_FaultApp.CurrentPhase >= (uint8)LRMH_PHASE_PROCESSING ||
            g_FaultApp.CurrentPhase == (uint8)LRMH_PHASE_IDLE) {
            /* Nothing to resume: the measured phases are over (the
             * final retract is in progress) or no test is running. A
             * transient here is noted and the bit cleared. */
            g_FaultApp.FaultFlags &= (uint16)~fault_flag;
            g_FaultApp.FaultActive = (g_FaultApp.FaultFlags != 0) ? 1 : 0;
            CFE_EVS_SendEvent(LRMH_FAULT_COALESCED_EID, CFE_EVS_EventType_DEBUG,
                              "LRMH_FAULT: T1 0x%04X outside measured phases, ignored",
                              (unsigned)fault_flag);
            break;
        }
        LRMH_FAULT_DispatchT1(fault_flag);
        break;
    case LRMH_TIER_T2_SKIP:
        LRMH_FAULT_DispatchT2(fault_flag);
        break;
    case LRMH_TIER_T3_SAFE:
        LRMH_FAULT_DispatchT3(fault_flag);
        break;
    default:
        g_FaultApp.ErrCount++;
        CFE_EVS_SendEvent(LRMH_FAULT_BAD_FLAG_EID,
                          CFE_EVS_EventType_ERROR,
                          "LRMH_FAULT: unknown fault flag 0x%04X",
                          (unsigned)fault_flag);
        break;
    }

    /* Track SENSOR_TIMEOUT specifically for HK */
    if (fault_flag == LRMH_FAULT_FLAG_SENSOR_TIMEOUT) {
        g_FaultApp.TimeoutCount++;
    }
}

/* ============================================================
 * Phase advance handler (ADR-0008 — T1 counter is per-phase)
 * ============================================================ */
void LRMH_FAULT_OnPhaseAdvance(uint8 new_phase)
{
    LRMH_FAULT_OnPhaseAdvanceAt(new_phase, LRMH_FAULT_NowUs());
}

void LRMH_FAULT_OnPhaseAdvanceAt(uint8 new_phase, uint64 now_us)
{
    if (new_phase == (uint8)LRMH_PHASE_IDLE && g_FaultApp.CurrentPhase != (uint8)LRMH_PHASE_IDLE) {
        /* Site ended: whatever was raised during it has been acted on
         * (T2 skipped the site, T3 safed the instrument). Slide the
         * T2 site window by one site. */
        g_FaultApp.FaultFlags  = 0;
        g_FaultApp.FaultActive = 0;
        g_FaultApp.T2SiteHistory = (uint8)(g_FaultApp.T2SiteHistory << 1);
        g_FaultApp.T2EventsInWindow = LRMH_FAULT_CountBits(
            (uint8)(g_FaultApp.T2SiteHistory & ((1u << LRMH_T2_ESCALATION_WINDOW) - 1u)));
    }
    g_FaultApp.CurrentPhase = new_phase;
    g_FaultApp.PhaseSince_us = now_us;
    g_FaultApp.T1AttemptsCurrentPhase = 0;
}

/* ============================================================
 * Sample TLM handler — caches for polled re-evaluation
 * ============================================================ */
void LRMH_FAULT_OnSampleTlm(const LRMH_SampleTlm_t *sample, uint64 now_us)
{
    g_FaultApp.SampleCache.valid           = true;
    g_FaultApp.SampleCache.receipt_time_us = now_us;
    g_FaultApp.SampleCache.sample          = *sample;
}

/* ============================================================
 * Polled path (ADR-0013)
 *
 * Runs at 10 Hz. Catches state-watching faults that LC's
 * single-sample evaluation cannot express:
 *
 *   - SENSOR_TIMEOUT: no sample TLM for > LRMH_SENSOR_TIMEOUT_MS
 *   - STALL, ENCODER_LOSS, etc. would also live here in a full
 *     implementation; SENSOR_TIMEOUT is implemented to prove the
 *     pattern and the others are left as placeholders.
 * ============================================================ */
void LRMH_FAULT_PolledEvaluate(uint64 now_us)
{
    if (!g_FaultApp.FaultEnabled) return;

    /* SENSOR_TIMEOUT only escalates in TESTING — outside TESTING
     * lrmh_sense intentionally stops acquiring (SAFE broadcast,
     * end of test, etc.) so the cache ages without that
     * indicating a sensor problem. Without this gate the
     * outside-TESTING force-escalation rule promotes the
     * sensor-timeout flag to T3, drives SAFE → FAULT, and gets
     * the recovery loop stuck. */
    if (g_FaultApp.CurrentMode != LRMH_MODE_TESTING) {
        return;
    }

    /* lrmh_sense acquires from POSITIONING's entry to PROCESSING's
     * entry. The floor is enforced from BASELINE_PEN through POST_PEN:
     * POSITIONING is the contact move, where the first samples are
     * still settling, and from PROCESSING on the cache ages because
     * acquisition has stopped, not because the sensors have. */
    if (g_FaultApp.CurrentPhase < LRMH_PHASE_BASELINE_PEN ||
        g_FaultApp.CurrentPhase > LRMH_PHASE_POST_PEN) {
        return;
    }

    /* SENSOR_TIMEOUT: nothing yet this phase, or a stale cache. A sample
     * from before the phase began does not count, and the floor is
     * measured from the phase's start until the first one arrives. */
    uint64 since_us = g_FaultApp.PhaseSince_us;
    if (!g_FaultApp.SampleCache.valid ||
        g_FaultApp.SampleCache.receipt_time_us < since_us) {
        if (now_us > since_us &&
            (now_us - since_us) / 1000ULL > LRMH_SENSOR_TIMEOUT_MS) {
            LRMH_FAULT_Raise(LRMH_FAULT_FLAG_SENSOR_TIMEOUT);
        }
        return;
    }

    uint64 age_us = (now_us > g_FaultApp.SampleCache.receipt_time_us)
                  ? (now_us - g_FaultApp.SampleCache.receipt_time_us)
                  : 0;
    uint64 age_ms = age_us / 1000ULL;
    if (age_ms > LRMH_SENSOR_TIMEOUT_MS) {
        LRMH_FAULT_Raise(LRMH_FAULT_FLAG_SENSOR_TIMEOUT);
    }
}

/* ============================================================
 * Reset all
 * ============================================================ */
void LRMH_FAULT_ResetAll(void)
{
    g_FaultApp.FaultFlags             = 0;
    g_FaultApp.FaultActive            = 0;
    g_FaultApp.LastRecoveryTier       = LRMH_TIER_NONE;
    g_FaultApp.T1AttemptsCurrentPhase = 0;
    g_FaultApp.T2EventsInWindow       = 0;
    g_FaultApp.T2SiteHistory          = 0;
    g_FaultApp.SampleCache.valid      = false;
    CFE_EVS_SendEvent(LRMH_FAULT_RESET_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_FAULT: state reset");
}

/* ============================================================
 * Status TLM publication
 * ============================================================ */
void LRMH_FAULT_PublishStatus(void)
{
    LRMH_FaultTlm_t tlm;
    memset(&tlm, 0, sizeof(tlm));
    CFE_MSG_Init(CFE_MSG_PTR(tlm.TlmHeader),
                 CFE_SB_ValueToMsgId(LRMH_FAULT_TLM_MID), sizeof(tlm));

    tlm.FaultFlags             = g_FaultApp.FaultFlags;
    tlm.FaultActive            = g_FaultApp.FaultActive;
    tlm.RecoveryTier           = g_FaultApp.LastRecoveryTier;
    tlm.T1AttemptsCurrentPhase = g_FaultApp.T1AttemptsCurrentPhase;
    tlm.T2EventsInWindow       = g_FaultApp.T2EventsInWindow;
    tlm.FaultEnabled           = g_FaultApp.FaultEnabled;
    tlm.TimeoutCount           = g_FaultApp.TimeoutCount;
    tlm.TotalT1Events          = g_FaultApp.TotalT1Events;
    tlm.TotalT2Events          = g_FaultApp.TotalT2Events;
    tlm.TotalT3Events          = g_FaultApp.TotalT3Events;
    tlm.CmdCount               = g_FaultApp.CmdCount;
    tlm.ErrCount               = g_FaultApp.ErrCount;

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(tlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(tlm.TlmHeader), true);
}

/* ============================================================
 * App main
 * ============================================================ */
void LRMH_FAULT_AppMain(void)
{
    int32 Status;
    CFE_SB_Buffer_t *BufPtr = NULL;
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = LRMH_FAULT_Init();
    if (Status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog("LRMH_FAULT: Init failed\n");
        CFE_ES_ExitApp(CFE_ES_RunStatus_APP_ERROR);
        return;
    }

    CFE_EVS_SendEvent(LRMH_FAULT_INIT_INF_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_FAULT: thin orchestrator initialized");

    while (CFE_ES_RunLoop(&RunStatus) == true) {
        Status = CFE_SB_ReceiveBuffer(&BufPtr, g_FaultApp.CmdPipe, 100);
        bool tick = (Status != CFE_SUCCESS);   /* pipe timeout: the 10 Hz tick */
        if (Status == CFE_SUCCESS && BufPtr != NULL) {
            CFE_SB_MsgId_t MsgId;
            CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);
            if (CFE_SB_MsgIdToValue(MsgId) == LRMH_FAULT_WAKEUP_MID) {
                tick = true;
            }
            LRMH_FAULT_ProcessMsg(BufPtr);
        }
        else if (Status != CFE_SB_TIME_OUT) {
            /* A software-bus receive error is not something the app can
             * recover from in place: event it and let ES restart the app
             * (HS policy) rather than spin on a broken pipe. */
            g_FaultApp.ErrCount++;
            CFE_EVS_SendEvent(LRMH_FAULT_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "LRMH_FAULT: SB receive error 0x%08lX, exiting",
                              (unsigned long)Status);
            RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
        /* The polled path and the status packet run on the tick, not
         * on every 100 Hz sample this pipe also receives. */
        if (tick) {
            LRMH_FAULT_PolledEvaluate(LRMH_FAULT_NowUs());
            LRMH_FAULT_PublishStatus();
        }
    }

    CFE_ES_ExitApp(RunStatus);
}

/* ============================================================
 * Initialization
 * ============================================================ */
static int32 LRMH_FAULT_Init(void)
{
    int32 Status;
    memset(&g_FaultApp, 0, sizeof(g_FaultApp));

    /* Disabled until lrmh_cmd transitions out of OFF — per
     * ADR-0013 init-time and shutdown safing. */
    g_FaultApp.FaultEnabled = 0;
    g_FaultApp.CurrentMode  = LRMH_MODE_OFF;
    g_FaultApp.LastRecoveryTier = LRMH_TIER_NONE;

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) return Status;

    Status = CFE_SB_CreatePipe(&g_FaultApp.CmdPipe,
                               LRMH_FAULT_PIPE_DEPTH, "LRMH_FAULT_PIPE");
    if (Status != CFE_SUCCESS) return Status;

    /* Subscriptions */
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_FAULT_CMD_MID),
                     g_FaultApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SENSE_TLM_MID),
                     g_FaultApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_FAULT_WAKEUP_MID),
                     g_FaultApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_TOKEN_RESET_MID),
                     g_FaultApp.CmdPipe);

    /* lrmh_cmd publishes mode-FSM state on this MID; we track
     * CurrentMode so the per-mode evaluation rules in
     * LRMH_FAULT_Raise (ADR-0013) work, and we toggle
     * FaultEnabled in lockstep with mode != OFF (ADR-0013 init
     * + shutdown safing). Without this subscription CurrentMode
     * would stay at MODE_OFF forever and every fault would
     * force-escalate to T3 SAFE regardless of the per-fault
     * tier table — which would mask the per-flag dispatch path
     * lrmh_fault is supposed to drive. */
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_CMD_STATUS_MID),
                     g_FaultApp.CmdPipe);

    return CFE_SUCCESS;
}

/* ============================================================
 * Message router
 * ============================================================ */
static void LRMH_FAULT_ProcessMsg(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_SB_MsgId_t MsgId;
    CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId)) {

    case LRMH_FAULT_CMD_MID: {
        CFE_MSG_FcnCode_t fcn;
        CFE_MSG_GetFcnCode(&BufPtr->Msg, &fcn);
        switch (fcn) {

        case LRMH_FAULT_NOOP_CC:
            g_FaultApp.CmdCount++;
            CFE_EVS_SendEvent(LRMH_FAULT_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "LRMH_FAULT: NOOP Cmd=%u Err=%u flags=0x%04X",
                              (unsigned)g_FaultApp.CmdCount, (unsigned)g_FaultApp.ErrCount,
                              (unsigned)g_FaultApp.FaultFlags);
            break;

        case LRMH_FAULT_LC_NOTIFY_CC: {
            const LRMH_FaultLcNotifyCmd_t *cmd =
                (const LRMH_FaultLcNotifyCmd_t *)BufPtr;
            CFE_EVS_SendEvent(LRMH_FAULT_LC_NOTIFY_EID,
                              CFE_EVS_EventType_INFORMATION,
                              "LRMH_FAULT: LC notify fault=0x%04X",
                              (unsigned)cmd->FaultFlag);
            LRMH_FAULT_Raise(cmd->FaultFlag);
            g_FaultApp.CmdCount++;
            break;
        }

        case LRMH_FAULT_PHASE_ADVANCE_CC: {
            const LRMH_FaultPhaseAdvanceCmd_t *cmd =
                (const LRMH_FaultPhaseAdvanceCmd_t *)BufPtr;
            LRMH_FAULT_OnPhaseAdvance(cmd->NewPhase);
            g_FaultApp.CmdCount++;
            break;
        }

        case LRMH_FAULT_RESET_CC:
            LRMH_FAULT_ResetAll();
            g_FaultApp.CmdCount++;
            break;

        case LRMH_FAULT_ENABLE_CC:
            g_FaultApp.FaultEnabled = 1;
            g_FaultApp.CmdCount++;
            break;

        case LRMH_FAULT_DISABLE_CC:
            g_FaultApp.FaultEnabled = 0;
            g_FaultApp.CmdCount++;
            break;

        default:
            g_FaultApp.ErrCount++;
            break;
        }
        break;
    }

    case LRMH_SENSE_TLM_MID:
        LRMH_FAULT_OnSampleTlm((const LRMH_SampleTlm_t *)BufPtr,
                                LRMH_FAULT_NowUs());
        break;

    case LRMH_FAULT_WAKEUP_MID:
        /* Polled evaluation runs after this returns in AppMain */
        break;

    case LRMH_TOKEN_RESET_MID: {
        /* On token reset (SAFE entry, INIT_INSTRUMENT, a new test, a
         * restart), clear active fault state but keep the totals
         * counters. Re-arm the LC actionpoints on every reason except
         * SAFE entry: the health watchpoint that just drove the T3 is
         * still fresh at that moment and would fire again at once,
         * turning SAFE into FAULT. The operator's INIT re-arms. */
        const LRMH_TokenResetTlm_t *reset = (const LRMH_TokenResetTlm_t *)BufPtr;
        g_FaultApp.FaultFlags = 0;
        g_FaultApp.FaultActive = 0;
        if (reset->ResetReason != (uint8)LRMH_TOKEN_RESET_SAFE_ENTRY) {
            LRMH_FAULT_RearmActionpoints();
        }
        break;
    }

    case LRMH_CMD_STATUS_MID: {
        /* lrmh_cmd publishes mode-FSM state. Track the current
         * mode so the per-mode evaluation rules in
         * LRMH_FAULT_Raise dispatch the correct tier, and
         * enable/disable orchestration per the OFF-vs-rest
         * boundary (ADR-0013). */
        const LRMH_CmdTlm_t *cmd = (const LRMH_CmdTlm_t *)BufPtr;
        uint8 new_mode = cmd->Mode;
        if (new_mode != g_FaultApp.CurrentMode) {
            CFE_EVS_SendEvent(LRMH_FAULT_MODE_TRACK_EID,
                              CFE_EVS_EventType_INFORMATION,
                              "LRMH_FAULT: mode tracker %u → %u "
                              "(enabled=%u)",
                              (unsigned)g_FaultApp.CurrentMode,
                              (unsigned)new_mode,
                              (new_mode != LRMH_MODE_OFF) ? 1u : 0u);
        }
        g_FaultApp.CurrentMode = new_mode;
        g_FaultApp.FaultEnabled =
            (new_mode != LRMH_MODE_OFF) ? 1 : 0;
        break;
    }

    default:
        break;
    }
}
