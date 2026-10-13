/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_fault_app.h
 * @brief LRMH lrmh_fault application — public interface
 *
 * Per ADR-0017 (FDIR via LC + SC + HS) and ADR-0008 (recovery
 * tiers): thin orchestrator. Detection moves to LC tables;
 * lrmh_fault is a pure dispatcher that:
 *
 *   - Receives LC actionpoint notifications via SB
 *   - Looks up the recovery tier from the per-fault table
 *   - Executes T1 (RESUME_PHASE), T2 (SKIP_SITE), or T3 (SAFE)
 *     dispatch via cross-app commands
 *   - Maintains escalation counters per ADR-0008
 *   - Runs the polled path from ADR-0013 for state-watching
 *     fault categories that LC cannot easily express
 *   - Publishes LRMH_FAULT_TLM_MID for lrmh_hk aggregation
 *
 * A pure LC + SC + mode-FSM design would dissolve fault dispatch
 * into tables. LRMH keeps this thin app because T1 RESUME_PHASE
 * recovery needs cross-app coordination (in-progress phase
 * parameters, escalation counters) that does not fit cleanly into
 * SC RTSes alone (ADR-0017).
 */

#ifndef LRMH_FAULT_APP_H
#define LRMH_FAULT_APP_H

#include "cfe.h"
#include "lrmh_msgids.h"
#include "lrmh_msg.h"
#include "lrmh_platform_cfg.h"

/* Bring in subscribed-from message shapes */
#include "lrmh_sense_msg.h"
#include "lrmh_cmd_msg.h"   /* LRMH_CmdTlm_t — for mode tracking */

#include "lrmh_fault_msg.h"
#include "lrmh_seq_token.h"   /* LRMH_TokenResetTlm_t, reset reasons */
#include "lrmh_fault_events.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Fault sample cache (ADR-0013 polled re-evaluation)
 * ============================================================ */
typedef struct {
    bool     valid;
    uint64_t receipt_time_us;
    LRMH_SampleTlm_t sample;
} LRMH_FaultSampleCache_t;

/* ============================================================
 * App data
 * ============================================================ */
typedef struct {
    CFE_SB_PipeId_t  CmdPipe;

    /* Active fault state */
    uint16   FaultFlags;             /* bitmask of currently-active categories */
    uint8    FaultActive;
    uint8    LastRecoveryTier;       /* most recent dispatch */
    uint8    FaultEnabled;
    uint8    CurrentMode;            /* LRMH_InstrumentMode_t cache */
    uint8    CurrentPhase;           /* LRMH_Phase_t cache (from phase advance cmd) */
    uint8    Reserved8;
    uint64   PhaseSince_us;          /* when CurrentPhase began: the sensor floor is measured from here */

    /* Escalation counters (ADR-0008) */
    uint8    T1AttemptsCurrentPhase;
    uint8    T2EventsInWindow;       /* sites with a T2 in the last LRMH_T2_ESCALATION_WINDOW sites */
    uint8    T2SiteHistory;          /* bit 0 = current site had a T2, bit n = n sites ago */
    uint8    Reserved8b;
    uint32   TotalT1Events;
    uint32   TotalT2Events;
    uint32   TotalT3Events;
    uint32   TimeoutCount;           /* SENSOR_TIMEOUT specifically */

    /* Polled path (ADR-0013) state */
    LRMH_FaultSampleCache_t SampleCache;

    /* Counters */
    uint32   CmdCount;
    uint32   ErrCount;
} LRMH_FAULT_AppData_t;

/* ============================================================
 * Public entry point
 * ============================================================ */
void LRMH_FAULT_AppMain(void);

/* ============================================================
 * Test-exposed internal API
 * ============================================================ */
extern LRMH_FAULT_AppData_t g_FaultApp;

/**
 * @brief Look up the recovery tier for a fault flag.
 * @param fault_flag  one of LRMH_FAULT_FLAG_*
 * @return LRMH_RecoveryTier_t (T1/T2/T3 or NONE for unknown)
 */
LRMH_RecoveryTier_t LRMH_FAULT_LookupTier(uint16 fault_flag);

/**
 * @brief Raise a fault — dispatch the recovery tier and update
 * the active flag, escalation counters, and totals.
 *
 * Called by both the event-driven path (LC notify) and the
 * polled path (10 Hz wakeup). Coalescing: if the fault category
 * is already in `FaultFlags`, the call is silently absorbed.
 */
void LRMH_FAULT_Raise(uint16 fault_flag);

/**
 * @brief Polled-path evaluation. Runs at 10 Hz.
 *
 * Implements SENSOR_TIMEOUT (sample cache too old during the
 * measured phases). The other state-watching categories (STALL,
 * ENCODER_LOSS, DEPTH_DISAGREE, HOST_NAV_TIMEOUT) have tier-table
 * entries but no detector yet. The threshold-based categories
 * (FORCE_SATURATION, OVERCURRENT, THERMAL, BUS_VOLTAGE, FORCE_RATE)
 * are LC's job.
 */
void LRMH_FAULT_PolledEvaluate(uint64 now_us);

/**
 * @brief Phase advance from lrmh_seq (LRMH_FAULT_PHASE_ADVANCE_CC).
 *
 * Resets the per-phase T1 attempt counter and the sensor floor's
 * reference time. A transition to IDLE is a site end: the active
 * fault set clears and the T2 site window slides by one.
 */
void LRMH_FAULT_OnPhaseAdvance(uint8 new_phase);

/** The same, at a given time (unit tests). */
void LRMH_FAULT_OnPhaseAdvanceAt(uint8 new_phase, uint64 now_us);

/**
 * @brief Cache an incoming sample for the polled re-evaluation
 * path.
 */
void LRMH_FAULT_OnSampleTlm(const LRMH_SampleTlm_t *sample, uint64 now_us);

/**
 * @brief Reset all fault state. Called on init and on
 * LRMH_FAULT_RESET_CC.
 */
void LRMH_FAULT_ResetAll(void);

/**
 * @brief Publish LRMH_FAULT_TLM_MID with current state.
 */
void LRMH_FAULT_PublishStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* LRMH_FAULT_APP_H */
