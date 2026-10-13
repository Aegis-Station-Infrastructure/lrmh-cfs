/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_cmd_app.h
 * @brief LRMH lrmh_cmd application — public interface
 *
 * lrmh_cmd is the front door for the instrument. It owns:
 *   - The mode FSM (ADR-0004) — six modes, transition matrix
 *   - The dispatch cache (ADR-0014) — 8-entry idempotency
 *   - The token epoch bumps (ADR-0007) — INIT/SAFE/restart paths
 *   - Ground/host command surface for the instrument
 *
 * Per ADR-0006, lrmh_cmd is deliberately separate from lrmh_seq:
 * a sequencer crash must not take down the command surface.
 */

#ifndef LRMH_CMD_APP_H
#define LRMH_CMD_APP_H

#include "cfe.h"
#include "lrmh_msgids.h"
#include "lrmh_msg.h"
#include "lrmh_seq_token.h"
#include "lrmh_hal.h"
#include "lrmh_platform_cfg.h"

#include "lrmh_cmd_msg.h"
#include "lrmh_cmd_events.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Dispatch cache (ADR-0014)
 *
 * 8-entry FIFO ring of recently-accepted (SessionId, SequenceNumber)
 * pairs and their outcomes. Used for IDEMPOTENT_RETRY support —
 * a retry of a previously-accepted command is silently absorbed.
 * ============================================================ */
#define LRMH_DISPATCH_CACHE_DEPTH  8

typedef enum {
    LRMH_DISPATCH_OUTCOME_NONE        = 0,
    LRMH_DISPATCH_OUTCOME_ACCEPTED    = 1,
    LRMH_DISPATCH_OUTCOME_IN_PROGRESS = 2,
    LRMH_DISPATCH_OUTCOME_COMPLETE    = 3,
    LRMH_DISPATCH_OUTCOME_REJECTED    = 4
} LRMH_DispatchOutcome_t;

typedef struct {
    uint16  SessionId;
    uint16  SequenceNumber;
    uint8   Outcome;             /* LRMH_DispatchOutcome_t */
    uint8   Reserved[3];
} LRMH_DispatchCacheEntry_t;

/* ============================================================
 * Host pose cache (ADR-0011, ADR-0014 precondition)
 *
 * lrmh_cmd subscribes to LRMH_HOST_POSE_MID directly to gate
 * START_SITE_TEST on host nav freshness. lrmh_sense maintains
 * its own (richer) cache for sample alignment; lrmh_cmd's cache
 * is single-purpose and minimal.
 * ============================================================ */
typedef struct {
    bool      valid;
    uint64_t  receipt_time_us;
    uint8     nav_state;         /* AEGIS_NAV_STATE_* from rover */
    uint8     reserved[3];
} LRMH_CmdHostPoseCache_t;

/* ============================================================
 * App data structure
 * ============================================================ */
typedef struct {
    CFE_SB_PipeId_t        CmdPipe;

    /* Mode FSM (ADR-0004) */
    LRMH_InstrumentMode_t  Mode;

    /* Counters */
    uint32                 CmdCount;
    uint32                 ErrCount;
    uint32                 RejectedCount;       /* illegal mode transitions */
    uint32                 DispatchCacheHits;   /* idempotent retries absorbed */

    /* Dispatch cache (ADR-0014) */
    LRMH_DispatchCacheEntry_t DispatchCache[LRMH_DISPATCH_CACHE_DEPTH];
    uint8                  DispatchCacheHead;   /* FIFO head */

    /* Most recently dispatched site (for HK) */
    uint16                 CurrentSessionId;
    uint16                 CurrentSequenceNumber;
    uint16                 CurrentSiteId;

    /* Host pose cache (ADR-0011) */
    LRMH_CmdHostPoseCache_t HostPose;
} LRMH_CMD_AppData_t;

/* ============================================================
 * Public entry point
 * ============================================================ */
void LRMH_CMD_AppMain(void);

/* ============================================================
 * Internal API exposed for unit testing only.
 *
 * These are static-equivalent functions that the standalone test
 * (apps/lrmh_cmd/fsw/test/test_lrmh_cmd.c) calls directly via the
 * include-the-source unit-test pattern. They are NOT intended for
 * production use from other apps.
 * ============================================================ */
extern LRMH_CMD_AppData_t g_CmdApp;

/* Mode FSM transition matrix (ADR-0004) */
typedef enum {
    LRMH_TRIGGER_POWER_ON           = 0,
    LRMH_TRIGGER_DEPLOY             = 1,
    LRMH_TRIGGER_READY_FOR_TEST     = 2,
    LRMH_TRIGGER_STOW               = 3,
    LRMH_TRIGGER_START_SITE_TEST    = 4,
    LRMH_TRIGGER_TEST_COMPLETE      = 5,
    LRMH_TRIGGER_FAULT_T1_T2        = 6,
    LRMH_TRIGGER_FAULT_T3           = 7,
    LRMH_TRIGGER_ABORT              = 8,
    LRMH_TRIGGER_INIT_INSTRUMENT    = 9,
    LRMH_TRIGGER_RESET_FAULT        = 10,
    LRMH_TRIGGER_EMERGENCY_OFF      = 11,
    LRMH_TRIGGER_POWER_OFF          = 12,
    LRMH_TRIGGER_COUNT
} LRMH_ModeTrigger_t;

bool LRMH_CMD_TransitionAllowed(LRMH_InstrumentMode_t from,
                                LRMH_ModeTrigger_t trigger,
                                LRMH_InstrumentMode_t *to_out);

void LRMH_CMD_BroadcastTokenReset(LRMH_TokenResetReason_t reason);

LRMH_DispatchOutcome_t LRMH_CMD_DispatchCacheLookup(uint16 session, uint16 seq);

void LRMH_CMD_DispatchCacheRecord(uint16 session, uint16 seq,
                                  LRMH_DispatchOutcome_t outcome);

void LRMH_CMD_PublishStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* LRMH_CMD_APP_H */
