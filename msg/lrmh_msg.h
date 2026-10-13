/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_msg.h
 * @brief LRMH shared message types and enums
 *
 * Shared types referenced from multiple apps. Per-app command/TLM
 * structures live in apps/<app>/fsw/inc/<app>_msg.h.
 *
 * Per ADRs 0004 (mode FSM), 0007 (sequence token), 0014 (dispatch
 * protocol).
 */

#ifndef LRMH_MSG_H
#define LRMH_MSG_H

#include "cfe.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration: token type, defined fully in lrmh_seq_token.h.
 * Avoids the circular include — apps that need the full type include
 * lrmh_seq_token.h directly. */
struct LRMH_SeqToken_t;

/* ============================================================
 * Instrument mode enum (ADR-0004)
 *
 * Six modes describing the instrument's lifecycle state. Owned
 * by lrmh_cmd's mode FSM. Transitions are validated by the
 * transition matrix in lrmh_cmd_app.c.
 * ============================================================ */
typedef enum {
    LRMH_MODE_OFF      = 0,
    LRMH_MODE_STANDBY  = 1,
    LRMH_MODE_READY    = 2,
    LRMH_MODE_TESTING  = 3,
    LRMH_MODE_SAFE     = 4,
    LRMH_MODE_FAULT    = 5,
    LRMH_MODE_COUNT
} LRMH_InstrumentMode_t;

/* ============================================================
 * Site-test phase enum (ADR-0004 phase machine)
 *
 * Nine phases. Five canonical (RMH-F-001/002/003/004) plus four
 * FSW bookends (PRECHECK/POSITIONING/PROCESSING/REPORTING).
 * Owned by lrmh_seq's phase machine.
 * ============================================================ */
typedef enum {
    LRMH_PHASE_IDLE         = 0,
    LRMH_PHASE_PRECHECK     = 1,   /* bookend */
    LRMH_PHASE_POSITIONING  = 2,   /* bookend */
    LRMH_PHASE_BASELINE_PEN = 3,   /* canon 1 / RMH-F-002 */
    LRMH_PHASE_LOAD_RAMP    = 4,   /* canon 2 / RMH-F-001 */
    LRMH_PHASE_REPEAT_LOAD  = 5,   /* canon 3 / RMH-F-001 + RMH-F-003 */
    LRMH_PHASE_POST_PEN     = 6,   /* canon 4 / RMH-F-003 */
    LRMH_PHASE_TRACTION     = 7,   /* canon 5 / RMH-F-004 (deferred stub) */
    LRMH_PHASE_PROCESSING   = 8,   /* bookend */
    LRMH_PHASE_REPORTING    = 9,   /* bookend */
    LRMH_PHASE_COUNT
} LRMH_Phase_t;

/* ============================================================
 * Quality flags for site results
 * ============================================================ */
typedef enum {
    LRMH_QUALITY_VALID    = 0,
    LRMH_QUALITY_PARTIAL  = 1,
    LRMH_QUALITY_FAULT    = 2
} LRMH_Quality_t;

/* ============================================================
 * Recovery tier (ADR-0008)
 * ============================================================ */
typedef enum {
    LRMH_TIER_NONE         = 0,
    LRMH_TIER_T1_RESUME    = 1,
    LRMH_TIER_T2_SKIP      = 2,
    LRMH_TIER_T3_SAFE      = 3
} LRMH_RecoveryTier_t;

/* ============================================================
 * Safing command codes shared by lrmh_cmd, lrmh_fault and lrmh_seq
 *
 * lrmh_fault sends SAFE to lrmh_cmd on a T3; lrmh_cmd forwards a
 * ground ABORT to lrmh_seq, whose ABORT code mirrors this value.
 * ============================================================ */
#define LRMH_CMD_SAFE_CC          10  /* lrmh_cmd: SAFE entry */
#define LRMH_CMD_ABORT_CC          5  /* lrmh_cmd: unconditional T3 */

/* ============================================================
 * No-args command shell (used for NOOP, RESET_COUNTERS, etc.)
 * ============================================================ */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
} LRMH_NoArgsCmd_t;

/* ============================================================
 * Sequencer → cmd FSM notification (LRMH_SEQ_TO_CMD_MID)
 *
 * lrmh_seq publishes this to drive ADR-0004 mode-FSM transitions
 * that the sequencer alone observes. ADR-0004 names the receiver:
 *   "lrmh_seq reports TEST_COMPLETE and *that* triggers the
 *    TESTING → STANDBY transition; phase advances within TESTING
 *    are invisible to the mode FSM."
 *
 * The notify is a state event, not a sequenced data command, so it
 * carries no token (ADR-0007 token validation applies to seq → ctrl/
 * sense/proc/report dataflow only).
 *
 * Lives in this shared header because both lrmh_seq (publisher) and
 * lrmh_cmd (subscriber) need it. Future notify types (e.g. abort
 * acknowledged, recovery completed) extend the NotifyType enum.
 * ============================================================ */
#define LRMH_SEQ_NOTIFY_TEST_COMPLETE   1   /* normal end of REPORTING */

typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
    uint8   NotifyType;             /* LRMH_SEQ_NOTIFY_* */
    uint8   Reserved[3];
} LRMH_SeqToCmdNotify_t;

#ifdef __cplusplus
}
#endif

#endif /* LRMH_MSG_H */
