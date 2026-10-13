/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_seq_token.h
 * @brief LRMH Sequence Token — Inter-app command ordering protection
 *
 * Every sequencer-issued command carries a 32-bit sequence token that
 * ties it to the exact test epoch and sequencer phase that produced it.
 * Receiving apps validate the token before acting on any command.
 *
 * Per ADR-0007 (sequence-token protocol). The LRMH_TOKEN_RESET_MID
 * broadcast carries a phase-start timestamp alongside the epoch
 * struct (per ADR-0009 phase-start sample rejection rule).
 *
 * Token structure: Epoch[16] | SeqState[8] | SeqCount[8]
 *   Epoch    — increments on every restart/INIT/SAFE-entry
 *   SeqState — current phase enum
 *   SeqCount — monotonic per epoch, wraps at 256
 *
 * Validation rules (in order):
 *   1. Unconditional accept when the receiver says so (its own
 *      safing commands: a retract or e-stop, a sense STOP)
 *   2. Token reset broadcast clears accepted state
 *   3. Stale epoch  (incoming.epoch  < accepted.epoch)        → reject
 *   4. Stale state  (incoming.state  < accepted.state, same epoch) → reject
 *   5. Duplicate    (incoming.count <= accepted.count, same state) → reject
 *   6. Otherwise accept
 */

#ifndef LRMH_SEQ_TOKEN_H
#define LRMH_SEQ_TOKEN_H

#include "cfe.h"
#include "lrmh_msg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Token format
 * ============================================================ */
typedef struct {
    uint16  Epoch;
    uint8   SeqState;
    uint8   SeqCount;
} LRMH_SeqToken_t;

/* Pack token into uint32 for inclusion in command packets */
#define LRMH_SEQ_TOKEN_PACK(tok) \
    ((uint32)((tok).Epoch) << 16 | \
     (uint32)((tok).SeqState) << 8 | \
     (uint32)((tok).SeqCount))

#define LRMH_SEQ_TOKEN_UNPACK_EPOCH(packed)    ((uint16)((packed) >> 16))
#define LRMH_SEQ_TOKEN_UNPACK_STATE(packed)    ((uint8) (((packed) >> 8) & 0xFF))
#define LRMH_SEQ_TOKEN_UNPACK_COUNT(packed)    ((uint8) ((packed) & 0xFF))

/* ============================================================
 * Validation result codes (ADR-0007 rules 3–6)
 * ============================================================ */
typedef enum {
    LRMH_TOKEN_ACCEPT                = 0,
    LRMH_TOKEN_REJECT_STALE_EPOCH    = 1,
    LRMH_TOKEN_REJECT_STALE_STATE    = 2,
    LRMH_TOKEN_REJECT_REORDER        = 3,
    LRMH_TOKEN_RESET_REQUIRED        = 4,  /* reserved; not returned today */
    LRMH_TOKEN_UNCONDITIONAL_ACCEPT  = 5   /* SAFE/ABORT path, ADR-0007 rule 1 */
} LRMH_TokenValidationResult_t;

/* Rule 1 is decided by the receiver: every app has its own command
 * code or mode namespace, so the library cannot tell a safing
 * command from an ordinary one. The receiver passes `unconditional`. */

/* ============================================================
 * Token reset broadcast (LRMH_TOKEN_RESET_MID)
 *
 * Sent by lrmh_cmd or lrmh_seq on every epoch bump:
 *   - INIT_INSTRUMENT from ground
 *   - SAFE entry by mode FSM
 *   - lrmh_seq HS restart (cold-start init path)
 *   - lrmh_seq cold start
 *
 * The PhaseStartTime_us field (ADR-0009) lets lrmh_proc reject
 * samples whose acquisition timestamp is older than the current
 * phase's start time.
 * ============================================================ */
typedef struct {
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    uint16  NewEpoch;                /* the epoch we're advancing to */
    uint8   ResetReason;             /* LRMH_TokenResetReason_t */
    uint8   Reserved[5];             /* explicit padding to the 8-byte field below */
    uint64  PhaseStartTime_us;       /* host monotonic; ADR-0009 */
} LRMH_TokenResetTlm_t;

typedef enum {
    LRMH_TOKEN_RESET_INIT_INSTRUMENT = 0,
    LRMH_TOKEN_RESET_SAFE_ENTRY      = 1,
    LRMH_TOKEN_RESET_HS_RESTART      = 2,  /* reserved; an HS restart takes the COLD_START path */
    LRMH_TOKEN_RESET_COLD_START      = 3,
    LRMH_TOKEN_RESET_PHASE_ADVANCE   = 4    /* not an epoch bump; phase boundary marker */
} LRMH_TokenResetReason_t;

/* ============================================================
 * Public API (implemented in msg/lrmh_seq_token.c, compiled into
 * every app that uses it)
 * ============================================================ */

/**
 * @brief Bump the sequence token epoch.
 *
 * Called by lrmh_seq init, lrmh_cmd SAFE entry, and lrmh_cmd
 * INIT_INSTRUMENT. Always followed by a LRMH_TOKEN_RESET_MID
 * broadcast on the SB.
 */
void LRMH_SeqToken_BumpEpoch(LRMH_TokenResetReason_t reason);

/**
 * @brief Stamp a fresh token (called by lrmh_seq before send).
 *
 * Reads the current epoch, the current phase, and the current
 * counter; packs them into the output token; increments the
 * counter for the next call.
 */
void LRMH_SeqToken_Stamp(LRMH_SeqToken_t *out);

/**
 * @brief Validate an incoming token against the receiver's
 * accepted-token state. ADR-0007 rules 1–6.
 *
 * @param incoming  Token from the incoming command
 * @param accepted  Receiver's most-recently-accepted token (in/out;
 *                  updated on accept)
 * @param unconditional  true for the receiver's own safing commands
 *                       (ADR-0007 rule 1); they bypass the token rules
 * @return LRMH_TokenValidationResult_t
 */
LRMH_TokenValidationResult_t LRMH_SeqToken_Validate(
    const LRMH_SeqToken_t *incoming,
    LRMH_SeqToken_t *accepted,
    bool unconditional);

/**
 * @brief Modular "is newer" comparison for the 8-bit count
 * field. Handles wrap at 256.
 */
static inline bool LRMH_SeqToken_IsNewer(uint8 new_count, uint8 old_count) {
    return ((uint8)(new_count - old_count) & 0xFF) < 128 && new_count != old_count;
}

/**
 * @brief Set the current phase (called by lrmh_seq on phase advance).
 * Resets the per-state count so the receiver can disambiguate
 * "first command in new state" from a stale prior-state command.
 */
void LRMH_SeqToken_SetPhase(uint8 new_phase);

/**
 * @brief Get the current epoch (for HK telemetry introspection).
 */
uint16 LRMH_SeqToken_GetCurrentEpoch(void);

#ifdef __cplusplus
}
#endif

#endif /* LRMH_SEQ_TOKEN_H */
