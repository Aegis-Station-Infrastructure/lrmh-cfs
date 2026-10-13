/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_seq_token.c
 * @brief LRMH sequence-token library — real implementation
 *
 * Per ADR-0007 (sequence-token protocol). Provides the three
 * helper functions declared in msg/lrmh_seq_token.h:
 *
 *   LRMH_SeqToken_BumpEpoch  — called by lrmh_seq init, lrmh_cmd
 *                              SAFE entry, and lrmh_cmd
 *                              INIT_INSTRUMENT to advance the
 *                              global epoch
 *   LRMH_SeqToken_Stamp      — called by lrmh_seq immediately
 *                              before sending any command
 *   LRMH_SeqToken_Validate   — called by every receiving app's
 *                              command dispatcher; first thing
 *                              it does
 *
 * The library maintains a single global current-epoch counter
 * (used by Stamp). Each receiving app maintains its own
 * accepted-token state (passed in/out of Validate).
 *
 * Standalone unit tests stub these functions locally; this
 * implementation is linked into the cFS image only.
 */

#include "lrmh_seq_token.h"

/* ============================================================
 * Library state — current epoch, current phase, current count
 * ============================================================ */
static LRMH_SeqToken_t g_current = {
    .Epoch    = 0,
    .SeqState = 0,
    .SeqCount = 0
};

/* ============================================================
 * BumpEpoch — advance the epoch and reset state/count
 *
 * Called on:
 *   - lrmh_seq cold start (init function)
 *   - lrmh_cmd INIT_INSTRUMENT command
 *   - lrmh_cmd SAFE entry (mode FSM transition)
 *   - lrmh_seq HS restart (cold-start path)
 *
 * The reason field is not stored in library state; it's used by
 * the caller to populate the LRMH_TokenResetTlm_t broadcast.
 * ============================================================ */
void LRMH_SeqToken_BumpEpoch(LRMH_TokenResetReason_t reason)
{
    (void)reason;
    g_current.Epoch++;
    g_current.SeqState = 0;
    g_current.SeqCount = 0;
}

/**
 * @brief Set the current phase (called by lrmh_seq on phase advance).
 *
 * Resets the per-state count to zero so the receiver can
 * disambiguate "first command in new state" from a stale
 * command from the previous state.
 *
 */
void LRMH_SeqToken_SetPhase(uint8 new_phase)
{
    g_current.SeqState = new_phase;
    g_current.SeqCount = 0;
}

/* ============================================================
 * Stamp — pack the current epoch+state+count into a token
 *
 * Increments the count for the next call. Atomic with respect to
 * the sequencer's main loop (single-threaded by cFS app model).
 * ============================================================ */
void LRMH_SeqToken_Stamp(LRMH_SeqToken_t *out)
{
    if (!out) return;
    out->Epoch    = g_current.Epoch;
    out->SeqState = g_current.SeqState;
    out->SeqCount = g_current.SeqCount;
    g_current.SeqCount++;  /* wraps at 256 — receivers handle wrap */
}

/* ============================================================
 * Validate — apply the six rules from ADR-0007
 * ============================================================ */
LRMH_TokenValidationResult_t LRMH_SeqToken_Validate(
    const LRMH_SeqToken_t *incoming,
    LRMH_SeqToken_t *accepted,
    bool unconditional)
{
    /* Rule 1: the receiver's own safing commands always pass */
    if (unconditional) {
        return LRMH_TOKEN_UNCONDITIONAL_ACCEPT;
    }

    if (!incoming || !accepted) {
        return LRMH_TOKEN_REJECT_REORDER;
    }

    /* Rule 3: stale epoch */
    if (incoming->Epoch < accepted->Epoch) {
        return LRMH_TOKEN_REJECT_STALE_EPOCH;
    }

    /* Different epoch (newer) — accept and re-baseline */
    if (incoming->Epoch > accepted->Epoch) {
        *accepted = *incoming;
        return LRMH_TOKEN_ACCEPT;
    }

    /* Same epoch */

    /* Rule 4: stale state (same epoch, earlier phase) */
    if (incoming->SeqState < accepted->SeqState) {
        return LRMH_TOKEN_REJECT_STALE_STATE;
    }

    /* Different state (newer) — accept and re-baseline within epoch */
    if (incoming->SeqState > accepted->SeqState) {
        *accepted = *incoming;
        return LRMH_TOKEN_ACCEPT;
    }

    /* Same epoch, same state */

    /* Rule 5: duplicate / reorder
     * Use modular arithmetic for the 8-bit count to handle wrap. */
    if (!LRMH_SeqToken_IsNewer(incoming->SeqCount, accepted->SeqCount)) {
        return LRMH_TOKEN_REJECT_REORDER;
    }

    /* Rule 6: accept */
    *accepted = *incoming;
    return LRMH_TOKEN_ACCEPT;
}

/* ============================================================
 * Library introspection (for HK telemetry)
 * ============================================================ */
uint16 LRMH_SeqToken_GetCurrentEpoch(void)
{
    return g_current.Epoch;
}
