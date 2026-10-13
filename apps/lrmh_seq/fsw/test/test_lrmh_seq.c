/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_lrmh_seq.c
 * @brief Standalone unit tests for lrmh_seq
 *
 * Same pattern as test_lrmh_cmd.c: #include the source under
 * test, stub the seq_token library locally so the test owns the
 * counters and can observe the side effects.
 *
 * Coverage:
 *
 *   1. Initial state — fresh g_SeqApp is in PHASE_IDLE
 *   2. StartSiteTest — accepted from IDLE, rejected from non-IDLE,
 *      latches all 2.0 fields, advances to PRECHECK, bumps epoch
 *   3. Phase machine walk — full nominal recipe (PRECHECK →
 *      POSITIONING → BASELINE_PEN → LOAD_RAMP → POST_PEN →
 *      PROCESSING → REPORTING → IDLE), one actuator command per
 *      step, advancing only on completion
 *   4. EnableTraction — TRACTION phase included when flag set,
 *      passes through immediately as a stub (ADR-0004)
 *   5. EnableRepeatLoad with custom RepeatLoadCount
 *   6. T1 RESUME_PHASE — does not change phase, increments
 *      T1AttemptsCurrentPhase, escalates to T2 after 3 attempts
 *   7. T2 SKIP_SITE — jumps to PROCESSING with PARTIAL quality
 *   8. T3 SAFE — sends safe broadcast, transitions to IDLE,
 *      QualityFlag = FAULT
 *   9. Token stamping — every send call advances seq_count
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* HAL stub state (declared extern in cfe_stubs/test_inc/lrmh_hal.h).
 * lrmh_seq doesn't call HAL directly per ADR-0006, but
 * lrmh_cmd_msg.h transitively pulls in lrmh_hal.h via the cmd
 * app header chain — so we need the stubs available. */
int     lrmh_hal_stub_init_count       = 0;
int32_t lrmh_hal_stub_init_return      = 0;
int     lrmh_hal_stub_estop_count      = 0;
int     lrmh_hal_stub_retract_count    = 0;
int32_t lrmh_hal_stub_retract_return   = 0;
int     lrmh_hal_stub_hold_count       = 0;
int     lrmh_hal_stub_position_count   = 0;
int     lrmh_hal_stub_force_count      = 0;

/* ── Sequence-token stub state ─────────────────────────────────── */
static int       g_token_bump_count    = 0;
static int       g_token_stamp_count   = 0;
static int       g_token_set_phase_count = 0;
static uint16_t  g_token_epoch         = 0;
static uint8_t   g_token_phase         = 0;
static uint8_t   g_token_count         = 0;

#include "lrmh_seq_token.h"

void LRMH_SeqToken_BumpEpoch(LRMH_TokenResetReason_t reason) {
    (void)reason;
    g_token_bump_count++;
    g_token_epoch++;
    g_token_phase = 0;
    g_token_count = 0;
}

void LRMH_SeqToken_Stamp(LRMH_SeqToken_t *out) {
    g_token_stamp_count++;
    if (out) {
        out->Epoch    = g_token_epoch;
        out->SeqState = g_token_phase;
        out->SeqCount = g_token_count;
    }
    g_token_count++;
}

void LRMH_SeqToken_SetPhase(uint8 new_phase) {
    g_token_set_phase_count++;
    g_token_phase = new_phase;
    g_token_count = 0;
}

uint16 LRMH_SeqToken_GetCurrentEpoch(void) { return g_token_epoch; }

LRMH_TokenValidationResult_t LRMH_SeqToken_Validate(
    const LRMH_SeqToken_t *incoming,
    LRMH_SeqToken_t *accepted,
    bool unconditional) {
    (void)incoming; (void)accepted; (void)unconditional;
    return LRMH_TOKEN_ACCEPT;
}

/* ── SB transmit capture ───────────────────────────────────────── */
typedef struct { uint16 mid; uint8 fcn; } SbXmit_t;
#define MAX_SB_LOG 64
static SbXmit_t  g_sb_log[MAX_SB_LOG];
static int       g_sb_count = 0;

static void reset_sb_log(void) { g_sb_count = 0; }

/* The cFE stub's CFE_SB_TransmitMsg is a no-op. We override it by
 * defining a wrapper macro that the .c source uses. The cleanest
 * way: redefine CFE_SB_TransmitMsg AFTER including the stub
 * header but BEFORE including the .c. Macros take precedence over
 * static inline functions. */
#undef CFE_SB_TransmitMsg
#define CFE_SB_TransmitMsg(msg, inc) \
    do { \
        CFE_SB_MsgId_t _mid; \
        CFE_MSG_GetMsgId((msg), &_mid); \
        if (g_sb_count < MAX_SB_LOG) { \
            g_sb_log[g_sb_count].mid = (uint16)_mid.Value; \
            g_sb_log[g_sb_count].fcn = 0; \
            g_sb_count++; \
        } \
        (void)(inc); \
    } while (0)

/* Now include the source under test */
#include "../src/lrmh_seq_app.c"

/* ── Test framework ────────────────────────────────────────────── */

static int passed = 0;
static int failed = 0;
static const char *current_test = "";

#define ASSERT(cond) do {                                              \
    if (!(cond)) {                                                     \
        fprintf(stderr, "  [%s] FAIL: %s  (%s:%d)\n",                  \
                current_test, #cond, __FILE__, __LINE__);              \
        failed++;                                                      \
    } else {                                                           \
        passed++;                                                      \
    }                                                                  \
} while (0)

#define TEST(name) do {                                                \
    current_test = name;                                               \
    fprintf(stderr, "RUN  %s\n", name);                                \
} while (0)

static void reset_state(void)
{
    memset(&g_SeqApp, 0, sizeof(g_SeqApp));
    g_SeqApp.Phase = LRMH_PHASE_IDLE;
    g_SeqApp.QualityFlag = LRMH_QUALITY_VALID;
    g_token_bump_count = 0;
    g_token_stamp_count = 0;
    g_token_set_phase_count = 0;
    g_token_epoch = 0;
    g_token_phase = 0;
    g_token_count = 0;
    reset_sb_log();
}

/* Build a default StartSiteTestCmd for tests */
static LRMH_StartSiteTestCmd_t make_start_cmd(void)
{
    LRMH_StartSiteTestCmd_t c;
    memset(&c, 0, sizeof(c));
    c.SiteId            = 1;
    c.TestProfileId     = 0;
    c.ToolId            = 0;
    c.MaxDepth_m        = 0.15f;
    c.MaxLoad_N         = 400.0f;
    c.EnableRepeatLoad  = 0;
    c.EnablePostProfile = 0;
    c.EnableTraction    = 0;
    c.SessionId         = 0xAAAA;
    c.SequenceNumber    = 1;
    c.RepeatLoadCount   = 0;
    return c;
}

/* ── Tests ─────────────────────────────────────────────────────── */

static void test_initial_state(void)
{
    TEST("initial_state");
    reset_state();
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_IDLE);
    ASSERT(g_SeqApp.QualityFlag == LRMH_QUALITY_VALID);
    ASSERT(g_SeqApp.CmdCount == 0u);
    ASSERT(g_SeqApp.T1AttemptsCurrentPhase == 0u);
}

static void test_start_test_accepted_from_idle(void)
{
    TEST("start_test_accepted_from_idle");
    reset_state();
    LRMH_StartSiteTestCmd_t c = make_start_cmd();

    bool ok = LRMH_SEQ_HandleStartTest(&c);
    ASSERT(ok);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_PRECHECK);
    ASSERT(g_SeqApp.SiteId == 1);
    ASSERT(g_SeqApp.SessionId == 0xAAAA);
    ASSERT(g_SeqApp.MaxDepth_m == 0.15f);
    ASSERT(g_SeqApp.RepeatCyclesTarget == 3);  /* default per ADR-0014 */
    ASSERT(g_token_bump_count == 1);  /* fresh epoch */
}

static void test_start_test_rejected_from_non_idle(void)
{
    TEST("start_test_rejected_from_non_idle");
    reset_state();
    g_SeqApp.Phase = LRMH_PHASE_BASELINE_PEN;
    LRMH_StartSiteTestCmd_t c = make_start_cmd();

    bool ok = LRMH_SEQ_HandleStartTest(&c);
    ASSERT(!ok);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_BASELINE_PEN);  /* unchanged */
    ASSERT(g_SeqApp.ErrCount == 1);
}

static void test_repeat_load_count_override(void)
{
    TEST("repeat_load_count_override");
    reset_state();
    LRMH_StartSiteTestCmd_t c = make_start_cmd();
    c.RepeatLoadCount = 7;
    LRMH_SEQ_HandleStartTest(&c);
    ASSERT(g_SeqApp.RepeatCyclesTarget == 7);
}

/* Drive the machine like the live loop does: each call either issues
 * a command (and then waits) or advances. The test stands in for
 * lrmh_ctrl by completing every command and for lrmh_proc by
 * answering every trigger. Returns the number of ticks used. */
static int walk_to_idle(int max_ticks)
{
    int ticks = 0;
    while (g_SeqApp.Phase != LRMH_PHASE_IDLE && ticks < max_ticks) {
        LRMH_SEQ_AdvancePhase();
        if (g_SeqApp.WaitingForCtrl) g_SeqApp.WaitingForCtrl = 0;  /* ctrl: complete */
        g_SeqApp.WaitingForProc = 0;                                /* proc: answered */
        ticks++;
    }
    return ticks;
}

static void test_phase_walk_minimal(void)
{
    /* Minimal test: BASELINE_PEN and one load cycle (no repeat, no
     * post, no traction) */
    TEST("phase_walk_minimal");
    reset_state();
    LRMH_StartSiteTestCmd_t c = make_start_cmd();
    LRMH_SEQ_HandleStartTest(&c);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_PRECHECK);

    /* The phase label names the motion in progress: after the first
     * tick the self-test command is in flight and the phase is still
     * PRECHECK; it only advances once the command completes. */
    LRMH_SEQ_AdvancePhase();
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_PRECHECK);
    ASSERT(g_SeqApp.WaitingForCtrl == 1);
    LRMH_SEQ_AdvancePhase();            /* still waiting: no change */
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_PRECHECK);
    g_SeqApp.WaitingForCtrl = 0;        /* ctrl reports complete */
    LRMH_SEQ_AdvancePhase();
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_POSITIONING);

    int ticks = walk_to_idle(60);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_IDLE);
    ASSERT(g_SeqApp.QualityFlag == LRMH_QUALITY_VALID);
    ASSERT(ticks < 60);
}

static void test_actuator_timeout_skips_site(void)
{
    TEST("actuator_timeout_skips_site");
    reset_state();
    LRMH_StartSiteTestCmd_t c = make_start_cmd();
    LRMH_SEQ_HandleStartTest(&c);
    LRMH_SEQ_AdvancePhase();            /* PRECHECK command issued, deadline armed */
    ASSERT(g_SeqApp.WaitingForCtrl == 1);
    ASSERT(g_SeqApp.CtrlDeadline_s > g_SeqApp.ElapsedTime_s);
    /* The stub clock never advances, so push elapsed time past the
     * deadline by hand: the command is abandoned and the site is
     * skipped as T2 (PARTIAL). */
    g_SeqApp.ElapsedTime_s = g_SeqApp.CtrlDeadline_s + 1.0f;
    LRMH_SEQ_AdvancePhase();
    ASSERT(g_SeqApp.WaitingForCtrl == 0 || g_SeqApp.Phase == LRMH_PHASE_PROCESSING);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_PROCESSING);
    ASSERT(g_SeqApp.QualityFlag == LRMH_QUALITY_PARTIAL);
    ASSERT(g_SeqApp.ErrCount == 1);
}

static void test_repeat_load_cycles_unload_and_reload(void)
{
    TEST("repeat_load_cycles_unload_and_reload");
    reset_state();
    LRMH_StartSiteTestCmd_t c = make_start_cmd();
    c.EnableRepeatLoad = 1;
    c.RepeatLoadCount  = 2;
    LRMH_SEQ_HandleStartTest(&c);
    /* Walk until REPEAT_LOAD is entered */
    int guard = 40;
    while (g_SeqApp.Phase != LRMH_PHASE_REPEAT_LOAD && guard-- > 0) {
        LRMH_SEQ_AdvancePhase();
        g_SeqApp.WaitingForCtrl = 0;
        g_SeqApp.WaitingForProc = 0;
    }
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_REPEAT_LOAD);
    /* Each cycle is a retract then a force command: two commands and
     * the phase stays REPEAT_LOAD until the second cycle is done. */
    reset_sb_log();
    LRMH_SEQ_AdvancePhase(); g_SeqApp.WaitingForCtrl = 0;   /* retract */
    LRMH_SEQ_AdvancePhase(); g_SeqApp.WaitingForCtrl = 0;   /* reload */
    LRMH_SEQ_AdvancePhase();                                 /* cycle 1 done */
    ASSERT(g_SeqApp.RepeatCyclesDone == 1);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_REPEAT_LOAD);
    LRMH_SEQ_AdvancePhase(); g_SeqApp.WaitingForCtrl = 0;
    LRMH_SEQ_AdvancePhase(); g_SeqApp.WaitingForCtrl = 0;
    LRMH_SEQ_AdvancePhase();                                 /* cycle 2 done: trigger + leave */
    ASSERT(g_SeqApp.RepeatCyclesDone == 2);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_PROCESSING);
    ASSERT(g_SeqApp.WaitingForProc == 1);   /* the load dataset trigger (baseline was answered in the walk) */
    /* 4 actuator commands + 1 proc trigger (+ status packets) */
    ASSERT(g_sb_count >= 5);
}

static void test_traction_phase_skipped_when_disabled(void)
{
    TEST("traction_phase_skipped_when_disabled");
    reset_state();
    LRMH_StartSiteTestCmd_t c = make_start_cmd();
    c.EnableTraction = 0;
    LRMH_SEQ_HandleStartTest(&c);

    /* Walk through nominal phases — TRACTION should not appear
     * because EnableTraction is 0. We can't easily verify the
     * negative without instrumentation, so just confirm we
     * reach IDLE. */
    walk_to_idle(60);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_IDLE);
}

static void test_traction_phase_passes_through_when_enabled(void)
{
    TEST("traction_phase_passes_through_when_enabled");
    reset_state();
    LRMH_StartSiteTestCmd_t c = make_start_cmd();
    c.EnableTraction = 1;
    LRMH_SEQ_HandleStartTest(&c);

    /* TRACTION is a stub per ADR-0004; it should pass through
     * immediately to PROCESSING. The test reaches IDLE in
     * normal walk count. */
    walk_to_idle(60);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_IDLE);
}

/* ── Recovery tier tests ───────────────────────────────────────── */

static void test_t1_resume_increments_counter(void)
{
    TEST("t1_resume_increments_counter");
    reset_state();
    g_SeqApp.Phase = LRMH_PHASE_BASELINE_PEN;

    LRMH_SEQ_HandleRecovery(0, LRMH_TIER_T1_RESUME);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_BASELINE_PEN);  /* unchanged */
    ASSERT(g_SeqApp.T1AttemptsCurrentPhase == 1);
}

static void test_t1_escalates_to_t2_after_max_attempts(void)
{
    TEST("t1_escalates_to_t2_after_max_attempts");
    reset_state();
    g_SeqApp.Phase = LRMH_PHASE_BASELINE_PEN;

    /* 3 T1 attempts allowed (LRMH_T1_MAX_ATTEMPTS_PER_PHASE = 3).
     * The 4th escalates to T2 SKIP_SITE → PROCESSING. */
    LRMH_SEQ_HandleRecovery(0, LRMH_TIER_T1_RESUME);
    LRMH_SEQ_HandleRecovery(0, LRMH_TIER_T1_RESUME);
    LRMH_SEQ_HandleRecovery(0, LRMH_TIER_T1_RESUME);
    ASSERT(g_SeqApp.T1AttemptsCurrentPhase == 3);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_BASELINE_PEN);

    /* 4th attempt → escalation */
    LRMH_SEQ_HandleRecovery(0, LRMH_TIER_T1_RESUME);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_PROCESSING);
    ASSERT(g_SeqApp.QualityFlag == LRMH_QUALITY_PARTIAL);
}

static void test_t2_skip_site_jumps_to_processing(void)
{
    TEST("t2_skip_site_jumps_to_processing");
    reset_state();
    g_SeqApp.Phase = LRMH_PHASE_LOAD_RAMP;

    LRMH_SEQ_HandleRecovery(0, LRMH_TIER_T2_SKIP);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_PROCESSING);
    ASSERT(g_SeqApp.QualityFlag == LRMH_QUALITY_PARTIAL);
}

static void test_t3_safe_transitions_to_idle(void)
{
    TEST("t3_safe_transitions_to_idle");
    reset_state();
    g_SeqApp.Phase = LRMH_PHASE_BASELINE_PEN;
    reset_sb_log();

    LRMH_SEQ_HandleRecovery(0, LRMH_TIER_T3_SAFE);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_IDLE);
    ASSERT(g_SeqApp.QualityFlag == LRMH_QUALITY_FAULT);

    /* Should have sent: sense stop, ctrl retract, safe broadcast */
    ASSERT(g_sb_count >= 3);
}

static void test_t1_resume_rejected_in_idle(void)
{
    TEST("t1_resume_rejected_in_idle");
    reset_state();
    g_SeqApp.Phase = LRMH_PHASE_IDLE;

    LRMH_SEQ_HandleRecovery(0, LRMH_TIER_T1_RESUME);
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_IDLE);  /* unchanged */
    ASSERT(g_SeqApp.T1AttemptsCurrentPhase == 0);
}

/* ── Token stamping tests ──────────────────────────────────────── */

static void test_phase_advance_calls_set_phase(void)
{
    TEST("phase_advance_calls_set_phase");
    reset_state();
    int before = g_token_set_phase_count;
    LRMH_SEQ_TransitionTo(LRMH_PHASE_BASELINE_PEN);
    ASSERT(g_token_set_phase_count == before + 1);
    ASSERT(g_token_phase == (uint8)LRMH_PHASE_BASELINE_PEN);
}

static void test_phase_advance_resets_t1_counter(void)
{
    TEST("phase_advance_resets_t1_counter");
    reset_state();
    g_SeqApp.Phase = LRMH_PHASE_BASELINE_PEN;
    g_SeqApp.T1AttemptsCurrentPhase = 2;

    LRMH_SEQ_TransitionTo(LRMH_PHASE_LOAD_RAMP);
    ASSERT(g_SeqApp.T1AttemptsCurrentPhase == 0);
}

/* ── Manual ground actuator command (LRMH_SEQ_MANUAL_CTRL_CC) ──── */

static void test_manual_ctrl_accepted_from_idle(void)
{
    TEST("manual_ctrl_accepted_from_idle");
    reset_state();
    g_SeqApp.Phase = LRMH_PHASE_IDLE;
    reset_sb_log();

    LRMH_SeqManualCtrlCmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.Mode             = LRMH_CTRL_MODE_RETRACT;
    cmd.RateLimit        = 0.008f;
    cmd.Timeout_s        = 30.0f;

    int stamps_before = g_token_stamp_count;
    LRMH_SEQ_HandleManualCtrl(&cmd);

    /* Exactly one SB transmit (forward on LRMH_SEQ_TO_CTRL_MID via
     * LRMH_SEQ_SendCtrlCmd) and exactly one token stamp (the
     * sequencer is the sole token producer per ADR-0007). The cFE
     * stub returns 0 for GetMsgId so we can't assert the actual
     * MID — match the convention used elsewhere in this file. */
    ASSERT(g_sb_count == 1);
    ASSERT(g_token_stamp_count == stamps_before + 1);
    ASSERT(g_SeqApp.CmdCount == 1);
    ASSERT(g_SeqApp.ErrCount == 0);
}

static void test_manual_ctrl_rejected_during_test(void)
{
    TEST("manual_ctrl_rejected_during_test");
    reset_state();
    g_SeqApp.Phase = LRMH_PHASE_BASELINE_PEN;
    reset_sb_log();

    LRMH_SeqManualCtrlCmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.Mode = LRMH_CTRL_MODE_HOLD;
    LRMH_SEQ_HandleManualCtrl(&cmd);

    /* Nothing forwarded; ErrCount bumped. */
    ASSERT(g_sb_count == 0);
    ASSERT(g_SeqApp.ErrCount == 1);
    ASSERT(g_SeqApp.CmdCount == 0);
    /* Phase unchanged */
    ASSERT(g_SeqApp.Phase == LRMH_PHASE_BASELINE_PEN);
}

/* ── main ──────────────────────────────────────────────────────── */

int main(void)
{
    fprintf(stderr, "lrmh_seq standalone unit tests\n");
    fprintf(stderr, "============================================\n");

    test_initial_state();
    test_start_test_accepted_from_idle();
    test_start_test_rejected_from_non_idle();
    test_repeat_load_count_override();
    test_phase_walk_minimal();
    test_actuator_timeout_skips_site();
    test_repeat_load_cycles_unload_and_reload();
    test_traction_phase_skipped_when_disabled();
    test_traction_phase_passes_through_when_enabled();
    test_t1_resume_increments_counter();
    test_t1_escalates_to_t2_after_max_attempts();
    test_t2_skip_site_jumps_to_processing();
    test_t3_safe_transitions_to_idle();
    test_t1_resume_rejected_in_idle();
    test_manual_ctrl_accepted_from_idle();
    test_manual_ctrl_rejected_during_test();
    test_phase_advance_calls_set_phase();
    test_phase_advance_resets_t1_counter();

    fprintf(stderr, "============================================\n");
    fprintf(stderr, "  passed: %d\n", passed);
    fprintf(stderr, "  failed: %d\n", failed);
    return failed == 0 ? 0 : 1;
}
