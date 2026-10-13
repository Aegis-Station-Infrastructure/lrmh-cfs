/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_lrmh_ctrl.c
 * @brief Standalone unit tests for lrmh_ctrl
 *
 * lrmh_ctrl is the first 2.0 receiver of token-validated commands.
 * Coverage focus:
 *
 *   1. Initial state — fresh g_CtrlApp, accepted token zeroed
 *   2. Mode → HAL mapping — every mode dispatches to the right
 *      LRMH_HAL_* function with the right argument
 *   3. Token validation — ACCEPT path, stale-epoch reject,
 *      stale-state reject, reorder reject
 *   4. Token reset — receiver clears state and accepts the next
 *      command unconditionally per ADR-0007 rule 2
 *   5. Command completion — HAL state IDLE/HOLDING clears
 *      CommandPending and sets CommandComplete flag
 *   6. Status TLM — published with correct fields
 *   7. Ground/fault commands bypass token validation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Pull in the stub HAL header first so the types are defined
 * before we provide the extern variable storage. */
#include "lrmh_hal.h"

/* HAL stub state — storage for the extern declarations in
 * cfe_stubs/test_inc/lrmh_hal.h */
int32_t lrmh_hal_stub_init_count       = 0;
int32_t lrmh_hal_stub_init_return      = 0;
int32_t lrmh_hal_stub_estop_count      = 0;
int32_t lrmh_hal_stub_retract_count    = 0;
int32_t lrmh_hal_stub_retract_return   = 0;
int32_t lrmh_hal_stub_hold_count       = 0;
int32_t lrmh_hal_stub_position_count   = 0;
int32_t lrmh_hal_stub_force_count      = 0;
LRMH_HAL_PositionCmd_t lrmh_hal_stub_last_position;
LRMH_HAL_ForceCmd_t    lrmh_hal_stub_last_force;
LRMH_HAL_SensorData_t  lrmh_hal_stub_inject_sensor;
LRMH_HAL_Health_t      lrmh_hal_stub_inject_health;

/* Sequence-token stub (real validate logic, not test counters,
 * because lrmh_ctrl actually exercises the validation rules) */
#include "lrmh_seq_token.h"

/* We want the REAL LRMH_SeqToken_Validate so the test exercises
 * ADR-0007's six rules. Compile in msg/lrmh_seq_token.c via
 * #include — the .c is small (~135 lines) and self-contained. */
#include "../../../../msg/lrmh_seq_token.c"

/* SB transmit capture */
typedef struct { uint16 mid; } SbXmit_t;
#define MAX_SB_LOG 32
static SbXmit_t  g_sb_log[MAX_SB_LOG];
static int       g_sb_count = 0;

#undef CFE_SB_TransmitMsg
#define CFE_SB_TransmitMsg(msg, inc) \
    do { \
        CFE_SB_MsgId_t _mid; \
        CFE_MSG_GetMsgId((msg), &_mid); \
        if (g_sb_count < MAX_SB_LOG) { \
            g_sb_log[g_sb_count].mid = (uint16)_mid.Value; \
            g_sb_count++; \
        } \
        (void)(inc); \
    } while (0)

/* Bring the source under test */
#include "../src/lrmh_ctrl_app.c"

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
    memset(&g_CtrlApp, 0, sizeof(g_CtrlApp));
    lrmh_hal_stub_init_count     = 0;
    lrmh_hal_stub_init_return    = LRMH_HAL_OK;
    lrmh_hal_stub_estop_count    = 0;
    lrmh_hal_stub_retract_count  = 0;
    lrmh_hal_stub_retract_return = LRMH_HAL_OK;
    lrmh_hal_stub_hold_count     = 0;
    lrmh_hal_stub_position_count = 0;
    lrmh_hal_stub_force_count    = 0;
    memset(&lrmh_hal_stub_last_position, 0, sizeof(lrmh_hal_stub_last_position));
    memset(&lrmh_hal_stub_last_force,    0, sizeof(lrmh_hal_stub_last_force));
    memset(&lrmh_hal_stub_inject_sensor, 0, sizeof(lrmh_hal_stub_inject_sensor));
    memset(&lrmh_hal_stub_inject_health, 0, sizeof(lrmh_hal_stub_inject_health));
    lrmh_hal_stub_inject_health.valid = true;
    lrmh_hal_stub_inject_health.state = LRMH_HAL_STATE_IDLE;
    g_sb_count = 0;
}

static LRMH_CtrlCmd_t make_cmd(uint8 mode, uint16 epoch, uint8 state, uint8 count)
{
    LRMH_CtrlCmd_t c;
    memset(&c, 0, sizeof(c));
    c.Mode = mode;
    c.TargetPosition_m = 0.15f;
    c.TargetForce_N    = 400.0f;
    c.RateLimit        = 0.005f;
    c.Timeout_s        = 30.0f;
    LRMH_SeqToken_t tok = { .Epoch = epoch, .SeqState = state, .SeqCount = count };
    c.SeqTokenPacked = LRMH_SEQ_TOKEN_PACK(tok);
    return c;
}

/* ── Tests ─────────────────────────────────────────────────────── */

static void test_initial_state(void)
{
    TEST("initial_state");
    reset_state();
    ASSERT(g_CtrlApp.ActiveMode == 0);
    ASSERT(g_CtrlApp.CommandPending == 0);
    ASSERT(g_CtrlApp.CmdCount == 0u);
    ASSERT(g_CtrlApp.StaleCount == 0u);
    ASSERT(g_CtrlApp.AcceptedToken.Epoch == 0);
}

/* ── Mode → HAL mapping ────────────────────────────────────────── */

static void test_position_mode_calls_hal_position(void)
{
    TEST("position_mode_calls_hal_position");
    reset_state();
    g_CtrlApp.ResetObserved = true;  /* accept any token unconditionally */
    LRMH_CtrlCmd_t c = make_cmd(LRMH_CTRL_MODE_POSITION, 1, 1, 0);
    LRMH_CTRL_ValidateAndDispatch(&c);
    ASSERT(lrmh_hal_stub_position_count == 1);
    ASSERT(lrmh_hal_stub_last_position.target_stroke_m == 0.15f);
    ASSERT(lrmh_hal_stub_last_position.max_force_n == 400.0f);   /* the command's TargetForce_N is the clamp */
    ASSERT(g_CtrlApp.LastCommandedPos_m == 0.15f);
    ASSERT(g_CtrlApp.CmdCount == 1);
    ASSERT(g_CtrlApp.CommandPending == 1);
}

static void test_force_mode_calls_hal_force(void)
{
    TEST("force_mode_calls_hal_force");
    reset_state();
    g_CtrlApp.ResetObserved = true;
    LRMH_CtrlCmd_t c = make_cmd(LRMH_CTRL_MODE_FORCE, 1, 1, 0);
    LRMH_CTRL_ValidateAndDispatch(&c);
    ASSERT(lrmh_hal_stub_force_count == 1);
    ASSERT(lrmh_hal_stub_last_force.target_force_n == 400.0f);
    ASSERT(lrmh_hal_stub_last_force.max_stroke_m == LRMH_LC_STROKE_THRESHOLD_M - 0.01f);   /* inside the LC watchpoint, ADR-0012 */
    ASSERT(g_CtrlApp.LastCommandedForce_N == 400.0f);
}

static void test_retract_mode_calls_hal_retract(void)
{
    TEST("retract_mode_calls_hal_retract");
    reset_state();
    g_CtrlApp.ResetObserved = true;
    LRMH_CtrlCmd_t c = make_cmd(LRMH_CTRL_MODE_RETRACT, 1, 1, 0);
    LRMH_CTRL_ValidateAndDispatch(&c);
    ASSERT(lrmh_hal_stub_retract_count == 1);
}

static void test_hold_mode_calls_hal_hold(void)
{
    TEST("hold_mode_calls_hal_hold");
    reset_state();
    g_CtrlApp.ResetObserved = true;
    LRMH_CtrlCmd_t c = make_cmd(LRMH_CTRL_MODE_HOLD, 1, 1, 0);
    LRMH_CTRL_ValidateAndDispatch(&c);
    ASSERT(lrmh_hal_stub_hold_count == 1);
}

static void test_estop_mode_calls_hal_estop(void)
{
    TEST("estop_mode_calls_hal_estop");
    reset_state();
    g_CtrlApp.ResetObserved = true;
    LRMH_CtrlCmd_t c = make_cmd(LRMH_CTRL_MODE_ESTOP, 1, 1, 0);
    LRMH_CTRL_ValidateAndDispatch(&c);
    ASSERT(lrmh_hal_stub_estop_count == 1);
}

static void test_precheck_mode_calls_hal_init(void)
{
    TEST("precheck_mode_calls_hal_init");
    reset_state();
    g_CtrlApp.ResetObserved = true;
    LRMH_CtrlCmd_t c = make_cmd(LRMH_CTRL_MODE_PRECHECK, 1, 1, 0);
    LRMH_CTRL_ValidateAndDispatch(&c);
    ASSERT(lrmh_hal_stub_init_count == 1);
}

static void test_bad_mode_increments_err(void)
{
    TEST("bad_mode_increments_err");
    reset_state();
    g_CtrlApp.ResetObserved = true;
    LRMH_CtrlCmd_t c = make_cmd(99, 1, 1, 0);
    LRMH_CTRL_ValidateAndDispatch(&c);
    ASSERT(g_CtrlApp.ErrCount == 1);
    ASSERT(g_CtrlApp.CmdCount == 0);
}

/* ── Token validation (the headline coverage) ─────────────────── */

static void test_first_command_after_reset_accepted(void)
{
    TEST("first_command_after_reset_accepted");
    reset_state();
    LRMH_CTRL_OnTokenReset();  /* sets ResetObserved=true */
    LRMH_CtrlCmd_t c = make_cmd(LRMH_CTRL_MODE_POSITION, 5, 3, 0);
    LRMH_TokenValidationResult_t r = LRMH_CTRL_ValidateAndDispatch(&c);
    ASSERT(r == LRMH_TOKEN_ACCEPT);
    ASSERT(g_CtrlApp.CmdCount == 1);
    ASSERT(g_CtrlApp.AcceptedToken.Epoch == 5);
    ASSERT(g_CtrlApp.AcceptedToken.SeqState == 3);
    ASSERT(g_CtrlApp.AcceptedToken.SeqCount == 0);
    ASSERT(!g_CtrlApp.ResetObserved);  /* cleared */
}

static void test_newer_in_same_state_accepted(void)
{
    TEST("newer_in_same_state_accepted");
    reset_state();
    LRMH_CTRL_OnTokenReset();
    /* First baseline */
    LRMH_CtrlCmd_t c1 = make_cmd(LRMH_CTRL_MODE_POSITION, 5, 3, 0);
    LRMH_CTRL_ValidateAndDispatch(&c1);
    /* Newer count */
    LRMH_CtrlCmd_t c2 = make_cmd(LRMH_CTRL_MODE_POSITION, 5, 3, 1);
    LRMH_TokenValidationResult_t r = LRMH_CTRL_ValidateAndDispatch(&c2);
    ASSERT(r == LRMH_TOKEN_ACCEPT);
    ASSERT(g_CtrlApp.AcceptedToken.SeqCount == 1);
    ASSERT(g_CtrlApp.CmdCount == 2);
}

static void test_stale_epoch_rejected(void)
{
    TEST("stale_epoch_rejected");
    reset_state();
    LRMH_CTRL_OnTokenReset();
    LRMH_CtrlCmd_t c1 = make_cmd(LRMH_CTRL_MODE_POSITION, 5, 3, 0);
    LRMH_CTRL_ValidateAndDispatch(&c1);
    /* Stale epoch */
    LRMH_CtrlCmd_t c2 = make_cmd(LRMH_CTRL_MODE_POSITION, 4, 5, 0);
    LRMH_TokenValidationResult_t r = LRMH_CTRL_ValidateAndDispatch(&c2);
    ASSERT(r == LRMH_TOKEN_REJECT_STALE_EPOCH);
    ASSERT(g_CtrlApp.StaleCount == 1);
    ASSERT(g_CtrlApp.CmdCount == 1);  /* unchanged */
}

static void test_stale_state_rejected(void)
{
    TEST("stale_state_rejected");
    reset_state();
    LRMH_CTRL_OnTokenReset();
    LRMH_CtrlCmd_t c1 = make_cmd(LRMH_CTRL_MODE_POSITION, 5, 3, 0);
    LRMH_CTRL_ValidateAndDispatch(&c1);
    /* Same epoch, earlier state */
    LRMH_CtrlCmd_t c2 = make_cmd(LRMH_CTRL_MODE_POSITION, 5, 2, 5);
    LRMH_TokenValidationResult_t r = LRMH_CTRL_ValidateAndDispatch(&c2);
    ASSERT(r == LRMH_TOKEN_REJECT_STALE_STATE);
    ASSERT(g_CtrlApp.StaleCount == 1);
}

static void test_reorder_rejected(void)
{
    TEST("reorder_rejected");
    reset_state();
    LRMH_CTRL_OnTokenReset();
    LRMH_CtrlCmd_t c1 = make_cmd(LRMH_CTRL_MODE_POSITION, 5, 3, 5);
    LRMH_CTRL_ValidateAndDispatch(&c1);
    /* Same epoch, same state, equal count → reorder */
    LRMH_CtrlCmd_t c2 = make_cmd(LRMH_CTRL_MODE_POSITION, 5, 3, 5);
    LRMH_TokenValidationResult_t r = LRMH_CTRL_ValidateAndDispatch(&c2);
    ASSERT(r == LRMH_TOKEN_REJECT_REORDER);
    /* Same epoch, same state, lower count → reorder */
    LRMH_CtrlCmd_t c3 = make_cmd(LRMH_CTRL_MODE_POSITION, 5, 3, 4);
    r = LRMH_CTRL_ValidateAndDispatch(&c3);
    ASSERT(r == LRMH_TOKEN_REJECT_REORDER);
    ASSERT(g_CtrlApp.StaleCount == 2);
}

static void test_safing_command_unconditional_accept(void)
{
    TEST("safing_command_unconditional_accept");
    reset_state();
    LRMH_CTRL_OnTokenReset();
    LRMH_CtrlCmd_t c1 = make_cmd(LRMH_CTRL_MODE_POSITION, 5, 3, 0);
    LRMH_CTRL_ValidateAndDispatch(&c1);
    /* Now send a stale-epoch RETRACT — should still be accepted
     * via ADR-0007 rule 1 because RETRACT is in the safing CC
     * set. */
    LRMH_CtrlCmd_t c2 = make_cmd(LRMH_CTRL_MODE_RETRACT, 0, 0, 0);
    LRMH_TokenValidationResult_t r = LRMH_CTRL_ValidateAndDispatch(&c2);
    ASSERT(r == LRMH_TOKEN_UNCONDITIONAL_ACCEPT);
    ASSERT(lrmh_hal_stub_retract_count == 1);
    ASSERT(g_CtrlApp.StaleCount == 0);
}

/* ── Token reset re-baselining ─────────────────────────────────── */

static void test_token_reset_clears_state(void)
{
    TEST("token_reset_clears_state");
    reset_state();
    g_CtrlApp.AcceptedToken.Epoch    = 100;
    g_CtrlApp.AcceptedToken.SeqState = 5;
    g_CtrlApp.AcceptedToken.SeqCount = 50;

    LRMH_CTRL_OnTokenReset();
    ASSERT(g_CtrlApp.AcceptedToken.Epoch == 0);
    ASSERT(g_CtrlApp.AcceptedToken.SeqState == 0);
    ASSERT(g_CtrlApp.AcceptedToken.SeqCount == 0);
    ASSERT(g_CtrlApp.ResetObserved == true);
}

static void test_reset_then_low_epoch_accepted(void)
{
    TEST("reset_then_low_epoch_accepted");
    reset_state();
    /* High accepted epoch from prior session */
    g_CtrlApp.AcceptedToken.Epoch = 100;
    g_CtrlApp.AcceptedToken.SeqState = 5;
    g_CtrlApp.AcceptedToken.SeqCount = 50;

    LRMH_CTRL_OnTokenReset();

    /* New (lower) epoch should be accepted unconditionally per
     * rule 2 because the reset cleared the baseline. */
    LRMH_CtrlCmd_t c = make_cmd(LRMH_CTRL_MODE_POSITION, 1, 1, 0);
    LRMH_TokenValidationResult_t r = LRMH_CTRL_ValidateAndDispatch(&c);
    ASSERT(r == LRMH_TOKEN_ACCEPT);
    ASSERT(g_CtrlApp.AcceptedToken.Epoch == 1);
}

/* ── Status TLM completion detection ──────────────────────────── */

static void test_completion_clears_pending_when_idle(void)
{
    TEST("completion_clears_pending_when_idle");
    reset_state();
    g_CtrlApp.CommandPending = 1;
    lrmh_hal_stub_inject_health.valid = true;
    lrmh_hal_stub_inject_health.state = LRMH_HAL_STATE_IDLE;

    LRMH_CTRL_PublishStatus();
    ASSERT(g_CtrlApp.CommandPending == 0);
    ASSERT(g_sb_count == 1);
}

static void test_completion_clears_pending_when_holding(void)
{
    TEST("completion_clears_pending_when_holding");
    reset_state();
    g_CtrlApp.CommandPending = 1;
    lrmh_hal_stub_inject_health.valid = true;
    lrmh_hal_stub_inject_health.state = LRMH_HAL_STATE_HOLDING;

    LRMH_CTRL_PublishStatus();
    ASSERT(g_CtrlApp.CommandPending == 0);
}

static void test_completion_does_not_fire_when_moving(void)
{
    TEST("completion_does_not_fire_when_moving");
    reset_state();
    g_CtrlApp.CommandPending = 1;
    lrmh_hal_stub_inject_health.valid = true;
    lrmh_hal_stub_inject_health.state = LRMH_HAL_STATE_MOVING;

    LRMH_CTRL_PublishStatus();
    ASSERT(g_CtrlApp.CommandPending == 1);  /* still pending */
}

/* ── Ground/fault commands bypass token validation ────────────── */

static void test_ground_command_bypasses_token(void)
{
    TEST("ground_command_bypasses_token");
    reset_state();
    /* No reset, no baseline — direct call simulates ground path */
    LRMH_CtrlCmd_t c = make_cmd(LRMH_CTRL_MODE_RETRACT, 0, 0, 0);
    LRMH_CTRL_HandleCommand(&c, false);  /* ground path bypasses validate */
    ASSERT(lrmh_hal_stub_retract_count == 1);
    ASSERT(g_CtrlApp.StaleCount == 0);  /* no validation happened */
}

/* ── HAL error propagation ────────────────────────────────────── */

static void test_hal_error_increments_counter(void)
{
    TEST("hal_error_increments_counter");
    reset_state();
    g_CtrlApp.ResetObserved = true;
    lrmh_hal_stub_init_return = LRMH_HAL_SELFTEST_FAIL;

    LRMH_CtrlCmd_t c = make_cmd(LRMH_CTRL_MODE_PRECHECK, 1, 1, 0);
    LRMH_CTRL_ValidateAndDispatch(&c);
    ASSERT(g_CtrlApp.HalErrCount == 1);
    ASSERT(g_CtrlApp.ErrCount == 1);
    ASSERT(g_CtrlApp.CommandPending == 0);  /* never set */
}

/* ── main ──────────────────────────────────────────────────────── */

int main(void)
{
    fprintf(stderr, "lrmh_ctrl standalone unit tests\n");
    fprintf(stderr, "============================================\n");

    test_initial_state();
    test_position_mode_calls_hal_position();
    test_force_mode_calls_hal_force();
    test_retract_mode_calls_hal_retract();
    test_hold_mode_calls_hal_hold();
    test_estop_mode_calls_hal_estop();
    test_precheck_mode_calls_hal_init();
    test_bad_mode_increments_err();

    test_first_command_after_reset_accepted();
    test_newer_in_same_state_accepted();
    test_stale_epoch_rejected();
    test_stale_state_rejected();
    test_reorder_rejected();
    test_safing_command_unconditional_accept();

    test_token_reset_clears_state();
    test_reset_then_low_epoch_accepted();

    test_completion_clears_pending_when_idle();
    test_completion_clears_pending_when_holding();
    test_completion_does_not_fire_when_moving();

    test_ground_command_bypasses_token();
    test_hal_error_increments_counter();

    fprintf(stderr, "============================================\n");
    fprintf(stderr, "  passed: %d\n", passed);
    fprintf(stderr, "  failed: %d\n", failed);
    return failed == 0 ? 0 : 1;
}
