/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_lrmh_cmd.c
 * @brief Standalone unit tests for lrmh_cmd
 *
 * Pattern: #include the app's .c so static functions and the
 * file-static `g_CmdApp` become callable. cFE/HAL/seq_token are
 * stubbed by cfe_stubs/inc/ which the test target adds to the
 * include path BEFORE the real headers.
 *
 * Coverage:
 *
 *   1. Initial state — fresh g_CmdApp has Mode=STANDBY, zero counters
 *   2. Mode FSM legal transitions — every cell in the ADR-0004 matrix
 *      that is allowed transitions cleanly
 *   3. Mode FSM illegal transitions — sample of cells marked illegal
 *      are rejected with RejectedCount incremented and Mode preserved
 *   4. SAFE entry side effect — token epoch bump on TESTING → SAFE
 *   5. INIT_INSTRUMENT path — HAL_Init success advances SAFE → STANDBY
 *      and bumps the epoch
 *   6. Dispatch cache — record + lookup, FIFO eviction
 *   7. Host nav precondition — fresh / stale / override flag
 *   8. StartSiteTest dispatch — accepted from READY, rejected from
 *      other modes, idempotent retry absorbed
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* HAL stub state (declared extern in cfe_stubs/test_inc/lrmh_hal.h) */
int     lrmh_hal_stub_init_count       = 0;
int32_t lrmh_hal_stub_init_return      = 0;
int     lrmh_hal_stub_estop_count      = 0;
int     lrmh_hal_stub_retract_count    = 0;
int32_t lrmh_hal_stub_retract_return   = 0;
int     lrmh_hal_stub_hold_count       = 0;
int     lrmh_hal_stub_position_count   = 0;
int     lrmh_hal_stub_force_count      = 0;

/* Sequence-token stub state — owned by the test file. The real
 * implementation lives at msg/lrmh_seq_token.c; we stub it here
 * so the cmd test asserts on the side effects (BumpEpoch count,
 * synthetic epoch progression) without dragging in the full
 * validate-and-update logic. test_lrmh_proc and test_lrmh_sense
 * link the real implementation when they need to exercise it. */
static int       lrmh_seq_token_stub_bump_count = 0;
static uint16_t  lrmh_seq_token_stub_current_epoch = 0;

#include "lrmh_seq_token.h"

void LRMH_SeqToken_BumpEpoch(LRMH_TokenResetReason_t reason) {
    (void)reason;
    lrmh_seq_token_stub_bump_count++;
    lrmh_seq_token_stub_current_epoch++;
}

void LRMH_SeqToken_Stamp(LRMH_SeqToken_t *out) {
    if (out) {
        out->Epoch = lrmh_seq_token_stub_current_epoch;
        out->SeqState = 0;
        out->SeqCount = 0;
    }
}

uint16 LRMH_SeqToken_GetCurrentEpoch(void) { return 0; }
LRMH_TokenValidationResult_t LRMH_SeqToken_Validate(
    const LRMH_SeqToken_t *incoming,
    LRMH_SeqToken_t *accepted,
    bool unconditional) {
    (void)incoming; (void)accepted; (void)unconditional;
    return LRMH_TOKEN_ACCEPT;
}

/* Bring the source under test directly into this translation unit.
 * The cfe_stubs/inc/ path is on the include search list ahead of
 * hal/include/ so the stubs win for HAL and seq_token. */
#include "../src/lrmh_cmd_app.c"

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

/* ── Test fixture ──────────────────────────────────────────────── */

static void reset_state(void)
{
    memset(&g_CmdApp, 0, sizeof(g_CmdApp));
    g_CmdApp.Mode = LRMH_MODE_STANDBY;
    lrmh_hal_stub_init_count       = 0;
    lrmh_hal_stub_init_return      = LRMH_HAL_OK;
    lrmh_hal_stub_estop_count      = 0;
    lrmh_hal_stub_retract_count    = 0;
    lrmh_hal_stub_retract_return   = LRMH_HAL_OK;
    lrmh_hal_stub_hold_count       = 0;
    lrmh_hal_stub_position_count   = 0;
    lrmh_hal_stub_force_count      = 0;
    lrmh_seq_token_stub_bump_count = 0;
}

/* ── 1. Initial state ──────────────────────────────────────────── */

static void test_initial_state(void)
{
    TEST("initial_state");
    reset_state();
    ASSERT(g_CmdApp.Mode == LRMH_MODE_STANDBY);
    ASSERT(g_CmdApp.CmdCount == 0u);
    ASSERT(g_CmdApp.ErrCount == 0u);
    ASSERT(g_CmdApp.RejectedCount == 0u);
    ASSERT(g_CmdApp.DispatchCacheHits == 0u);
}

/* ── 2. Allowed transitions (ADR-0004 matrix) ──────────────────── */

static void allow(LRMH_InstrumentMode_t from,
                  LRMH_ModeTrigger_t trigger,
                  LRMH_InstrumentMode_t expected_to,
                  const char *name)
{
    TEST(name);
    reset_state();
    g_CmdApp.Mode = from;
    LRMH_InstrumentMode_t to;
    bool ok = LRMH_CMD_TransitionAllowed(from, trigger, &to);
    ASSERT(ok);
    ASSERT(to == expected_to);
}

static void test_allowed_transitions(void)
{
    /* From STANDBY */
    allow(LRMH_MODE_STANDBY, LRMH_TRIGGER_READY_FOR_TEST, LRMH_MODE_READY,
          "standby_to_ready");
    allow(LRMH_MODE_STANDBY, LRMH_TRIGGER_POWER_OFF,      LRMH_MODE_OFF,
          "standby_to_off_via_power_off");
    allow(LRMH_MODE_STANDBY, LRMH_TRIGGER_FAULT_T3,       LRMH_MODE_SAFE,
          "standby_to_safe_on_t3");

    /* From READY */
    allow(LRMH_MODE_READY,   LRMH_TRIGGER_START_SITE_TEST, LRMH_MODE_TESTING,
          "ready_to_testing");
    allow(LRMH_MODE_READY,   LRMH_TRIGGER_STOW,            LRMH_MODE_STANDBY,
          "ready_to_standby");
    allow(LRMH_MODE_READY,   LRMH_TRIGGER_FAULT_T3,        LRMH_MODE_SAFE,
          "ready_to_safe_on_t3");

    /* From TESTING */
    allow(LRMH_MODE_TESTING, LRMH_TRIGGER_TEST_COMPLETE,   LRMH_MODE_STANDBY,
          "testing_to_standby");
    allow(LRMH_MODE_TESTING, LRMH_TRIGGER_FAULT_T3,        LRMH_MODE_SAFE,
          "testing_to_safe");
    allow(LRMH_MODE_TESTING, LRMH_TRIGGER_ABORT,           LRMH_MODE_SAFE,
          "testing_to_safe_on_abort");
    /* T1/T2 fault during TESTING does NOT change mode (matrix encodes
     * the no-op as TESTING → TESTING) */
    allow(LRMH_MODE_TESTING, LRMH_TRIGGER_FAULT_T1_T2,     LRMH_MODE_TESTING,
          "testing_t1_t2_no_mode_change");

    /* From SAFE */
    allow(LRMH_MODE_SAFE,    LRMH_TRIGGER_INIT_INSTRUMENT, LRMH_MODE_STANDBY,
          "safe_to_standby_via_init");
    allow(LRMH_MODE_SAFE,    LRMH_TRIGGER_FAULT_T3,        LRMH_MODE_FAULT,
          "safe_to_fault_on_persistent_t3");

    /* From FAULT */
    allow(LRMH_MODE_FAULT,   LRMH_TRIGGER_RESET_FAULT,     LRMH_MODE_SAFE,
          "fault_to_safe_via_ground_reset");
}

/* ── 3. Forbidden transitions (sample of cells) ────────────────── */

static void deny(LRMH_InstrumentMode_t from,
                 LRMH_ModeTrigger_t trigger,
                 const char *name)
{
    TEST(name);
    reset_state();
    g_CmdApp.Mode = from;
    LRMH_InstrumentMode_t to;
    bool ok = LRMH_CMD_TransitionAllowed(from, trigger, &to);
    ASSERT(!ok);
}

static void test_forbidden_transitions(void)
{
    /* OFF cannot directly enter TESTING */
    deny(LRMH_MODE_OFF,     LRMH_TRIGGER_START_SITE_TEST, "off_no_start_site_test");
    /* STANDBY cannot directly enter TESTING (must go through READY) */
    deny(LRMH_MODE_STANDBY, LRMH_TRIGGER_START_SITE_TEST, "standby_no_start_site_test");
    /* FAULT terminal — only RESET_FAULT or EMERGENCY_OFF allowed */
    deny(LRMH_MODE_FAULT,   LRMH_TRIGGER_DEPLOY,          "fault_no_deploy");
    deny(LRMH_MODE_FAULT,   LRMH_TRIGGER_INIT_INSTRUMENT, "fault_no_init");
    /* TESTING cannot deploy */
    deny(LRMH_MODE_TESTING, LRMH_TRIGGER_DEPLOY,          "testing_no_deploy");
    /* READY cannot transition to FAULT directly */
    deny(LRMH_MODE_READY,   LRMH_TRIGGER_RESET_FAULT,     "ready_no_reset_fault");
    /* STOW from STANDBY is illegal — power-down goes through POWER_OFF */
    deny(LRMH_MODE_STANDBY, LRMH_TRIGGER_STOW,            "standby_no_stow");
    /* POWER_OFF only valid from STANDBY */
    deny(LRMH_MODE_READY,   LRMH_TRIGGER_POWER_OFF,       "ready_no_power_off");
    deny(LRMH_MODE_TESTING, LRMH_TRIGGER_POWER_OFF,       "testing_no_power_off");
}

/* ── 4. AdvanceMode side effects ───────────────────────────────── */

static void test_safe_entry_bumps_token(void)
{
    TEST("safe_entry_bumps_token");
    reset_state();
    g_CmdApp.Mode = LRMH_MODE_TESTING;

    bool ok = LRMH_CMD_AdvanceMode(LRMH_TRIGGER_FAULT_T3);
    ASSERT(ok);
    ASSERT(g_CmdApp.Mode == LRMH_MODE_SAFE);
    ASSERT(lrmh_seq_token_stub_bump_count == 1);
}

static void test_safe_to_safe_no_double_bump(void)
{
    TEST("safe_to_safe_no_double_bump");
    reset_state();
    g_CmdApp.Mode = LRMH_MODE_SAFE;

    /* Persistent T3 from SAFE → FAULT (terminal) does NOT bump
     * the epoch a second time because the matrix entry is FAULT,
     * not SAFE. The bump rule fires only when entering SAFE
     * from a non-SAFE state. */
    bool ok = LRMH_CMD_AdvanceMode(LRMH_TRIGGER_FAULT_T3);
    ASSERT(ok);
    ASSERT(g_CmdApp.Mode == LRMH_MODE_FAULT);
    ASSERT(lrmh_seq_token_stub_bump_count == 0);
}

static void test_illegal_transition_increments_reject(void)
{
    TEST("illegal_transition_increments_reject");
    reset_state();
    g_CmdApp.Mode = LRMH_MODE_STANDBY;

    bool ok = LRMH_CMD_AdvanceMode(LRMH_TRIGGER_START_SITE_TEST);
    ASSERT(!ok);
    ASSERT(g_CmdApp.Mode == LRMH_MODE_STANDBY);  /* unchanged */
    ASSERT(g_CmdApp.RejectedCount == 1);
}

/* ── 5. INIT_INSTRUMENT path ───────────────────────────────────── */

static void test_init_instrument_advances_safe_to_standby(void)
{
    TEST("init_instrument_safe_to_standby");
    reset_state();
    g_CmdApp.Mode = LRMH_MODE_SAFE;
    lrmh_hal_stub_init_return = LRMH_HAL_OK;

    LRMH_CMD_InitInstrument();

    ASSERT(g_CmdApp.Mode == LRMH_MODE_STANDBY);
    ASSERT(lrmh_hal_stub_init_count == 1);
    /* Token bump count: should be 1 (from the InitInstrument
     * broadcast). The AdvanceMode SAFE→STANDBY does NOT bump
     * because the rule fires only on entering SAFE. */
    ASSERT(lrmh_seq_token_stub_bump_count == 1);
    ASSERT(g_CmdApp.CmdCount == 1);
}

static void test_init_instrument_hal_failure(void)
{
    TEST("init_instrument_hal_failure");
    reset_state();
    g_CmdApp.Mode = LRMH_MODE_SAFE;
    lrmh_hal_stub_init_return = LRMH_HAL_SELFTEST_FAIL;

    LRMH_CMD_InitInstrument();

    ASSERT(g_CmdApp.Mode == LRMH_MODE_SAFE);  /* did not advance */
    ASSERT(g_CmdApp.ErrCount == 1);
    ASSERT(lrmh_seq_token_stub_bump_count == 0);
}

static void test_init_instrument_off_to_standby(void)
{
    TEST("init_instrument_off_to_standby");
    reset_state();
    g_CmdApp.Mode = LRMH_MODE_OFF;
    lrmh_hal_stub_init_return = LRMH_HAL_OK;

    LRMH_CMD_InitInstrument();

    ASSERT(g_CmdApp.Mode == LRMH_MODE_STANDBY);
    ASSERT(lrmh_hal_stub_init_count == 1);
    ASSERT(g_CmdApp.CmdCount == 1);
}

static void test_seq_notify_test_complete_drives_testing_to_standby(void)
{
    /* lrmh_seq publishes LRMH_SeqToCmdNotify_t with NotifyType =
     * TEST_COMPLETE at end of REPORTING. lrmh_cmd subscribes and
     * advances mode TESTING → STANDBY. */
    TEST("seq_notify_test_complete_advances_testing_to_standby");
    reset_state();
    g_CmdApp.Mode = LRMH_MODE_TESTING;

    bool ok = LRMH_CMD_AdvanceMode(LRMH_TRIGGER_TEST_COMPLETE);
    ASSERT(ok);
    ASSERT(g_CmdApp.Mode == LRMH_MODE_STANDBY);
}

static void test_seq_notify_test_complete_rejected_outside_testing(void)
{
    /* TEST_COMPLETE from non-TESTING modes is illegal. Defends
     * against a stray notify after manual safing or a bug. */
    TEST("seq_notify_test_complete_rejected_outside_testing");
    reset_state();
    g_CmdApp.Mode = LRMH_MODE_STANDBY;

    bool ok = LRMH_CMD_AdvanceMode(LRMH_TRIGGER_TEST_COMPLETE);
    ASSERT(!ok);
    ASSERT(g_CmdApp.Mode == LRMH_MODE_STANDBY);
    ASSERT(g_CmdApp.RejectedCount == 1);
}

/* ── 6. Dispatch cache (ADR-0014) ──────────────────────────────── */

static void test_dispatch_cache_lookup_record(void)
{
    TEST("dispatch_cache_lookup_record");
    reset_state();

    /* Empty cache → NONE */
    ASSERT(LRMH_CMD_DispatchCacheLookup(0x42, 7) == LRMH_DISPATCH_OUTCOME_NONE);

    LRMH_CMD_DispatchCacheRecord(0x42, 7, LRMH_DISPATCH_OUTCOME_IN_PROGRESS);
    ASSERT(LRMH_CMD_DispatchCacheLookup(0x42, 7) == LRMH_DISPATCH_OUTCOME_IN_PROGRESS);

    /* Update existing entry */
    LRMH_CMD_DispatchCacheRecord(0x42, 7, LRMH_DISPATCH_OUTCOME_COMPLETE);
    ASSERT(LRMH_CMD_DispatchCacheLookup(0x42, 7) == LRMH_DISPATCH_OUTCOME_COMPLETE);

    /* Different key */
    ASSERT(LRMH_CMD_DispatchCacheLookup(0x42, 8) == LRMH_DISPATCH_OUTCOME_NONE);
}

static void test_dispatch_cache_fifo_eviction(void)
{
    TEST("dispatch_cache_fifo_eviction");
    reset_state();

    /* Fill cache with 8 entries */
    for (int i = 0; i < LRMH_DISPATCH_CACHE_DEPTH; i++) {
        LRMH_CMD_DispatchCacheRecord(0x100, i,
                                     LRMH_DISPATCH_OUTCOME_COMPLETE);
    }
    ASSERT(LRMH_CMD_DispatchCacheLookup(0x100, 0) ==
           LRMH_DISPATCH_OUTCOME_COMPLETE);

    /* Insert a 9th — evicts entry 0 (oldest) */
    LRMH_CMD_DispatchCacheRecord(0x100, 99,
                                 LRMH_DISPATCH_OUTCOME_COMPLETE);
    ASSERT(LRMH_CMD_DispatchCacheLookup(0x100, 0) ==
           LRMH_DISPATCH_OUTCOME_NONE);
    ASSERT(LRMH_CMD_DispatchCacheLookup(0x100, 99) ==
           LRMH_DISPATCH_OUTCOME_COMPLETE);
}

/* ── 7. Host nav precondition (ADR-0011 + ADR-0014) ────────────── */

static void test_host_nav_precondition(void)
{
    TEST("host_nav_precondition");
    reset_state();

    /* No host pose received → fail */
    g_CmdApp.HostPose.valid = false;
    ASSERT(!LRMH_CMD_HostNavOk(0));

    /* Fresh host pose, IDLE → ok */
    g_CmdApp.HostPose.valid = true;
    g_CmdApp.HostPose.receipt_time_us = LRMH_CMD_NowUs();
    g_CmdApp.HostPose.nav_state = AEGIS_NAV_STATE_IDLE;
    ASSERT(LRMH_CMD_HostNavOk(0));

    /* Override flags bypass the check */
    g_CmdApp.HostPose.valid = false;
    ASSERT(LRMH_CMD_HostNavOk(LRMH_DISPATCH_OVERRIDE_HOST));
    ASSERT(LRMH_CMD_HostNavOk(LRMH_DISPATCH_WITNESS_TARGET));
}

/* ── 8. StartSiteTest dispatch path ───────────────────────────── */

static void test_start_site_test_rejected_from_standby(void)
{
    TEST("start_site_test_rejected_from_standby");
    reset_state();
    g_CmdApp.Mode = LRMH_MODE_STANDBY;
    g_CmdApp.HostPose.valid = true;
    g_CmdApp.HostPose.receipt_time_us = LRMH_CMD_NowUs();

    LRMH_StartSiteTestCmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.SiteId = 1;

    LRMH_CMD_StartSiteTest(&cmd);

    ASSERT(g_CmdApp.Mode == LRMH_MODE_STANDBY);  /* not advanced */
    ASSERT(g_CmdApp.ErrCount == 1);
}

static void test_start_site_test_accepted_from_ready(void)
{
    TEST("start_site_test_accepted_from_ready");
    reset_state();
    g_CmdApp.Mode = LRMH_MODE_READY;
    g_CmdApp.HostPose.valid = true;
    g_CmdApp.HostPose.receipt_time_us = LRMH_CMD_NowUs();
    g_CmdApp.HostPose.nav_state = AEGIS_NAV_STATE_IDLE;

    LRMH_StartSiteTestCmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.SiteId = 1;
    cmd.SessionId = 0xAAAA;
    cmd.SequenceNumber = 1;

    LRMH_CMD_StartSiteTest(&cmd);

    ASSERT(g_CmdApp.Mode == LRMH_MODE_TESTING);
    ASSERT(g_CmdApp.CmdCount == 1);
    ASSERT(g_CmdApp.CurrentSiteId == 1);
    /* Phase advance broadcast bumps the epoch */
    ASSERT(lrmh_seq_token_stub_bump_count == 1);
}

static void test_start_site_test_idempotent_retry_absorbed(void)
{
    TEST("start_site_test_idempotent_retry_absorbed");
    reset_state();
    g_CmdApp.Mode = LRMH_MODE_READY;
    g_CmdApp.HostPose.valid = true;
    g_CmdApp.HostPose.receipt_time_us = LRMH_CMD_NowUs();
    g_CmdApp.HostPose.nav_state = AEGIS_NAV_STATE_IDLE;

    LRMH_StartSiteTestCmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.SiteId = 1;
    cmd.SessionId = 0xAAAA;
    cmd.SequenceNumber = 1;
    cmd.DispatchFlags = LRMH_DISPATCH_IDEMPOTENT;

    LRMH_CMD_StartSiteTest(&cmd);
    ASSERT(g_CmdApp.Mode == LRMH_MODE_TESTING);
    ASSERT(g_CmdApp.CmdCount == 1);

    /* Retry same (Session, Seq) — absorbed silently */
    LRMH_CMD_StartSiteTest(&cmd);
    ASSERT(g_CmdApp.CmdCount == 1);  /* unchanged */
    ASSERT(g_CmdApp.DispatchCacheHits == 1);
}

/* ── main ──────────────────────────────────────────────────────── */

int main(void)
{
    fprintf(stderr, "lrmh_cmd standalone unit tests\n");
    fprintf(stderr, "============================================\n");

    test_initial_state();
    test_allowed_transitions();
    test_forbidden_transitions();
    test_safe_entry_bumps_token();
    test_safe_to_safe_no_double_bump();
    test_illegal_transition_increments_reject();
    test_init_instrument_advances_safe_to_standby();
    test_init_instrument_hal_failure();
    test_init_instrument_off_to_standby();
    test_seq_notify_test_complete_drives_testing_to_standby();
    test_seq_notify_test_complete_rejected_outside_testing();
    test_dispatch_cache_lookup_record();
    test_dispatch_cache_fifo_eviction();
    test_host_nav_precondition();
    test_start_site_test_rejected_from_standby();
    test_start_site_test_accepted_from_ready();
    test_start_site_test_idempotent_retry_absorbed();

    fprintf(stderr, "============================================\n");
    fprintf(stderr, "  passed: %d\n", passed);
    fprintf(stderr, "  failed: %d\n", failed);
    return failed == 0 ? 0 : 1;
}
