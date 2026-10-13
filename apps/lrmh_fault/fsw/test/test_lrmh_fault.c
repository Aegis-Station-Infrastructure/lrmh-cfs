/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_lrmh_fault.c
 * @brief Standalone unit tests for lrmh_fault
 *
 * Coverage focus:
 *
 *   1. Initial state — fresh g_FaultApp, disabled by default
 *   2. Tier lookup — every fault category resolves to its
 *      ADR-0008 tier
 *   3. Bit-position helper handles single-bit input
 *   4. Per-fault tier dispatch — each category goes through
 *      the correct dispatcher
 *   5. T1 attempt counter increments
 *   6. T1 → T2 escalation at LRMH_T1_MAX_ATTEMPTS_PER_PHASE
 *   7. T2 → T3 escalation at LRMH_T2_ESCALATION_THRESHOLD
 *   8. Coalescing (suppress while fault active)
 *   9. Per-mode evaluation (STANDBY → only T3 categories)
 *  10. Polled path — sample cache stale → SENSOR_TIMEOUT raise
 *  11. Phase advance resets T1 counter
 *  12. Disable suppresses raises
 *  13. Reset clears state but preserves totals
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "lrmh_hal.h"

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

/* SB transmit capture */
#define MAX_SB_LOG 32
static int g_sb_count = 0;

#undef CFE_SB_TransmitMsg
#define CFE_SB_TransmitMsg(msg, inc) \
    do { (void)(msg); (void)(inc); g_sb_count++; } while (0)

#include "../src/lrmh_fault_app.c"

/* ── Test framework ────────────────────────────────────────────── */

static int passed = 0;
static int failed = 0;
static const char *current_test = "";

#define ASSERT(cond) do {                                              \
    if (!(cond)) {                                                     \
        fprintf(stderr, "  [%s] FAIL: %s  (%s:%d)\n",                  \
                current_test, #cond, __FILE__, __LINE__);              \
        failed++;                                                      \
    } else { passed++; }                                               \
} while (0)

#define TEST(name) do {                                                \
    current_test = name;                                               \
    fprintf(stderr, "RUN  %s\n", name);                                \
} while (0)

static void reset_state(void)
{
    memset(&g_FaultApp, 0, sizeof(g_FaultApp));
    g_FaultApp.FaultEnabled = 1;
    g_FaultApp.CurrentMode  = LRMH_MODE_TESTING;
    g_FaultApp.CurrentPhase = LRMH_PHASE_BASELINE_PEN;   /* a measured phase: T1 resumes are dispatched */
    g_FaultApp.LastRecoveryTier = LRMH_TIER_NONE;
    g_sb_count = 0;
}

/* ── Tests ─────────────────────────────────────────────────────── */

static void test_initial_state(void)
{
    TEST("initial_state");
    memset(&g_FaultApp, 0, sizeof(g_FaultApp));
    ASSERT(g_FaultApp.FaultEnabled == 0);  /* disabled at boot */
    ASSERT(g_FaultApp.FaultFlags == 0);
    ASSERT(g_FaultApp.LastRecoveryTier == LRMH_TIER_NONE);
}

static void test_bit_position_helper(void)
{
    TEST("bit_position_helper");
    ASSERT(LRMH_FAULT_BitPosition(LRMH_FAULT_FLAG_STALL) == 0);
    ASSERT(LRMH_FAULT_BitPosition(LRMH_FAULT_FLAG_OVERCURRENT) == 1);
    ASSERT(LRMH_FAULT_BitPosition(LRMH_FAULT_FLAG_HOST_NAV_TIMEOUT) == 10);
    ASSERT(LRMH_FAULT_BitPosition(0x0003) == -1);  /* multi-bit */
    ASSERT(LRMH_FAULT_BitPosition(0) == -1);       /* zero */
}

static void test_tier_lookup_all_categories(void)
{
    TEST("tier_lookup_all_categories");
    ASSERT(LRMH_FAULT_LookupTier(LRMH_FAULT_FLAG_STALL)            == LRMH_TIER_T2_SKIP);
    ASSERT(LRMH_FAULT_LookupTier(LRMH_FAULT_FLAG_OVERCURRENT)      == LRMH_TIER_T3_SAFE);
    ASSERT(LRMH_FAULT_LookupTier(LRMH_FAULT_FLAG_FORCE_SATURATION) == LRMH_TIER_T2_SKIP);
    ASSERT(LRMH_FAULT_LookupTier(LRMH_FAULT_FLAG_ENCODER_LOSS)     == LRMH_TIER_T1_RESUME);
    ASSERT(LRMH_FAULT_LookupTier(LRMH_FAULT_FLAG_THERMAL)          == LRMH_TIER_T3_SAFE);
    ASSERT(LRMH_FAULT_LookupTier(LRMH_FAULT_FLAG_FORCE_RATE)       == LRMH_TIER_T1_RESUME);
    ASSERT(LRMH_FAULT_LookupTier(LRMH_FAULT_FLAG_HARD_STOP)        == LRMH_TIER_T2_SKIP);
    ASSERT(LRMH_FAULT_LookupTier(LRMH_FAULT_FLAG_SENSOR_TIMEOUT)   == LRMH_TIER_T1_RESUME);
    ASSERT(LRMH_FAULT_LookupTier(LRMH_FAULT_FLAG_DEPTH_DISAGREE)   == LRMH_TIER_T2_SKIP);
    ASSERT(LRMH_FAULT_LookupTier(LRMH_FAULT_FLAG_BUS_VOLTAGE)      == LRMH_TIER_T3_SAFE);
    ASSERT(LRMH_FAULT_LookupTier(LRMH_FAULT_FLAG_HOST_NAV_TIMEOUT) == LRMH_TIER_T2_SKIP);
}

/* ── Per-fault dispatch in TESTING mode ───────────────────────── */

static void test_t1_dispatch(void)
{
    TEST("t1_dispatch");
    reset_state();
    LRMH_FAULT_Raise(LRMH_FAULT_FLAG_FORCE_RATE);
    /* A T1 resume is the whole recovery: the bit clears at dispatch so
     * a repeat counts as a new attempt. */
    ASSERT((g_FaultApp.FaultFlags & LRMH_FAULT_FLAG_FORCE_RATE) == 0);
    ASSERT(g_FaultApp.LastRecoveryTier == LRMH_TIER_T1_RESUME);
    ASSERT(g_FaultApp.T1AttemptsCurrentPhase == 1);
    ASSERT(g_FaultApp.TotalT1Events == 1);
}

static void test_t2_dispatch(void)
{
    TEST("t2_dispatch");
    reset_state();
    LRMH_FAULT_Raise(LRMH_FAULT_FLAG_FORCE_SATURATION);
    ASSERT(g_FaultApp.FaultFlags & LRMH_FAULT_FLAG_FORCE_SATURATION);
    ASSERT(g_FaultApp.LastRecoveryTier == LRMH_TIER_T2_SKIP);
    ASSERT(g_FaultApp.T2EventsInWindow == 1);
    ASSERT(g_FaultApp.TotalT2Events == 1);
}

static void test_t3_dispatch(void)
{
    TEST("t3_dispatch");
    reset_state();
    LRMH_FAULT_Raise(LRMH_FAULT_FLAG_OVERCURRENT);
    ASSERT(g_FaultApp.FaultFlags & LRMH_FAULT_FLAG_OVERCURRENT);
    ASSERT(g_FaultApp.LastRecoveryTier == LRMH_TIER_T3_SAFE);
    ASSERT(g_FaultApp.TotalT3Events == 1);
}

/* ── Escalation ───────────────────────────────────────────────── */

static void test_t1_to_t2_escalation(void)
{
    TEST("t1_to_t2_escalation");
    reset_state();

    /* Raise the same T1 fault N+1 times. The bit must be cleared
     * between raises to bypass coalescing — that simulates the
     * recovery completing between attempts. */
    for (int i = 0; i < LRMH_T1_MAX_ATTEMPTS_PER_PHASE; i++) {
        g_FaultApp.FaultFlags &= ~LRMH_FAULT_FLAG_FORCE_RATE;
        LRMH_FAULT_Raise(LRMH_FAULT_FLAG_FORCE_RATE);
    }
    ASSERT(g_FaultApp.T1AttemptsCurrentPhase == LRMH_T1_MAX_ATTEMPTS_PER_PHASE);
    ASSERT(g_FaultApp.LastRecoveryTier == LRMH_TIER_T1_RESUME);

    /* (N+1)th raise → escalates to T2 */
    g_FaultApp.FaultFlags &= ~LRMH_FAULT_FLAG_FORCE_RATE;
    LRMH_FAULT_Raise(LRMH_FAULT_FLAG_FORCE_RATE);
    ASSERT(g_FaultApp.LastRecoveryTier == LRMH_TIER_T2_SKIP);
    ASSERT(g_FaultApp.TotalT2Events == 1);
}

static void test_t2_to_t3_escalation(void)
{
    TEST("t2_to_t3_escalation");
    reset_state();

    /* A T2 on each of N consecutive sites; the window counts sites,
     * so each raise is separated by a site end (IDLE) and a new
     * acquiring phase. Escalation fires on the Nth site. */
    for (int i = 0; i < LRMH_T2_ESCALATION_THRESHOLD; i++) {
        LRMH_FAULT_OnPhaseAdvanceAt(LRMH_PHASE_BASELINE_PEN, 1000000u * (i + 1));
        LRMH_FAULT_Raise(LRMH_FAULT_FLAG_HARD_STOP);
        if (i + 1 < LRMH_T2_ESCALATION_THRESHOLD) {
            LRMH_FAULT_OnPhaseAdvanceAt(LRMH_PHASE_IDLE, 1000000u * (i + 1) + 500000u);
        }
    }
    ASSERT(g_FaultApp.T2EventsInWindow == LRMH_T2_ESCALATION_THRESHOLD);
    /* On the threshold-th event, escalation fires → T3 dispatched */
    ASSERT(g_FaultApp.LastRecoveryTier == LRMH_TIER_T3_SAFE);
    ASSERT(g_FaultApp.TotalT3Events == 1);
}

/* ── Coalescing ───────────────────────────────────────────────── */

static void test_coalescing_suppresses_duplicate(void)
{
    TEST("coalescing_suppresses_duplicate");
    reset_state();

    LRMH_FAULT_Raise(LRMH_FAULT_FLAG_OVERCURRENT);
    ASSERT(g_FaultApp.TotalT3Events == 1);

    /* Second raise of the same flag is silently absorbed */
    LRMH_FAULT_Raise(LRMH_FAULT_FLAG_OVERCURRENT);
    ASSERT(g_FaultApp.TotalT3Events == 1);  /* unchanged */
}

/* ── Per-mode evaluation (ADR-0013) ───────────────────────────── */

static void test_standby_mode_forces_t3(void)
{
    TEST("standby_mode_forces_t3");
    reset_state();
    g_FaultApp.CurrentMode = LRMH_MODE_STANDBY;

    /* In STANDBY, a normally-T1 fault should be forced to T3
     * because there's no test in progress to recover */
    LRMH_FAULT_Raise(LRMH_FAULT_FLAG_FORCE_RATE);
    ASSERT(g_FaultApp.LastRecoveryTier == LRMH_TIER_T3_SAFE);
    ASSERT(g_FaultApp.TotalT3Events == 1);
    ASSERT(g_FaultApp.TotalT1Events == 0);
}

/* ── Polled path (ADR-0013) ───────────────────────────────────── */

static void test_polled_path_no_sample_in_testing(void)
{
    TEST("polled_path_no_sample_in_testing");
    reset_state();
    g_FaultApp.CurrentMode = LRMH_MODE_TESTING;
    LRMH_FAULT_OnPhaseAdvanceAt(LRMH_PHASE_BASELINE_PEN, 500000);

    /* No sample yet, 100 ms into the acquiring phase → within the floor */
    LRMH_FAULT_PolledEvaluate(600000);
    ASSERT((g_FaultApp.FaultFlags & LRMH_FAULT_FLAG_SENSOR_TIMEOUT) == 0);

    /* No sample 500 ms into it → SENSOR_TIMEOUT raised */
    LRMH_FAULT_PolledEvaluate(1000000);
    ASSERT(g_FaultApp.TimeoutCount >= 1);   /* raised (T1 tier: the bit clears at dispatch) */
    ASSERT(g_FaultApp.TimeoutCount == 1);
}

static void test_polled_path_no_sample_before_acquisition(void)
{
    TEST("polled_path_no_sample_before_acquisition");
    reset_state();
    g_FaultApp.CurrentMode = LRMH_MODE_TESTING;

    /* PRECHECK and POSITIONING come before the first sample: no fault */
    LRMH_FAULT_OnPhaseAdvanceAt(LRMH_PHASE_PRECHECK, 0);
    LRMH_FAULT_PolledEvaluate(1000000);
    LRMH_FAULT_OnPhaseAdvanceAt(LRMH_PHASE_POSITIONING, 1000000);
    LRMH_FAULT_PolledEvaluate(2000000);
    ASSERT((g_FaultApp.FaultFlags & LRMH_FAULT_FLAG_SENSOR_TIMEOUT) == 0);

    /* Nor after acquisition stops */
    LRMH_FAULT_OnPhaseAdvanceAt(LRMH_PHASE_PROCESSING, 2000000);
    LRMH_FAULT_PolledEvaluate(3000000);
    ASSERT((g_FaultApp.FaultFlags & LRMH_FAULT_FLAG_SENSOR_TIMEOUT) == 0);
}

static void test_polled_path_old_sample_does_not_count(void)
{
    TEST("polled_path_old_sample_does_not_count");
    reset_state();
    g_FaultApp.CurrentMode = LRMH_MODE_TESTING;

    /* A sample from the previous test, then a new acquiring phase */
    LRMH_SampleTlm_t s;
    memset(&s, 0, sizeof(s));
    LRMH_FAULT_OnSampleTlm(&s, 100000);
    LRMH_FAULT_OnPhaseAdvanceAt(LRMH_PHASE_BASELINE_PEN, 500000);

    LRMH_FAULT_PolledEvaluate(600000);
    ASSERT((g_FaultApp.FaultFlags & LRMH_FAULT_FLAG_SENSOR_TIMEOUT) == 0);
    LRMH_FAULT_PolledEvaluate(800000);
    ASSERT(g_FaultApp.TimeoutCount >= 1);   /* raised (T1 tier: the bit clears at dispatch) */
}

static void test_polled_path_no_sample_in_standby(void)
{
    TEST("polled_path_no_sample_in_standby");
    reset_state();
    g_FaultApp.CurrentMode = LRMH_MODE_STANDBY;

    /* No sample but not in TESTING → no fault raised */
    LRMH_FAULT_PolledEvaluate(1000000);
    ASSERT((g_FaultApp.FaultFlags & LRMH_FAULT_FLAG_SENSOR_TIMEOUT) == 0);
}

static void test_polled_path_stale_sample(void)
{
    TEST("polled_path_stale_sample");
    reset_state();
    g_FaultApp.CurrentMode = LRMH_MODE_TESTING;
    LRMH_FAULT_OnPhaseAdvanceAt(LRMH_PHASE_BASELINE_PEN, 500000);

    /* Cache a sample at t=1.0s */
    LRMH_SampleTlm_t s;
    memset(&s, 0, sizeof(s));
    s.sample_time_us = 1000000;
    LRMH_FAULT_OnSampleTlm(&s, 1000000);

    /* Evaluate at t=1.5s — 500 ms > LRMH_SENSOR_TIMEOUT_MS (200 ms) */
    LRMH_FAULT_PolledEvaluate(1500000);
    ASSERT(g_FaultApp.TimeoutCount >= 1);   /* raised (T1 tier: the bit clears at dispatch) */
}

static void test_polled_path_fresh_sample(void)
{
    TEST("polled_path_fresh_sample");
    reset_state();
    g_FaultApp.CurrentMode = LRMH_MODE_TESTING;
    LRMH_FAULT_OnPhaseAdvanceAt(LRMH_PHASE_BASELINE_PEN, 500000);

    LRMH_SampleTlm_t s;
    memset(&s, 0, sizeof(s));
    s.sample_time_us = 1000000;
    LRMH_FAULT_OnSampleTlm(&s, 1000000);

    /* Evaluate at t=1.05s — 50 ms < 200 ms threshold */
    LRMH_FAULT_PolledEvaluate(1050000);
    ASSERT((g_FaultApp.FaultFlags & LRMH_FAULT_FLAG_SENSOR_TIMEOUT) == 0);
}

/* ── Phase advance + counter management ───────────────────────── */

static void test_phase_advance_resets_t1_counter(void)
{
    TEST("phase_advance_resets_t1_counter");
    reset_state();

    /* Accumulate 2 T1 attempts */
    g_FaultApp.FaultFlags &= ~LRMH_FAULT_FLAG_FORCE_RATE;
    LRMH_FAULT_Raise(LRMH_FAULT_FLAG_FORCE_RATE);
    g_FaultApp.FaultFlags &= ~LRMH_FAULT_FLAG_FORCE_RATE;
    LRMH_FAULT_Raise(LRMH_FAULT_FLAG_FORCE_RATE);
    ASSERT(g_FaultApp.T1AttemptsCurrentPhase == 2);

    /* Phase advance — counter resets */
    LRMH_FAULT_OnPhaseAdvance(LRMH_PHASE_LOAD_RAMP);
    ASSERT(g_FaultApp.T1AttemptsCurrentPhase == 0);
    ASSERT(g_FaultApp.CurrentPhase == LRMH_PHASE_LOAD_RAMP);
}

/* ── Enable / disable / reset ─────────────────────────────────── */

static void test_disable_suppresses_raises(void)
{
    TEST("disable_suppresses_raises");
    reset_state();
    g_FaultApp.FaultEnabled = 0;

    LRMH_FAULT_Raise(LRMH_FAULT_FLAG_OVERCURRENT);
    ASSERT(g_FaultApp.FaultFlags == 0);
    ASSERT(g_FaultApp.TotalT3Events == 0);
}

static void test_reset_clears_active_state(void)
{
    TEST("reset_clears_active_state");
    reset_state();

    LRMH_FAULT_Raise(LRMH_FAULT_FLAG_OVERCURRENT);
    LRMH_FAULT_Raise(LRMH_FAULT_FLAG_THERMAL);
    ASSERT(g_FaultApp.TotalT3Events == 2);
    ASSERT(g_FaultApp.FaultActive == 1);

    LRMH_FAULT_ResetAll();
    ASSERT(g_FaultApp.FaultFlags == 0);
    ASSERT(g_FaultApp.FaultActive == 0);
    ASSERT(g_FaultApp.T1AttemptsCurrentPhase == 0);
    ASSERT(g_FaultApp.T2EventsInWindow == 0);
    /* Totals are NOT reset by ResetAll — preserved for HK */
    ASSERT(g_FaultApp.TotalT3Events == 2);
}

/* ── main ──────────────────────────────────────────────────────── */

int main(void)
{
    fprintf(stderr, "lrmh_fault standalone unit tests\n");
    fprintf(stderr, "============================================\n");

    test_initial_state();
    test_bit_position_helper();
    test_tier_lookup_all_categories();
    test_t1_dispatch();
    test_t2_dispatch();
    test_t3_dispatch();
    test_t1_to_t2_escalation();
    test_t2_to_t3_escalation();
    test_coalescing_suppresses_duplicate();
    test_standby_mode_forces_t3();
    test_polled_path_no_sample_in_testing();
    test_polled_path_no_sample_before_acquisition();
    test_polled_path_old_sample_does_not_count();
    test_polled_path_no_sample_in_standby();
    test_polled_path_stale_sample();
    test_polled_path_fresh_sample();
    test_phase_advance_resets_t1_counter();
    test_disable_suppresses_raises();
    test_reset_clears_active_state();

    fprintf(stderr, "============================================\n");
    fprintf(stderr, "  passed: %d\n", passed);
    fprintf(stderr, "  failed: %d\n", failed);
    return failed == 0 ? 0 : 1;
}
