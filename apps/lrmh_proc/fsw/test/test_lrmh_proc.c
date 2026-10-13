/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_lrmh_proc.c
 * @brief Standalone unit tests for lrmh_proc
 *
 * Coverage focus:
 *
 *   1. Initial state — fresh g_ProcApp, empty buffer
 *   2. Sample ingestion — append to buffer, advance count
 *   3. Phase-start rejection (ADR-0009) — sample with t < phase_start
 *      is rejected and StaleSampleCount increments
 *   4. Insufficient samples → PARTIAL quality
 *   5. Penetration profile — max force / max sinkage from synthetic data
 *   6. Stiffness via linear regression — known slope from linear data
 *   7. Layer break detection — synthetic two-layer profile
 *   8. Compaction delta — post-pen vs baseline
 *   9. Buffer full handling
 *  10. Token reset clears state and updates phase start
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

/* Stub HAL header — lrmh_proc doesn't call HAL but its header
 * chain pulls it in via lrmh_sense_msg.h → lrmh_msg.h. */
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

/* Real seq_token library */
#include "lrmh_seq_token.h"
#include "../../../../msg/lrmh_seq_token.c"

/* SB transmit capture */
#define MAX_SB_LOG 32
static int g_sb_count = 0;

#undef CFE_SB_TransmitMsg
#define CFE_SB_TransmitMsg(msg, inc) \
    do { (void)(msg); (void)(inc); g_sb_count++; } while (0)

#include "../src/lrmh_proc_app.c"

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

#define ASSERT_NEAR(a, b, tol) do {                                    \
    double _diff = (double)(a) - (double)(b);                          \
    if (_diff < 0) _diff = -_diff;                                     \
    if (_diff > (tol)) {                                               \
        fprintf(stderr, "  [%s] FAIL: %s ≈ %s (%g vs %g, diff %g > %g)\n", \
                current_test, #a, #b, (double)(a), (double)(b),        \
                _diff, (double)(tol));                                 \
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
    memset(&g_ProcApp, 0, sizeof(g_ProcApp));
    g_sb_count = 0;
}

/* Build a sample TLM with specified time, force, depth */
static LRMH_SampleTlm_t make_sample(uint64 t_us, float force_n, float depth_m)
{
    LRMH_SampleTlm_t s;
    memset(&s, 0, sizeof(s));
    s.sample_time_us = t_us;
    s.force_n = force_n;
    s.depth_m = depth_m;
    s.stroke_m = depth_m;
    s.current_a = 1.5f;
    s.channel_valid = 0xFFFF;
    return s;
}

/* Inject N linear samples: force = slope * depth */
static void inject_linear(uint32 n, float slope, float depth_step)
{
    for (uint32 i = 0; i < n; i++) {
        float depth = depth_step * (float)i;
        float force = slope * depth;
        LRMH_SampleTlm_t s = make_sample(1000000 + i * 10000, force, depth);
        LRMH_PROC_IngestSample(&s);
    }
}

/* Inject a two-layer profile: linear with slope1 up to break, then
 * abruptly transitions to slope2 */
static void inject_two_layer(uint32 n, float slope1, float slope2,
                              float break_depth, float depth_step)
{
    float force = 0.0f;
    for (uint32 i = 0; i < n; i++) {
        float depth = depth_step * (float)i;
        if (depth < break_depth) {
            force = slope1 * depth;
        } else {
            float pre_break_force = slope1 * break_depth;
            force = pre_break_force + slope2 * (depth - break_depth);
        }
        LRMH_SampleTlm_t s = make_sample(1000000 + i * 10000, force, depth);
        LRMH_PROC_IngestSample(&s);
    }
}

/* ── Tests ─────────────────────────────────────────────────────── */

static void test_initial_state(void)
{
    TEST("initial_state");
    reset_state();
    ASSERT(g_ProcApp.SampleCount == 0u);
    ASSERT(g_ProcApp.StaleSampleCount == 0u);
    ASSERT(g_ProcApp.CurrentPhaseStartTime_us == 0u);
}

static void test_sample_ingestion(void)
{
    TEST("sample_ingestion");
    reset_state();

    LRMH_SampleTlm_t s = make_sample(1000000, 100.0f, 0.05f);
    LRMH_PROC_IngestSample(&s);
    ASSERT(g_ProcApp.SampleCount == 1);
    ASSERT(g_ProcApp.Samples[0].force_n == 100.0f);
    ASSERT(g_ProcApp.Samples[0].depth_m == 0.05f);
    ASSERT(g_ProcApp.Samples[0].time_us == 1000000);
}

/* ── ADR-0009 phase-start rejection ───────────────────────────── */

static void test_phase_start_rejection(void)
{
    TEST("phase_start_rejection");
    reset_state();
    g_ProcApp.CurrentPhaseStartTime_us = 5000000;  /* phase started at t=5s */

    /* Sample acquired before phase start → rejected */
    LRMH_SampleTlm_t old_s = make_sample(4000000, 100.0f, 0.05f);
    LRMH_PROC_IngestSample(&old_s);
    ASSERT(g_ProcApp.SampleCount == 0);
    ASSERT(g_ProcApp.StaleSampleCount == 1);

    /* Sample acquired after phase start → accepted */
    LRMH_SampleTlm_t new_s = make_sample(5500000, 100.0f, 0.05f);
    LRMH_PROC_IngestSample(&new_s);
    ASSERT(g_ProcApp.SampleCount == 1);
    ASSERT(g_ProcApp.StaleSampleCount == 1);  /* unchanged */
}

static void test_phase_start_boundary(void)
{
    TEST("phase_start_boundary");
    reset_state();
    g_ProcApp.CurrentPhaseStartTime_us = 5000000;

    /* Exactly at phase start → accepted */
    LRMH_SampleTlm_t boundary = make_sample(5000000, 100.0f, 0.05f);
    LRMH_PROC_IngestSample(&boundary);
    ASSERT(g_ProcApp.SampleCount == 1);
    ASSERT(g_ProcApp.StaleSampleCount == 0);
}

/* ── Phase result computation ─────────────────────────────────── */

static void test_insufficient_samples_partial(void)
{
    TEST("insufficient_samples_partial");
    reset_state();

    /* Inject only 5 samples — below the 10-sample minimum */
    inject_linear(5, 1000.0f, 0.005f);

    LRMH_ProcPhaseResult_t result;
    LRMH_PROC_RunPhase(LRMH_PHASE_BASELINE_PEN, &result);

    ASSERT(result.quality == LRMH_QUALITY_PARTIAL);
    ASSERT(result.sample_count == 5);
    /* Buffer cleared after phase */
    ASSERT(g_ProcApp.SampleCount == 0);
}

static void test_penetration_profile(void)
{
    TEST("penetration_profile");
    reset_state();

    /* Linear: force = 1000 * depth, 50 samples from 0 to 0.245 m */
    inject_linear(50, 1000.0f, 0.005f);

    LRMH_ProcPhaseResult_t result;
    LRMH_PROC_RunPhase(LRMH_PHASE_BASELINE_PEN, &result);

    ASSERT(result.quality == LRMH_QUALITY_VALID);
    ASSERT(result.sample_count == 50);
    /* Max depth = 0.005 * 49 = 0.245 m */
    ASSERT_NEAR(result.max_sinkage_m, 0.245, 0.001);
    /* Max force = 1000 * 0.245 = 245 N */
    ASSERT_NEAR(result.max_force_n, 245.0, 0.5);
}

static void test_stiffness_linear_regression(void)
{
    TEST("stiffness_linear_regression");
    reset_state();

    /* Pure linear: slope = 2000 N/m */
    inject_linear(50, 2000.0f, 0.005f);

    LRMH_ProcPhaseResult_t result;
    LRMH_PROC_RunPhase(LRMH_PHASE_BASELINE_PEN, &result);

    /* Linear regression should recover slope = 2000 ± noise */
    ASSERT_NEAR(result.stiffness_proxy_npm, 2000.0, 1.0);
}

static void test_layer_break_detection(void)
{
    TEST("layer_break_detection");
    reset_state();

    /* Two-layer profile: slope=500 from 0 to 0.05 m, then
     * slope=5000 from 0.05 m onward. Layer break at 0.05 m. */
    inject_two_layer(80, 500.0f, 5000.0f, 0.05f, 0.002f);

    LRMH_ProcPhaseResult_t result;
    LRMH_PROC_RunPhase(LRMH_PHASE_BASELINE_PEN, &result);

    /* Layer break detector should find a break near 0.05 m.
     * The detector returns the depth of the maximum
     * second-derivative; with the abrupt slope change at 0.05 m,
     * it should find the break within a few samples of the
     * transition. */
    ASSERT(result.layer_break_depth_m > 0.04f);
    ASSERT(result.layer_break_depth_m < 0.06f);
}

static void test_no_layer_break_in_linear_data(void)
{
    TEST("no_layer_break_in_linear_data");
    reset_state();

    /* Pure linear: no layer break */
    inject_linear(50, 1000.0f, 0.005f);

    LRMH_ProcPhaseResult_t result;
    LRMH_PROC_RunPhase(LRMH_PHASE_BASELINE_PEN, &result);

    /* No break detected — depth_m field should be zero */
    ASSERT(result.layer_break_depth_m == 0.0f);
}

static void test_compaction_delta(void)
{
    TEST("compaction_delta");
    reset_state();

    /* Baseline: stiffness = 1000 N/m */
    inject_linear(50, 1000.0f, 0.005f);
    LRMH_ProcPhaseResult_t baseline;
    LRMH_PROC_RunPhase(LRMH_PHASE_BASELINE_PEN, &baseline);
    ASSERT(g_ProcApp.BaselinePenResult.sample_count == 50);

    /* Post-pen: stiffness = 1500 N/m → 50% increase */
    inject_linear(50, 1500.0f, 0.005f);
    LRMH_ProcPhaseResult_t post;
    LRMH_PROC_RunPhase(LRMH_PHASE_POST_PEN, &post);

    /* Compaction delta = (1500-1000)/1000 = 0.5 */
    ASSERT_NEAR(post.compaction_delta, 0.5, 0.01);
}

static void test_bearing_proxy_at_1cm(void)
{
    TEST("bearing_proxy_at_1cm");
    reset_state();

    /* Linear: 1000 N/m → at 1 cm = 10 N → bearing proxy = 10/500 = 0.02 */
    inject_linear(50, 1000.0f, 0.001f);  /* 1 mm steps */

    LRMH_ProcPhaseResult_t result;
    LRMH_PROC_RunPhase(LRMH_PHASE_BASELINE_PEN, &result);

    ASSERT_NEAR(result.bearing_proxy, 0.02, 0.005);
}

/* ── Buffer management ────────────────────────────────────────── */

static void test_buffer_full_transitions_to_streaming(void)
{
    TEST("buffer_full_transitions_to_streaming");
    reset_state();

    /* Fill the raw buffer exactly */
    for (uint32 i = 0; i < LRMH_PROC_BUFFER_SIZE; i++) {
        LRMH_SampleTlm_t s = make_sample(1000000 + i * 10, 100.0f, 0.001f * (float)i);
        LRMH_PROC_IngestSample(&s);
    }
    ASSERT(g_ProcApp.SampleCount == LRMH_PROC_BUFFER_SIZE);
    ASSERT(g_ProcApp.IsStreaming == false);

    /* One more sample triggers RAW → STREAMING (ADR-0015). Raw
     * buffer is preserved as the first-N anchor; new arrivals
     * route into the accumulator. */
    LRMH_SampleTlm_t overflow = make_sample(2000000, 200.0f, 1.0f);
    LRMH_PROC_IngestSample(&overflow);
    ASSERT(g_ProcApp.SampleCount == LRMH_PROC_BUFFER_SIZE);
    ASSERT(g_ProcApp.IsStreaming == true);
    ASSERT(g_ProcApp.StreamingAcc.warning_published == true);
    ASSERT(g_ProcApp.StreamingAcc.samples_in_streaming == 1);
    /* Reservoir was seeded from the raw buffer (1000 of 10000) */
    ASSERT(g_ProcApp.StreamingAcc.reservoir_count == LRMH_RESERVOIR_SIZE);
    /* Last-N ring captured the single streaming-mode arrival */
    ASSERT(g_ProcApp.StreamingAcc.last_n_count == 1);
}

static void test_streaming_compute_max_force_from_channel_stats(void)
{
    TEST("streaming_compute_max_force_from_channel_stats");
    reset_state();

    /* Fill the raw buffer with a linear ramp 0..100 N over
     * 0..0.10 m. Then push 5000 streaming-mode samples that
     * extend the ramp to 200 N at 0.20 m. The streaming-mode
     * compute must report max_force = 200 (not 100, which is
     * what RAW-only would have computed from the truncated
     * buffer). */
    for (uint32 i = 0; i < LRMH_PROC_BUFFER_SIZE; i++) {
        float depth = 0.10f * ((float)i / (float)(LRMH_PROC_BUFFER_SIZE - 1));
        float force = 100.0f * ((float)i / (float)(LRMH_PROC_BUFFER_SIZE - 1));
        LRMH_SampleTlm_t s = make_sample(1000000 + i * 100, force, depth);
        LRMH_PROC_IngestSample(&s);
    }
    ASSERT(g_ProcApp.IsStreaming == false);

    /* Push 5000 more — extends to 200 N / 0.20 m */
    for (uint32 i = 0; i < 5000; i++) {
        float t = (float)i / 5000.0f;  /* 0..1 */
        float depth = 0.10f + 0.10f * t;
        float force = 100.0f + 100.0f * t;
        LRMH_SampleTlm_t s = make_sample(2000000 + i * 100, force, depth);
        LRMH_PROC_IngestSample(&s);
    }
    ASSERT(g_ProcApp.IsStreaming == true);
    ASSERT(g_ProcApp.StreamingAcc.samples_in_streaming == 5000);

    LRMH_ProcPhaseResult_t r;
    memset(&r, 0, sizeof(r));
    LRMH_PROC_ComputePenetrationProfile(&r);
    /* Within 1% of 200 N (running stats hold the exact max) */
    ASSERT(fabsf(r.max_force_n - 199.98f) < 1.0f);
    ASSERT(fabsf(r.max_sinkage_m - 0.19998f) < 0.001f);
}

static void test_streaming_stiffness_from_reservoir(void)
{
    TEST("streaming_stiffness_from_reservoir");
    reset_state();

    /* Linear F(z) = 1000 z over 30000 samples. Buffer fills at
     * 10000, streaming kicks in for the remaining 20000. The
     * Vitter-R reservoir should preserve the linear relationship
     * and the regression should recover the slope within
     * sampling tolerance. */
    const float k = 1000.0f;  /* N/m */
    const uint32 N = 30000;
    for (uint32 i = 0; i < N; i++) {
        float depth = 0.20f * ((float)i / (float)(N - 1));  /* 0..0.20 m */
        float force = k * depth;
        LRMH_SampleTlm_t s = make_sample(1000000 + i * 100, force, depth);
        LRMH_PROC_IngestSample(&s);
    }
    ASSERT(g_ProcApp.IsStreaming == true);

    LRMH_ProcPhaseResult_t r;
    memset(&r, 0, sizeof(r));
    LRMH_PROC_ComputeStiffness(&r);
    /* Reservoir-based regression on a noiseless linear ramp
     * should be very close to the true slope. Allow 5% slack
     * for the middle-60% trim plus reservoir sampling. */
    ASSERT(fabsf(r.stiffness_proxy_npm - k) < 0.05f * k);
}

static void test_streaming_resets_on_phase_advance(void)
{
    TEST("streaming_resets_on_phase_advance");
    reset_state();

    /* Push enough samples to engage streaming */
    for (uint32 i = 0; i < LRMH_PROC_BUFFER_SIZE + 100; i++) {
        LRMH_SampleTlm_t s = make_sample(1000000 + i * 100, 50.0f, 0.001f * (float)i);
        LRMH_PROC_IngestSample(&s);
    }
    ASSERT(g_ProcApp.IsStreaming == true);

    /* Phase advance via token reset should clear the accumulator
     * and put the next phase back in RAW mode. */
    LRMH_PROC_OnTokenReset(5000000);
    ASSERT(g_ProcApp.IsStreaming == false);
    ASSERT(g_ProcApp.StreamingAcc.samples_in_streaming == 0);
    ASSERT(g_ProcApp.StreamingAcc.reservoir_count == 0);
    ASSERT(g_ProcApp.StreamingAcc.warning_published == false);
}

/* ── Token reset ──────────────────────────────────────────────── */

static void test_token_reset_updates_phase_start(void)
{
    TEST("token_reset_updates_phase_start");
    reset_state();

    LRMH_PROC_OnTokenReset(7000000);
    ASSERT(g_ProcApp.CurrentPhaseStartTime_us == 7000000);
    ASSERT(g_ProcApp.ResetObserved == true);
    ASSERT(g_ProcApp.AcceptedToken.Epoch == 0);
}

static void test_token_reset_does_not_clear_buffer(void)
{
    TEST("token_reset_does_not_clear_buffer");
    reset_state();

    inject_linear(20, 1000.0f, 0.005f);
    ASSERT(g_ProcApp.SampleCount == 20);

    /* Reset only updates phase start; buffer is RunPhase's job */
    LRMH_PROC_OnTokenReset(2000000);
    ASSERT(g_ProcApp.SampleCount == 20);
}

/* ── main ──────────────────────────────────────────────────────── */

int main(void)
{
    fprintf(stderr, "lrmh_proc standalone unit tests\n");
    fprintf(stderr, "============================================\n");

    test_initial_state();
    test_sample_ingestion();
    test_phase_start_rejection();
    test_phase_start_boundary();
    test_insufficient_samples_partial();
    test_penetration_profile();
    test_stiffness_linear_regression();
    test_layer_break_detection();
    test_no_layer_break_in_linear_data();
    test_compaction_delta();
    test_bearing_proxy_at_1cm();
    test_buffer_full_transitions_to_streaming();
    test_streaming_compute_max_force_from_channel_stats();
    test_streaming_stiffness_from_reservoir();
    test_streaming_resets_on_phase_advance();
    test_token_reset_updates_phase_start();
    test_token_reset_does_not_clear_buffer();

    fprintf(stderr, "============================================\n");
    fprintf(stderr, "  passed: %d\n", passed);
    fprintf(stderr, "  failed: %d\n", failed);
    return failed == 0 ? 0 : 1;
}
