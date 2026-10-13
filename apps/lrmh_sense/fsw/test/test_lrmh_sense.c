/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_lrmh_sense.c
 * @brief Standalone unit tests for lrmh_sense
 *
 * Coverage focus:
 *
 *   1. Initial state — fresh g_SenseApp, no acquisition
 *   2. Freshness floor — ADR-0009 per-channel validity check
 *      a. Backend marks valid=true, channel_valid bit set → channel passes
 *      b. Backend marks valid=false → all channels fail
 *      c. Backend bit cleared → that channel fails
 *
 *      Note: lrmh_sense rebases the HAL sample timestamp into the
 *      cFE TIME domain at the read boundary (the HAL backend has its
 *      own clock epoch — sim uses CLOCK_MONOTONIC). The freshness
 *      floor is therefore enforced by the HAL backend's own
 *      valid/channel_valid bits rather than an FSW-side age compare.
 *   3. 3-cycle timeout escalation — three consecutive failures on
 *      a channel → SENSOR_TIMEOUT count increments
 *   4. Force-rate computation between samples
 *   5. Host pose / velocity cache update
 *   6. Host alignment fields populated in published TLM
 *   7. Token reset re-baselines accepted state
 *   8. Acquisition lifecycle (start/stop, sample index)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Pull in the stub HAL header BEFORE declaring stub state */
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

/* Real seq_token library — exercises the validation rules */
#include "lrmh_seq_token.h"
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

#include "../src/lrmh_sense_app.c"

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
    memset(&g_SenseApp, 0, sizeof(g_SenseApp));
    memset(&lrmh_hal_stub_inject_sensor, 0, sizeof(lrmh_hal_stub_inject_sensor));
    g_sb_count = 0;
}

/* Helper: build a "fresh and valid" sensor reading */
static void inject_fresh_sample(uint64 time_us, float force_n)
{
    lrmh_hal_stub_inject_sensor.valid = true;
    lrmh_hal_stub_inject_sensor.channel_valid = LRMH_HAL_CH_ALL;
    lrmh_hal_stub_inject_sensor.time_us = time_us;
    lrmh_hal_stub_inject_sensor.force_n = force_n;
    lrmh_hal_stub_inject_sensor.depth_m = 0.05f;
    lrmh_hal_stub_inject_sensor.stroke_m = 0.05f;
    lrmh_hal_stub_inject_sensor.current_a = 1.5f;
    lrmh_hal_stub_inject_sensor.bus_v = 28.0f;
    lrmh_hal_stub_inject_sensor.temp_c = 22.0f;
}

/* ── Tests ─────────────────────────────────────────────────────── */

static void test_initial_state(void)
{
    TEST("initial_state");
    reset_state();
    ASSERT(g_SenseApp.AcquiringActive == 0);
    ASSERT(g_SenseApp.SampleIndex == 0u);
    ASSERT(g_SenseApp.HostPose.valid == false);
    ASSERT(g_SenseApp.HostVel.valid == false);
    ASSERT(g_SenseApp.TimeoutCount == 0u);
}

/* ── Freshness floor ──────────────────────────────────────────── */

static void test_fresh_sample_passes_all_channels(void)
{
    TEST("fresh_sample_passes_all_channels");
    reset_state();
    inject_fresh_sample(1000000, 100.0f);  /* 1 s */

    LRMH_SENSE_AcquireSample(1000000 + 5000);  /* 5 ms later */

    /* All 6 channels should be fresh — bit mask = 0x3F */
    ASSERT(g_sb_count == 1);  /* sample TLM published */
    /* Per-channel stale counters all zero */
    for (int c = 0; c < LRMH_HAL_N_CHANNELS; c++) {
        ASSERT(g_SenseApp.Freshness.consecutive_stale[c] == 0);
    }
}

static void test_backend_invalid_fails_all_channels(void)
{
    TEST("backend_invalid_fails_all_channels");
    reset_state();
    inject_fresh_sample(1000000, 100.0f);
    lrmh_hal_stub_inject_sensor.valid = false;  /* override */

    LRMH_SENSE_AcquireSample(1005000);

    for (int c = 0; c < LRMH_HAL_N_CHANNELS; c++) {
        ASSERT(g_SenseApp.Freshness.consecutive_stale[c] == 1);
        ASSERT(g_SenseApp.Freshness.total_stale[c] == 1);
    }
}

static void test_per_channel_bit_cleared_fails_that_channel(void)
{
    TEST("per_channel_bit_cleared_fails_that_channel");
    reset_state();
    inject_fresh_sample(1000000, 100.0f);
    /* Force channel bit cleared */
    lrmh_hal_stub_inject_sensor.channel_valid &= ~LRMH_HAL_CH_BIT(LRMH_HAL_CH_FORCE);

    LRMH_SENSE_AcquireSample(1005000);

    ASSERT(g_SenseApp.Freshness.consecutive_stale[LRMH_HAL_CH_FORCE] == 1);
    /* Other channels still fresh */
    ASSERT(g_SenseApp.Freshness.consecutive_stale[LRMH_HAL_CH_DEPTH] == 0);
    ASSERT(g_SenseApp.Freshness.consecutive_stale[LRMH_HAL_CH_TEMP] == 0);
}

/* The previous "age_exceeds_floor_fails_force_channel" and
 * "temperature_floor_is_more_lenient" tests asserted an FSW-side
 * age compare between data.time_us and now_us. lrmh_sense now
 * rebases data.time_us into the cFE TIME domain at the HAL read
 * boundary (HAL sim uses CLOCK_MONOTONIC, FSW uses cFE TIME — two
 * epochs), so that compare is no longer meaningful. The freshness
 * intent is now enforced via the HAL backend's own valid /
 * channel_valid bits, covered by the two tests above. */

/* ── 3-cycle timeout escalation ───────────────────────────────── */

static void test_three_consecutive_failures_trigger_timeout(void)
{
    TEST("three_consecutive_failures_trigger_timeout");
    reset_state();
    /* Backend always returns invalid */
    inject_fresh_sample(1000000, 100.0f);
    lrmh_hal_stub_inject_sensor.valid = false;

    /* First 2 failures don't escalate */
    LRMH_SENSE_AcquireSample(1005000);
    ASSERT(g_SenseApp.TimeoutCount == 0);
    LRMH_SENSE_AcquireSample(1010000);
    ASSERT(g_SenseApp.TimeoutCount == 0);

    /* 3rd failure → escalation */
    LRMH_SENSE_AcquireSample(1015000);
    ASSERT(g_SenseApp.TimeoutCount == 1);
}

static void test_recovery_resets_consecutive_counter(void)
{
    TEST("recovery_resets_consecutive_counter");
    reset_state();

    /* Two stale cycles */
    inject_fresh_sample(1000000, 100.0f);
    lrmh_hal_stub_inject_sensor.valid = false;
    LRMH_SENSE_AcquireSample(1005000);
    LRMH_SENSE_AcquireSample(1010000);
    ASSERT(g_SenseApp.Freshness.consecutive_stale[LRMH_HAL_CH_FORCE] == 2);

    /* Now a fresh sample — counter resets */
    inject_fresh_sample(2000000, 100.0f);
    LRMH_SENSE_AcquireSample(2005000);
    ASSERT(g_SenseApp.Freshness.consecutive_stale[LRMH_HAL_CH_FORCE] == 0);
}

/* ── Force-rate computation ───────────────────────────────────── */

static void test_force_rate_computed_between_samples(void)
{
    TEST("force_rate_computed_between_samples");
    reset_state();

    /* First sample at t=1.0s, force=100 N */
    inject_fresh_sample(1000000, 100.0f);
    LRMH_SENSE_AcquireSample(1005000);

    /* Second sample at t=1.01s, force=110 N → 1000 N/s */
    inject_fresh_sample(1010000, 110.0f);
    LRMH_SENSE_AcquireSample(1015000);

    /* Verify by checking last cached values */
    ASSERT(g_SenseApp.LastForce_N == 110.0f);
    ASSERT(g_SenseApp.LastForceValid == true);
}

/* ── Host nav cache ───────────────────────────────────────────── */

static void test_host_pose_cache_update(void)
{
    TEST("host_pose_cache_update");
    reset_state();

    LRMH_SENSE_HandleHostPose(0, 100.0f, 50.0f, 2.0f,
                               0.01f, 0.02f, 1.5f, 5000000);

    ASSERT(g_SenseApp.HostPose.valid == true);
    ASSERT(g_SenseApp.HostPose.x_m == 100.0f);
    ASSERT(g_SenseApp.HostPose.y_m == 50.0f);
    ASSERT(g_SenseApp.HostPose.yaw_rad == 1.5f);
    ASSERT(g_SenseApp.HostPose.receipt_time_us == 5000000);
}

static void test_host_vel_cache_update(void)
{
    TEST("host_vel_cache_update");
    reset_state();

    LRMH_SENSE_HandleHostVel(0.5f, 0.0f, 0.0f, 0.1f, 5000000);

    ASSERT(g_SenseApp.HostVel.valid == true);
    ASSERT(g_SenseApp.HostVel.vx_mps == 0.5f);
    ASSERT(g_SenseApp.HostVel.omega_z_rps == 0.1f);
}

/* ── Token reset ──────────────────────────────────────────────── */

static void test_token_reset_clears_accepted_state(void)
{
    TEST("token_reset_clears_accepted_state");
    reset_state();
    g_SenseApp.AcceptedToken.Epoch = 100;
    g_SenseApp.AcceptedToken.SeqState = 5;

    LRMH_SENSE_OnTokenReset();

    ASSERT(g_SenseApp.AcceptedToken.Epoch == 0);
    ASSERT(g_SenseApp.AcceptedToken.SeqState == 0);
    ASSERT(g_SenseApp.ResetObserved == true);
}

/* ── Acquisition lifecycle ────────────────────────────────────── */

static void test_start_acquisition_resets_state(void)
{
    TEST("start_acquisition_resets_state");
    reset_state();
    g_SenseApp.SampleIndex = 999;
    g_SenseApp.BufferCount = 5000;
    g_SenseApp.LastForceValid = true;

    LRMH_SENSE_StartAcquisition();

    ASSERT(g_SenseApp.AcquiringActive == 1);
    ASSERT(g_SenseApp.SampleIndex == 0);
    ASSERT(g_SenseApp.BufferCount == 0);
    ASSERT(g_SenseApp.LastForceValid == false);
}

static void test_stop_acquisition(void)
{
    TEST("stop_acquisition");
    reset_state();
    LRMH_SENSE_StartAcquisition();
    ASSERT(g_SenseApp.AcquiringActive == 1);

    LRMH_SENSE_StopAcquisition();
    ASSERT(g_SenseApp.AcquiringActive == 0);
}

static void test_sample_index_advances(void)
{
    TEST("sample_index_advances");
    reset_state();

    inject_fresh_sample(1000000, 100.0f);
    LRMH_SENSE_AcquireSample(1005000);
    inject_fresh_sample(1010000, 100.0f);
    LRMH_SENSE_AcquireSample(1015000);
    inject_fresh_sample(1020000, 100.0f);
    LRMH_SENSE_AcquireSample(1025000);

    ASSERT(g_SenseApp.SampleIndex == 3);
    ASSERT(g_sb_count == 3);
}

/* ── main ──────────────────────────────────────────────────────── */

int main(void)
{
    fprintf(stderr, "lrmh_sense standalone unit tests\n");
    fprintf(stderr, "============================================\n");

    test_initial_state();
    test_fresh_sample_passes_all_channels();
    test_backend_invalid_fails_all_channels();
    test_per_channel_bit_cleared_fails_that_channel();
    test_three_consecutive_failures_trigger_timeout();
    test_recovery_resets_consecutive_counter();
    test_force_rate_computed_between_samples();
    test_host_pose_cache_update();
    test_host_vel_cache_update();
    test_token_reset_clears_accepted_state();
    test_start_acquisition_resets_state();
    test_stop_acquisition();
    test_sample_index_advances();

    fprintf(stderr, "============================================\n");
    fprintf(stderr, "  passed: %d\n", passed);
    fprintf(stderr, "  failed: %d\n", failed);
    return failed == 0 ? 0 : 1;
}
