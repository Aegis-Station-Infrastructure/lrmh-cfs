/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_lrmh_hk.c
 * @brief Standalone unit tests for lrmh_hk
 *
 * Coverage focus:
 *
 *   1. Initial state — fresh g_HkApp, no caches valid
 *   2. Cache update from each subscribed TLM
 *   3. Latest value wins on multiple updates
 *   4. HK packet publication assembles cache contents
 *   5. HK cycle counter advances on publication
 *   6. Aggregated counters sum across apps
 *   7. NO HAL calls (structural — links cleanly without LRMH_IF_*)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "lrmh_hal.h"

/* HAL stub state — pulled in transitively via the per-app
 * msg headers (lrmh_ctrl_msg.h → lrmh_hal.h chain). */
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

#include "../src/lrmh_hk_app.c"

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
    memset(&g_HkApp, 0, sizeof(g_HkApp));
    g_sb_count = 0;
}

/* ── Tests ─────────────────────────────────────────────────────── */

static void test_initial_state(void)
{
    TEST("initial_state");
    reset_state();
    ASSERT(g_HkApp.SeqCache.valid == false);
    ASSERT(g_HkApp.CtrlCache.valid == false);
    ASSERT(g_HkApp.SenseCache.valid == false);
    ASSERT(g_HkApp.HkCycleCount == 0u);
}

static void test_seq_cache_update(void)
{
    TEST("seq_cache_update");
    reset_state();

    LRMH_SeqStatusTlm_t tlm;
    memset(&tlm, 0, sizeof(tlm));
    tlm.Phase = LRMH_PHASE_LOAD_RAMP;
    tlm.QualityFlag = LRMH_QUALITY_VALID;
    tlm.SiteId = 17;
    tlm.SessionId = 0xABCD;
    tlm.Epoch = 5;
    tlm.T1AttemptsCurrentPhase = 1;
    tlm.CmdCount = 42;

    LRMH_HK_OnSeqStatus(&tlm);
    ASSERT(g_HkApp.SeqCache.valid == true);
    ASSERT(g_HkApp.SeqCache.phase == LRMH_PHASE_LOAD_RAMP);
    ASSERT(g_HkApp.SeqCache.site_id == 17);
    ASSERT(g_HkApp.SeqCache.session_id == 0xABCD);
    ASSERT(g_HkApp.SeqCache.epoch == 5);
    ASSERT(g_HkApp.SeqCache.t1_attempts == 1);
    ASSERT(g_HkApp.SeqCache.cmd_count == 42);
}

static void test_ctrl_cache_update(void)
{
    TEST("ctrl_cache_update");
    reset_state();

    LRMH_CtrlStatusTlm_t tlm;
    memset(&tlm, 0, sizeof(tlm));
    tlm.ActiveMode = LRMH_CTRL_MODE_FORCE;
    tlm.CommandPending = 1;
    tlm.HalState = LRMH_HAL_STATE_HOLDING;
    tlm.HalFlags = LRMH_HAL_FLAG_FORCE_REACHED;
    tlm.StaleCount = 3;
    tlm.CmdCount = 100;
    tlm.CurrentStroke_m = 0.142f;
    tlm.LastCommandedForce_N = 425.0f;

    LRMH_HK_OnCtrlStatus(&tlm);
    ASSERT(g_HkApp.CtrlCache.valid == true);
    ASSERT(g_HkApp.CtrlCache.active_mode == LRMH_CTRL_MODE_FORCE);
    ASSERT(g_HkApp.CtrlCache.hal_state == LRMH_HAL_STATE_HOLDING);
    ASSERT(g_HkApp.CtrlCache.stale_count == 3);
    ASSERT(g_HkApp.CtrlCache.stroke_m == 0.142f);
    ASSERT(g_HkApp.CtrlCache.last_force_n == 425.0f);
}

static void test_fault_cache_update_and_publish(void)
{
    TEST("fault_cache_update_and_publish");
    reset_state();

    LRMH_FaultTlm_t fault;
    memset(&fault, 0, sizeof(fault));
    fault.FaultFlags    = LRMH_FAULT_FLAG_OVERCURRENT |
                          LRMH_FAULT_FLAG_FORCE_RATE;
    fault.FaultActive   = 1;
    fault.RecoveryTier  = LRMH_TIER_T2_SKIP;
    fault.TimeoutCount  = 7;

    LRMH_HK_OnFaultStatus(&fault);

    /* Cache fields populated directly on the app data (no
     * sub-struct here — the publish path reads them as-is) */
    ASSERT(g_HkApp.FaultFlags ==
           (LRMH_FAULT_FLAG_OVERCURRENT | LRMH_FAULT_FLAG_FORCE_RATE));
    ASSERT(g_HkApp.FaultActive == 1);
    ASSERT(g_HkApp.RecoveryTier == LRMH_TIER_T2_SKIP);
    ASSERT(g_HkApp.TimeoutCount == 7);

    /* Publish — the HK packet should carry the fault state through
     * to the ground-visible TLM. This is the regression we wanted
     * to lock in: before the wiring, FaultFlags was published as
     * zero even when faults were firing. */
    LRMH_HK_PublishHk();
    ASSERT(g_HkApp.HkPacket.FaultFlags ==
           (LRMH_FAULT_FLAG_OVERCURRENT | LRMH_FAULT_FLAG_FORCE_RATE));
    ASSERT(g_HkApp.HkPacket.FaultActive == 1);
    ASSERT(g_HkApp.HkPacket.RecoveryTier == LRMH_TIER_T2_SKIP);
    ASSERT(g_HkApp.HkPacket.TimeoutCount == 7);
}

static void test_sense_cache_update(void)
{
    TEST("sense_cache_update");
    reset_state();

    LRMH_SampleTlm_t tlm;
    memset(&tlm, 0, sizeof(tlm));
    tlm.channel_valid = 0x3F;
    tlm.force_n = 250.0f;
    tlm.depth_m = 0.075f;
    tlm.current_a = 2.1f;
    tlm.bus_v = 28.5f;
    tlm.temp_c = 23.0f;
    tlm.force_rate_nps = 350.0f;

    LRMH_HK_OnSenseSample(&tlm);
    ASSERT(g_HkApp.SenseCache.valid == true);
    ASSERT(g_HkApp.SenseCache.force_n == 250.0f);
    ASSERT(g_HkApp.SenseCache.bus_v == 28.5f);
    ASSERT(g_HkApp.SenseCache.temp_c == 23.0f);
    ASSERT(g_HkApp.SenseCache.force_rate_nps == 350.0f);
}

static void test_latest_value_wins(void)
{
    TEST("latest_value_wins");
    reset_state();

    LRMH_SampleTlm_t s1;
    memset(&s1, 0, sizeof(s1));
    s1.force_n = 100.0f;
    LRMH_HK_OnSenseSample(&s1);
    ASSERT(g_HkApp.SenseCache.force_n == 100.0f);

    LRMH_SampleTlm_t s2;
    memset(&s2, 0, sizeof(s2));
    s2.force_n = 200.0f;
    LRMH_HK_OnSenseSample(&s2);
    ASSERT(g_HkApp.SenseCache.force_n == 200.0f);
}

static void test_publish_assembles_packet(void)
{
    TEST("publish_assembles_packet");
    reset_state();

    /* Populate all three caches */
    LRMH_SeqStatusTlm_t seq;
    memset(&seq, 0, sizeof(seq));
    seq.Phase = LRMH_PHASE_BASELINE_PEN;
    seq.SiteId = 5;
    seq.Epoch = 3;
    seq.CmdCount = 10;
    LRMH_HK_OnSeqStatus(&seq);

    LRMH_CtrlStatusTlm_t ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.ActiveMode = LRMH_CTRL_MODE_POSITION;
    ctrl.CurrentStroke_m = 0.05f;
    ctrl.CmdCount = 8;
    LRMH_HK_OnCtrlStatus(&ctrl);

    LRMH_SampleTlm_t sense;
    memset(&sense, 0, sizeof(sense));
    sense.force_n = 175.0f;
    sense.bus_v = 28.0f;
    LRMH_HK_OnSenseSample(&sense);

    LRMH_HK_PublishHk();

    /* HK packet should reflect every cache */
    ASSERT(g_HkApp.HkPacket.SeqPhase == LRMH_PHASE_BASELINE_PEN);
    ASSERT(g_HkApp.HkPacket.CurrentSiteId == 5);
    ASSERT(g_HkApp.HkPacket.Epoch == 3);
    ASSERT(g_HkApp.HkPacket.CtrlActiveMode == LRMH_CTRL_MODE_POSITION);
    ASSERT(g_HkApp.HkPacket.StrokePosition_m == 0.05f);
    ASSERT(g_HkApp.HkPacket.Force_N == 175.0f);
    ASSERT(g_HkApp.HkPacket.BusVoltage_V == 28.0f);

    /* Aggregated counters: seq.cmd + ctrl.cmd = 10 + 8 = 18 */
    ASSERT(g_HkApp.HkPacket.TotalCmdCount == 18);

    /* Cycle counter advanced */
    ASSERT(g_HkApp.HkPacket.HkCycleCount == 1);
    ASSERT(g_HkApp.HkCycleCount == 1);

    /* One SB transmit */
    ASSERT(g_sb_count == 1);
}

static void test_hk_cycle_counter_monotonic(void)
{
    TEST("hk_cycle_counter_monotonic");
    reset_state();

    LRMH_HK_PublishHk();
    ASSERT(g_HkApp.HkCycleCount == 1);
    LRMH_HK_PublishHk();
    ASSERT(g_HkApp.HkCycleCount == 2);
    LRMH_HK_PublishHk();
    ASSERT(g_HkApp.HkCycleCount == 3);
    ASSERT(g_sb_count == 3);
}

static void test_publish_with_no_caches_valid(void)
{
    TEST("publish_with_no_caches_valid");
    reset_state();

    /* No caches updated — packet fields should be zero but
     * publication should still happen */
    LRMH_HK_PublishHk();
    ASSERT(g_HkApp.HkPacket.SeqPhase == 0);
    ASSERT(g_HkApp.HkPacket.Force_N == 0.0f);
    ASSERT(g_sb_count == 1);
    ASSERT(g_HkApp.HkCycleCount == 1);
}

/* ── Structural ADR-0006 verification ─────────────────────────── */

static void test_no_hal_calls(void)
{
    TEST("no_hal_calls");
    reset_state();

    /* Run a full publish cycle */
    LRMH_HK_PublishHk();

    /* lrmh_hk should NOT touch the HAL — verify by checking
     * the HAL stub call counters are still zero */
    ASSERT(lrmh_hal_stub_init_count == 0);
    ASSERT(lrmh_hal_stub_position_count == 0);
    ASSERT(lrmh_hal_stub_force_count == 0);
    ASSERT(lrmh_hal_stub_retract_count == 0);
    ASSERT(lrmh_hal_stub_hold_count == 0);
    ASSERT(lrmh_hal_stub_estop_count == 0);
}

/* ── main ──────────────────────────────────────────────────────── */

int main(void)
{
    fprintf(stderr, "lrmh_hk standalone unit tests\n");
    fprintf(stderr, "============================================\n");

    test_initial_state();
    test_seq_cache_update();
    test_ctrl_cache_update();
    test_sense_cache_update();
    test_fault_cache_update_and_publish();
    test_latest_value_wins();
    test_publish_assembles_packet();
    test_hk_cycle_counter_monotonic();
    test_publish_with_no_caches_valid();
    test_no_hal_calls();

    fprintf(stderr, "============================================\n");
    fprintf(stderr, "  passed: %d\n", passed);
    fprintf(stderr, "  failed: %d\n", failed);
    return failed == 0 ? 0 : 1;
}
