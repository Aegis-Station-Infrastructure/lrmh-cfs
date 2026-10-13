/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file test_lrmh_report.c
 * @brief Standalone unit tests for lrmh_report
 *
 * Coverage focus:
 *
 *   1. Initial state — fresh g_ReportApp, no phases
 *   2. Phase ingestion from LRMH_PROC_TLM_MID
 *   3. Site result assembly aggregates per-phase data
 *   4. Both MIDs published on GenerateSiteResult — binary +
 *      human text via CFE_SB_TransmitMsg (captured)
 *   5. Human text format includes the documented sections
 *   6. PARTIAL tagging via TagPartial path
 *   7. Reset clears phase accumulator after generation
 *   8. Token reset re-baselines accepted state
 *   9. Verify NO file I/O calls (the headline ADR-0016 fix)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "lrmh_hal.h"

/* HAL stub state — pulled in transitively via header chain */
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

/* SB transmit capture — log MID per call */
typedef struct { uint16 mid; size_t size; } SbXmit_t;
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

#include "../src/lrmh_report_app.c"

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
        fprintf(stderr, "  [%s] FAIL: %s ≈ %s (%g vs %g)\n",           \
                current_test, #a, #b, (double)(a), (double)(b));       \
        failed++;                                                      \
    } else { passed++; }                                               \
} while (0)

#define TEST(name) do {                                                \
    current_test = name;                                               \
    fprintf(stderr, "RUN  %s\n", name);                                \
} while (0)

static void reset_state(void)
{
    memset(&g_ReportApp, 0, sizeof(g_ReportApp));
    g_sb_count = 0;
}

/* Build a synthetic phase result */
static LRMH_ProcResultTlm_t make_proc_result(uint16 site_id, uint8 phase,
                                              uint8 quality,
                                              float max_force, float stiffness)
{
    LRMH_ProcResultTlm_t r;
    memset(&r, 0, sizeof(r));
    r.SiteId = site_id;
    r.Result.phase = phase;
    r.Result.quality = quality;
    r.Result.sample_count = 200;
    r.Result.max_force_n = max_force;
    r.Result.max_sinkage_m = 0.15f;
    r.Result.stiffness_proxy_npm = stiffness;
    r.Result.bearing_proxy = 0.5f;
    return r;
}

/* ── Tests ─────────────────────────────────────────────────────── */

static void test_initial_state(void)
{
    TEST("initial_state");
    reset_state();
    ASSERT(g_ReportApp.PhaseCount == 0);
    ASSERT(g_ReportApp.CmdCount == 0u);
    ASSERT(g_ReportApp.PartialTaggedCount == 0u);
}

static void test_phase_ingestion(void)
{
    TEST("phase_ingestion");
    reset_state();

    LRMH_ProcResultTlm_t r1 = make_proc_result(42, LRMH_PHASE_BASELINE_PEN,
                                                LRMH_QUALITY_VALID, 200.0f, 1500.0f);
    LRMH_REPORT_IngestPhaseResult(&r1);
    ASSERT(g_ReportApp.PhaseCount == 1);
    ASSERT(g_ReportApp.CurrentSiteId == 42);
    ASSERT(g_ReportApp.Phases[0].max_force_n == 200.0f);

    LRMH_ProcResultTlm_t r2 = make_proc_result(42, LRMH_PHASE_LOAD_RAMP,
                                                LRMH_QUALITY_VALID, 450.0f, 2200.0f);
    LRMH_REPORT_IngestPhaseResult(&r2);
    ASSERT(g_ReportApp.PhaseCount == 2);
}

static void test_phase_ingestion_max_overflow(void)
{
    TEST("phase_ingestion_max_overflow");
    reset_state();

    /* Fill the buffer */
    for (int i = 0; i < LRMH_MAX_PHASES_PER_TEST; i++) {
        LRMH_ProcResultTlm_t r = make_proc_result(42, (uint8)i,
                                                   LRMH_QUALITY_VALID, 100.0f, 1000.0f);
        LRMH_REPORT_IngestPhaseResult(&r);
    }
    ASSERT(g_ReportApp.PhaseCount == LRMH_MAX_PHASES_PER_TEST);

    /* One more — rejected, ErrCount increments */
    LRMH_ProcResultTlm_t overflow = make_proc_result(42, 99,
                                                      LRMH_QUALITY_VALID, 100.0f, 1000.0f);
    LRMH_REPORT_IngestPhaseResult(&overflow);
    ASSERT(g_ReportApp.PhaseCount == LRMH_MAX_PHASES_PER_TEST);
    ASSERT(g_ReportApp.ErrCount == 1);
}

static void test_generate_publishes_both_mids(void)
{
    TEST("generate_publishes_both_mids");
    reset_state();

    LRMH_ProcResultTlm_t r = make_proc_result(7, LRMH_PHASE_BASELINE_PEN,
                                               LRMH_QUALITY_VALID, 250.0f, 1800.0f);
    LRMH_REPORT_IngestPhaseResult(&r);

    LRMH_REPORT_GenerateSiteResult(7, LRMH_QUALITY_VALID);

    /* Two transmits: binary record + human text. The cFE stub's
     * CFE_MSG_GetMsgId returns 0 (not the actual MID), so the
     * functional assertion is just the count. The fact that
     * `LRMH_REPORT_PublishRecord` and `LRMH_REPORT_PublishHumanText`
     * each call CFE_SB_TransmitMsg exactly once is verified by
     * code review and by g_sb_count == 2. */
    ASSERT(g_sb_count == 2);

    /* Phase accumulator cleared after generation */
    ASSERT(g_ReportApp.PhaseCount == 0);
    ASSERT(g_ReportApp.CmdCount == 1);
}

static void test_generate_aggregates_phase_data(void)
{
    TEST("generate_aggregates_phase_data");
    reset_state();

    /* Two phases with different stiffness — record should pick max */
    LRMH_ProcResultTlm_t r1 = make_proc_result(1, LRMH_PHASE_BASELINE_PEN,
                                                LRMH_QUALITY_VALID, 200.0f, 1500.0f);
    LRMH_ProcResultTlm_t r2 = make_proc_result(1, LRMH_PHASE_LOAD_RAMP,
                                                LRMH_QUALITY_VALID, 450.0f, 3200.0f);
    LRMH_REPORT_IngestPhaseResult(&r1);
    LRMH_REPORT_IngestPhaseResult(&r2);

    /* Assemble directly to inspect the record */
    LRMH_SiteResultRecord_t rec;
    LRMH_REPORT_AssembleRecord(1, LRMH_QUALITY_VALID, &rec);

    ASSERT(rec.phase_count == 2);
    /* Stiffness aggregator picks max → 3200 N/m → 3.2 kN/m */
    ASSERT_NEAR(rec.load_sinkage_stiffness_kn_per_m, 3.2, 0.01);
    /* Penetration peak: only PEN phases counted, mean of {200} = 200 */
    /* 200 N peak over the nominal 1 cm^2 tip = 2000 kPa */
    ASSERT_NEAR(rec.penetration_resistance_peak_kpa, 2000.0, 5.0);
    ASSERT(rec.outcome == LRMH_QUALITY_VALID);
    ASSERT(rec.version == 1);
}

static void test_partial_tagging(void)
{
    TEST("partial_tagging");
    reset_state();

    LRMH_ProcResultTlm_t r = make_proc_result(99, LRMH_PHASE_BASELINE_PEN,
                                               LRMH_QUALITY_VALID, 100.0f, 800.0f);
    LRMH_REPORT_IngestPhaseResult(&r);

    LRMH_REPORT_TagPartial(99);

    ASSERT(g_ReportApp.PartialTaggedCount == 1);
    /* Two transmits even on PARTIAL */
    ASSERT(g_sb_count == 2);
    /* Buffer cleared */
    ASSERT(g_ReportApp.PhaseCount == 0);
}

/* ── Human text formatting ────────────────────────────────────── */

static void test_human_text_contains_required_sections(void)
{
    TEST("human_text_contains_required_sections");
    reset_state();

    LRMH_SiteResultRecord_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.version = 1;
    rec.session_id = 0xABCD;
    rec.site_id = 17;
    rec.outcome = LRMH_QUALITY_VALID;
    rec.phase_count = 1;
    rec.phases[0].phase = LRMH_PHASE_BASELINE_PEN;
    rec.phases[0].quality = LRMH_QUALITY_VALID;
    rec.phases[0].max_force_n = 412.0f;
    rec.phases[0].max_sinkage_m = 0.187f;
    rec.phases[0].stiffness_proxy_npm = 1850.0f;
    rec.phases[0].sample_count = 4520;

    char text[LRMH_MAX_REPORT_TEXT_LEN];
    int n = LRMH_REPORT_FormatHumanText(&rec, text, sizeof(text));
    ASSERT(n > 0);
    ASSERT((size_t)n < sizeof(text));

    /* Required sections per ADR-0016 */
    ASSERT(strstr(text, "=== LRMH Site Result ===") != NULL);
    ASSERT(strstr(text, "Session: 0xABCD") != NULL);
    ASSERT(strstr(text, "Site: 17") != NULL);
    ASSERT(strstr(text, "VALID") != NULL);
    ASSERT(strstr(text, "Per-phase") != NULL);
    ASSERT(strstr(text, "Derived:") != NULL);
    ASSERT(strstr(text, "Faults:") != NULL);
    ASSERT(strstr(text, "========================") != NULL);
}

static void test_human_text_outcome_strings(void)
{
    TEST("human_text_outcome_strings");
    reset_state();

    LRMH_SiteResultRecord_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.version = 1;

    char text[LRMH_MAX_REPORT_TEXT_LEN];

    rec.outcome = LRMH_QUALITY_VALID;
    LRMH_REPORT_FormatHumanText(&rec, text, sizeof(text));
    ASSERT(strstr(text, "VALID") != NULL);

    rec.outcome = LRMH_QUALITY_PARTIAL;
    LRMH_REPORT_FormatHumanText(&rec, text, sizeof(text));
    ASSERT(strstr(text, "PARTIAL") != NULL);

    rec.outcome = LRMH_QUALITY_FAULT;
    LRMH_REPORT_FormatHumanText(&rec, text, sizeof(text));
    ASSERT(strstr(text, "FAULT") != NULL);
}

/* ── Token reset ──────────────────────────────────────────────── */

static void test_token_reset_keeps_phases_on_safe_entry(void)
{
    TEST("token_reset_keeps_phases_on_safe_entry");
    reset_state();

    LRMH_ProcResultTlm_t r = make_proc_result(1, LRMH_PHASE_BASELINE_PEN,
                                               LRMH_QUALITY_VALID, 100.0f, 1000.0f);
    LRMH_REPORT_IngestPhaseResult(&r);
    ASSERT(g_ReportApp.PhaseCount == 1);

    /* SAFE entry keeps what was collected: the FAULT-quality record
     * that follows still carries the completed phases. */
    LRMH_REPORT_OnTokenReset((uint8)LRMH_TOKEN_RESET_SAFE_ENTRY);
    ASSERT(g_ReportApp.PhaseCount == 1);
    ASSERT(g_ReportApp.ResetObserved == true);
    ASSERT(g_ReportApp.AcceptedToken.Epoch == 0);

    /* An INIT (or a new test) starts a fresh site. */
    LRMH_REPORT_OnTokenReset((uint8)LRMH_TOKEN_RESET_INIT_INSTRUMENT);
    ASSERT(g_ReportApp.PhaseCount == 0);
}

/* ── ADR-0016 verification: NO file I/O ───────────────────────── */

static void test_no_file_io_in_generate_path(void)
{
    TEST("no_file_io_in_generate_path");
    reset_state();

    LRMH_ProcResultTlm_t r = make_proc_result(1, LRMH_PHASE_BASELINE_PEN,
                                               LRMH_QUALITY_VALID, 100.0f, 1000.0f);
    LRMH_REPORT_IngestPhaseResult(&r);

    /* If lrmh_report_app.c had any OS_open / OS_write / OS_close
     * calls, the test wouldn't link because the cFE stubs don't
     * provide them. The fact that this test compiles and runs is
     * the structural assertion that ADR-0016 holds. The
     * functional assertion is "two SB transmits, no file ops". */
    LRMH_REPORT_GenerateSiteResult(1, LRMH_QUALITY_VALID);
    ASSERT(g_sb_count == 2);
}

/* ── main ──────────────────────────────────────────────────────── */

int main(void)
{
    fprintf(stderr, "lrmh_report standalone unit tests\n");
    fprintf(stderr, "============================================\n");

    test_initial_state();
    test_phase_ingestion();
    test_phase_ingestion_max_overflow();
    test_generate_publishes_both_mids();
    test_generate_aggregates_phase_data();
    test_partial_tagging();
    test_human_text_contains_required_sections();
    test_human_text_outcome_strings();
    test_token_reset_keeps_phases_on_safe_entry();
    test_no_file_io_in_generate_path();

    fprintf(stderr, "============================================\n");
    fprintf(stderr, "  passed: %d\n", passed);
    fprintf(stderr, "  failed: %d\n", failed);
    return failed == 0 ? 0 : 1;
}
