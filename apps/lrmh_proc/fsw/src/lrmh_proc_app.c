/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_proc_app.c
 * @brief LRMH lrmh_proc application — implementation
 *
 * Signal-derived engineering quantities only (ADR-0002). Per
 * ADR-0009 (phase-start sample rejection): every incoming sample
 * is checked against the current phase start time and rejected
 * if older. Per ADR-0007: sequencer triggers are token-validated.
 *
 * The signal processing (penetration profile, load sinkage, layer
 * break detection via a second difference, stiffness via linear
 * regression) is plain math on the sample buffer (ADR-0002).
 */

#include "lrmh_proc_app.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>  /* qsort for ADR-0015 streaming reservoir */

LRMH_PROC_AppData_t g_ProcApp;

/* ============================================================
 * Forward declarations
 * ============================================================ */
static int32 LRMH_PROC_Init(void);
static void  LRMH_PROC_ProcessMsg(const CFE_SB_Buffer_t *BufPtr);
static void  LRMH_PROC_PublishResult(const LRMH_ProcPhaseResult_t *result);
static void  LRMH_PROC_PublishPhaseExtended(uint64 transition_time_us);

/* ============================================================
 * Tiny xorshift32 PRNG for the Vitter-R reservoir.
 *
 * Not cryptographic. Seeded once at app init from cFE TIME and
 * reseeded on every token reset; that's enough randomness for
 * statistical sampling on a slow per-sample tick.
 * ============================================================ */
static uint32 g_ProcRngState = 0xA5A5C3C3u;

static uint32 LRMH_PROC_RandU32(void)
{
    uint32 x = g_ProcRngState;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_ProcRngState = x;
    return x;
}

static void LRMH_PROC_SeedRng(uint64 seed_us)
{
    uint32 s = (uint32)(seed_us ^ (seed_us >> 32));
    if (s == 0) s = 0xA5A5C3C3u;
    g_ProcRngState = s;
}

/* ============================================================
 * App main
 * ============================================================ */
void LRMH_PROC_AppMain(void)
{
    int32 Status;
    CFE_SB_Buffer_t *BufPtr = NULL;
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = LRMH_PROC_Init();
    if (Status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog("LRMH_PROC: Init failed\n");
        CFE_ES_ExitApp(CFE_ES_RunStatus_APP_ERROR);
        return;
    }

    CFE_EVS_SendEvent(LRMH_PROC_INIT_INF_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_PROC: initialized");

    while (CFE_ES_RunLoop(&RunStatus) == true) {
        /* 1 s timeout so HS AppMon sees the exec counter tick even
         * when no messages arrive (PEND_FOREVER would leave the
         * counter static and trip HS's 30 s threshold). */
        Status = CFE_SB_ReceiveBuffer(&BufPtr, g_ProcApp.CmdPipe, 1000);
        if (Status == CFE_SUCCESS && BufPtr != NULL) {
            LRMH_PROC_ProcessMsg(BufPtr);
        }
        else if (Status != CFE_SB_TIME_OUT) {
            /* A software-bus receive error is not something the app can
             * recover from in place: event it and let ES restart the app
             * (HS policy) rather than spin on a broken pipe. */
            g_ProcApp.ErrCount++;
            CFE_EVS_SendEvent(LRMH_PROC_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "LRMH_PROC: SB receive error 0x%08lX, exiting",
                              (unsigned long)Status);
            RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    CFE_ES_ExitApp(RunStatus);
}

/* ============================================================
 * Initialization
 * ============================================================ */
static int32 LRMH_PROC_Init(void)
{
    int32 Status;
    memset(&g_ProcApp, 0, sizeof(g_ProcApp));

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) return Status;

    Status = CFE_SB_CreatePipe(&g_ProcApp.CmdPipe,
                               LRMH_PROC_PIPE_DEPTH, "LRMH_PROC_PIPE");
    if (Status != CFE_SUCCESS) return Status;

    /* Subscriptions */
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_PROC_CMD_MID),
                     g_ProcApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEQ_TO_PROC_MID),
                     g_ProcApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SENSE_TLM_MID),
                     g_ProcApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEQ_SAFE_MID),
                     g_ProcApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_TOKEN_RESET_MID),
                     g_ProcApp.CmdPipe);

    return CFE_SUCCESS;
}

/* ============================================================
 * Token reset handler — clears state and updates phase start time
 *
 * Per ADR-0009: a token reset carries a phase-start timestamp, and
 * the sequencer's PHASE_START command (OnSegmentStart) stamps one
 * at the start of every measured segment. Samples older than it
 * are rejected as stale-from-prior-phase.
 * ============================================================ */
void LRMH_PROC_OnTokenReset(uint64 phase_start_time_us)
{
    g_ProcApp.CurrentPhaseStartTime_us = phase_start_time_us;
    memset(&g_ProcApp.AcceptedToken, 0, sizeof(g_ProcApp.AcceptedToken));
    g_ProcApp.ResetObserved = true;
    /* Reset the streaming accumulator on every phase boundary —
     * each phase starts fresh in RAW mode per ADR-0015. The raw
     * SampleCount itself is cleared by RunPhase after the result
     * is produced (the buffer may still be valid for the
     * in-progress phase). */
    LRMH_PROC_ResetStreamingAccumulator();
    LRMH_PROC_SeedRng(phase_start_time_us);
    CFE_EVS_SendEvent(LRMH_PROC_TOKEN_RESET_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_PROC: token reset, phase_start=%llu",
                      (unsigned long long)phase_start_time_us);
}

/* ============================================================
 * Segment start (from lrmh_seq, LRMH_PROC_PHASE_START_CC)
 *
 * The sequencer marks the beginning of each measured segment, after
 * any retract that precedes it. Everything buffered so far belongs
 * to motion that is not part of the phase and is discarded; the
 * phase start time is stamped here so per-phase durations and
 * ADR-0009 stale-sample rejection refer to the segment.
 * ============================================================ */
void LRMH_PROC_OnSegmentStart(uint8 phase)
{
    CFE_TIME_SysTime_t t = CFE_TIME_GetTime();
    g_ProcApp.CurrentPhaseStartTime_us =
        (uint64)t.Seconds * 1000000ULL + (((uint64)t.Subseconds * 1000000ULL) >> 32);
    g_ProcApp.CurrentPhase = phase;
    g_ProcApp.SampleCount  = 0;
    LRMH_PROC_ResetStreamingAccumulator();
    CFE_EVS_SendEvent(LRMH_PROC_SEGMENT_START_EID,
                      CFE_EVS_EventType_DEBUG,
                      "LRMH_PROC: segment start, phase %u",
                      (unsigned)phase);
}

/* ============================================================
 * Streaming accumulator (ADR-0015)
 *
 * Engaged when the raw buffer fills. The first 10000 raw samples
 * in g_ProcApp.Samples remain valid as the first-N anchor; the
 * accumulator captures everything after.
 * ============================================================ */
void LRMH_PROC_ResetStreamingAccumulator(void)
{
    LRMH_ProcStreamingAccumulator_t *acc = &g_ProcApp.StreamingAcc;
    memset(acc, 0, sizeof(*acc));
    /* Bin widths derived from the platform envelopes (ADR-0012).
     * The histogram covers 0..MaxStroke / 0..MaxForce in fixed
     * bins; sample force/depth values outside the envelope drop
     * into the edge bins or are excluded. */
    acc->depth_bin_width = LRMH_MAX_STROKE_M / (float)LRMH_PROC_HIST_DEPTH_BINS;
    acc->force_bin_width = LRMH_MAX_FORCE_N  / (float)LRMH_PROC_HIST_FORCE_BINS;
    /* Initialize per-channel min/max neutral values */
    for (int c = 0; c < LRMH_PROC_N_CHANNELS; c++) {
        acc->channels[c].min =  1e30f;
        acc->channels[c].max = -1e30f;
    }
    g_ProcApp.IsStreaming = false;
}

static void LRMH_PROC_AccumulateChannel(LRMH_ProcChannelStats_t *cs,
                                        float v, uint64 t_us)
{
    cs->count++;
    cs->sum    += (double)v;
    cs->sum_sq += (double)v * (double)v;
    if (v < cs->min) { cs->min = v; cs->min_time_us = t_us; }
    if (v > cs->max) { cs->max = v; cs->max_time_us = t_us; }
}

static void LRMH_PROC_AccumulateAllChannels(const LRMH_ProcRawSample_t *s)
{
    LRMH_ProcStreamingAccumulator_t *acc = &g_ProcApp.StreamingAcc;
    LRMH_PROC_AccumulateChannel(&acc->channels[LRMH_PROC_CH_FORCE],
                                s->force_n,   s->time_us);
    LRMH_PROC_AccumulateChannel(&acc->channels[LRMH_PROC_CH_DEPTH],
                                s->depth_m,   s->time_us);
    LRMH_PROC_AccumulateChannel(&acc->channels[LRMH_PROC_CH_STROKE],
                                s->stroke_m,  s->time_us);
    LRMH_PROC_AccumulateChannel(&acc->channels[LRMH_PROC_CH_CURRENT],
                                s->current_a, s->time_us);
}

static void LRMH_PROC_AccumulateHistogram(const LRMH_ProcRawSample_t *s)
{
    LRMH_ProcStreamingAccumulator_t *acc = &g_ProcApp.StreamingAcc;
    if (acc->depth_bin_width <= 0.0f || acc->force_bin_width <= 0.0f) return;
    int d_bin = (int)(s->depth_m / acc->depth_bin_width);
    int f_bin = (int)(s->force_n / acc->force_bin_width);
    if (d_bin < 0) d_bin = 0;
    if (d_bin >= LRMH_PROC_HIST_DEPTH_BINS) d_bin = LRMH_PROC_HIST_DEPTH_BINS - 1;
    if (f_bin < 0) f_bin = 0;
    if (f_bin >= LRMH_PROC_HIST_FORCE_BINS) f_bin = LRMH_PROC_HIST_FORCE_BINS - 1;
    acc->histogram[d_bin][f_bin] += 1.0f;
}

void LRMH_PROC_AppendStreaming(const LRMH_ProcRawSample_t *s)
{
    LRMH_ProcStreamingAccumulator_t *acc = &g_ProcApp.StreamingAcc;

    acc->total_samples_seen++;
    acc->samples_in_streaming++;

    LRMH_PROC_AccumulateAllChannels(s);
    LRMH_PROC_AccumulateHistogram(s);

    /* Vitter Algorithm R reservoir sampling */
    if (acc->reservoir_count < LRMH_RESERVOIR_SIZE) {
        acc->reservoir[acc->reservoir_count++] = *s;
    } else {
        uint32 j = LRMH_PROC_RandU32() % acc->total_samples_seen;
        if (j < LRMH_RESERVOIR_SIZE) {
            acc->reservoir[j] = *s;
        }
    }

    /* Last-N raw ring */
    acc->last_n_ring[acc->last_n_head] = *s;
    acc->last_n_head = (acc->last_n_head + 1) % LRMH_PROVENANCE_FIRST_LAST_N;
    if (acc->last_n_count < LRMH_PROVENANCE_FIRST_LAST_N) {
        acc->last_n_count++;
    }
}

void LRMH_PROC_TransitionToStreaming(uint64 now_us)
{
    LRMH_PROC_ResetStreamingAccumulator();
    LRMH_ProcStreamingAccumulator_t *acc = &g_ProcApp.StreamingAcc;

    /* Seed the channel stats and histogram with the raw buffer
     * so streaming-mode statistics include the early phase. The
     * raw buffer is preserved as the first-N provenance anchor,
     * but its contribution to channel stats / histogram is folded
     * in here so the streaming math has the full phase. */
    for (uint32 i = 0; i < g_ProcApp.SampleCount; i++) {
        const LRMH_ProcRawSample_t *s = &g_ProcApp.Samples[i];
        acc->total_samples_seen++;
        LRMH_PROC_AccumulateAllChannels(s);
        LRMH_PROC_AccumulateHistogram(s);
    }

    /* Seed the reservoir with a uniform sample of the first 1000
     * raw points so the reservoir reflects the early phase before
     * any new streaming samples arrive. */
    uint32 seed_n = (g_ProcApp.SampleCount < LRMH_RESERVOIR_SIZE)
                  ? g_ProcApp.SampleCount : LRMH_RESERVOIR_SIZE;
    for (uint32 i = 0; i < seed_n; i++) {
        acc->reservoir[i] = g_ProcApp.Samples[i];
    }
    acc->reservoir_count = seed_n;

    acc->streaming_start_time_us = now_us;
    g_ProcApp.IsStreaming = true;

    if (!acc->warning_published) {
        LRMH_PROC_PublishPhaseExtended(now_us);
        acc->warning_published = true;
        CFE_EVS_SendEvent(LRMH_PROC_BUFFER_FULL_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "LRMH_PROC: phase extended → STREAMING at %u raw samples",
                          (unsigned)g_ProcApp.SampleCount);
    }
}

static void LRMH_PROC_PublishPhaseExtended(uint64 transition_time_us)
{
    LRMH_PhaseExtendedTlm_t tlm;
    memset(&tlm, 0, sizeof(tlm));
    CFE_MSG_Init(CFE_MSG_PTR(tlm.TlmHeader),
                 CFE_SB_ValueToMsgId(LRMH_PROC_PHASE_EXTENDED_TLM_MID),
                 sizeof(tlm));
    tlm.SiteId                 = g_ProcApp.CurrentSiteId;
    tlm.PhaseId                = g_ProcApp.CurrentPhase;
    tlm.Reason                 = LRMH_PHASE_EXTENDED_REASON_BUFFER_FULL;
    tlm.TransitionTime_us      = transition_time_us;
    tlm.RawSamplesAtTransition = g_ProcApp.SampleCount;
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(tlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(tlm.TlmHeader), true);
}

/* ============================================================
 * Sample ingestion (ADR-0009 phase-start rejection)
 * ============================================================ */
void LRMH_PROC_IngestSample(const LRMH_SampleTlm_t *sample)
{
    /* ADR-0009 phase-start rejection */
    if (sample->sample_time_us < g_ProcApp.CurrentPhaseStartTime_us) {
        g_ProcApp.StaleSampleCount++;
        CFE_EVS_SendEvent(LRMH_PROC_STALE_SAMPLE_EID,
                          CFE_EVS_EventType_DEBUG,
                          "LRMH_PROC: stale sample dropped (t=%llu < phase_start=%llu)",
                          (unsigned long long)sample->sample_time_us,
                          (unsigned long long)g_ProcApp.CurrentPhaseStartTime_us);
        return;
    }

    /* ADR-0009: a sample whose force or depth channel is flagged
     * stale carries a held value; it must not enter the regression,
     * the extrema or the layer-break scan. Counted with the stale
     * samples, no event (dropouts are routine). */
    /* channel_valid bit i is HAL channel i (0 force, 1 depth, ...),
     * the same indexing as LRMH_PROC_CH_*. */
    const uint16 need = (uint16)((1u << LRMH_PROC_CH_FORCE) | (1u << LRMH_PROC_CH_DEPTH));
    if ((sample->channel_valid & need) != need) {
        g_ProcApp.StaleSampleCount++;
        return;
    }

    /* Build the raw sample once for both paths */
    LRMH_ProcRawSample_t rec;
    rec.time_us   = sample->sample_time_us;
    rec.force_n   = sample->force_n;
    rec.depth_m   = sample->depth_m;
    rec.stroke_m  = sample->stroke_m;
    rec.current_a = sample->current_a;
    rec.phase     = sample->phase;
    rec.reserved[0] = rec.reserved[1] = rec.reserved[2] = 0;

    /* Track the phase of the latest accepted sample so the
     * ADR-0015 PHASE_EXTENDED TLM published on RAW→STREAMING
     * transition can identify which phase overflowed. */
    g_ProcApp.CurrentPhase = sample->phase;

    if (g_ProcApp.IsStreaming) {
        /* ADR-0015 STREAMING mode — accumulator only */
        LRMH_PROC_AppendStreaming(&rec);
        return;
    }

    if (g_ProcApp.SampleCount >= LRMH_PROC_BUFFER_SIZE) {
        /* RAW buffer full — transition to STREAMING (ADR-0015).
         * The first 10000 samples in g_ProcApp.Samples remain
         * valid as the first-N provenance anchor; new arrivals
         * route into the streaming accumulator. */
        LRMH_PROC_TransitionToStreaming(rec.time_us);
        LRMH_PROC_AppendStreaming(&rec);
        return;
    }

    /* RAW append */
    g_ProcApp.Samples[g_ProcApp.SampleCount] = rec;
    g_ProcApp.SampleCount++;
}

/* ============================================================
 * Signal processing helpers — RAW + STREAMING (ADR-0015)
 *
 * The same math runs on either input source. RAW mode reads
 * g_ProcApp.Samples; the
 * STREAMING branch reads from the streaming accumulator (running
 * stats for max/min, Vitter-R reservoir for derivative quantities,
 * 2D histogram for the bearing-proxy lookup).
 * ============================================================ */

/* qsort comparator: sort raw samples by depth_m ascending */
static int LRMH_PROC_CompareByDepth(const void *a, const void *b)
{
    const LRMH_ProcRawSample_t *sa = (const LRMH_ProcRawSample_t *)a;
    const LRMH_ProcRawSample_t *sb = (const LRMH_ProcRawSample_t *)b;
    if (sa->depth_m < sb->depth_m) return -1;
    if (sa->depth_m > sb->depth_m) return  1;
    return 0;
}

void LRMH_PROC_ComputePenetrationProfile(LRMH_ProcPhaseResult_t *r)
{
    if (g_ProcApp.IsStreaming) {
        /* Channel running stats span the full phase (raw +
         * streaming portion) by construction — TransitionToStreaming
         * folded the raw buffer's contributions in. */
        r->max_force_n  = g_ProcApp.StreamingAcc.channels[LRMH_PROC_CH_FORCE].max;
        r->max_sinkage_m = g_ProcApp.StreamingAcc.channels[LRMH_PROC_CH_DEPTH].max;
        return;
    }

    float maxForce = 0.0f;
    float maxDepth = 0.0f;
    for (uint32 i = 0; i < g_ProcApp.SampleCount; i++) {
        if (g_ProcApp.Samples[i].force_n > maxForce) {
            maxForce = g_ProcApp.Samples[i].force_n;
        }
        if (g_ProcApp.Samples[i].depth_m > maxDepth) {
            maxDepth = g_ProcApp.Samples[i].depth_m;
        }
    }
    r->max_force_n  = maxForce;
    r->max_sinkage_m = maxDepth;
}

void LRMH_PROC_ComputeLoadSinkage(LRMH_ProcPhaseResult_t *r)
{
    if (g_ProcApp.IsStreaming) {
        /* Bearing proxy from histogram: pick the depth bin
         * containing 1 cm, take the count-weighted mean force
         * across that row's force bins, normalize to 500 N. */
        const LRMH_ProcStreamingAccumulator_t *acc = &g_ProcApp.StreamingAcc;
        if (acc->depth_bin_width <= 0.0f) return;
        int d_bin = (int)(0.01f / acc->depth_bin_width);
        if (d_bin < 0 || d_bin >= LRMH_PROC_HIST_DEPTH_BINS) return;
        double sum_count = 0.0;
        double sum_fweight = 0.0;
        for (int f_bin = 0; f_bin < LRMH_PROC_HIST_FORCE_BINS; f_bin++) {
            float n = acc->histogram[d_bin][f_bin];
            if (n <= 0.0f) continue;
            float f_center = ((float)f_bin + 0.5f) * acc->force_bin_width;
            sum_count   += n;
            sum_fweight += (double)n * (double)f_center;
        }
        float forceAt1cm = (sum_count > 0.0)
                         ? (float)(sum_fweight / sum_count) : 0.0f;
        r->bearing_proxy = forceAt1cm / LRMH_PROC_BEARING_REF_N;
        return;
    }

    if (g_ProcApp.SampleCount < 2) return;

    /* Bearing proxy: force at 1 cm depth (linear interpolation
     * between the two samples bracketing 1 cm), normalized to
     * the LRMH_PROC_BEARING_REF_N project-level reference. */
    float forceAt1cm = 0.0f;
    for (uint32 i = 1; i < g_ProcApp.SampleCount; i++) {
        if (g_ProcApp.Samples[i].depth_m >= 0.01f &&
            g_ProcApp.Samples[i-1].depth_m < 0.01f) {
            float dz = g_ProcApp.Samples[i].depth_m -
                       g_ProcApp.Samples[i-1].depth_m;
            if (dz > 1e-9f) {
                float frac = (0.01f - g_ProcApp.Samples[i-1].depth_m) / dz;
                forceAt1cm = g_ProcApp.Samples[i-1].force_n +
                             frac * (g_ProcApp.Samples[i].force_n -
                                     g_ProcApp.Samples[i-1].force_n);
            }
            break;
        }
    }
    r->bearing_proxy = forceAt1cm / LRMH_PROC_BEARING_REF_N;
}

void LRMH_PROC_DetectLayerBreak(LRMH_ProcPhaseResult_t *r)
{
    if (g_ProcApp.IsStreaming) {
        /* Sort the reservoir by depth and run the same
         * second-derivative inflection scan over the sorted
         * sample. The reservoir is frozen at phase end (no more
         * inserts), so in-place sort is safe — RunPhase resets
         * the accumulator afterward anyway. */
        LRMH_ProcStreamingAccumulator_t *acc = &g_ProcApp.StreamingAcc;
        uint32 n = acc->reservoir_count;
        if (n < 20) return;
        qsort(acc->reservoir, n, sizeof(acc->reservoir[0]),
              LRMH_PROC_CompareByDepth);
        float maxSecondDeriv = 0.0f;
        float breakDepth = 0.0f;
        uint32 window = 5;
        for (uint32 i = window; i < n - window; i++) {
            float dz1 = acc->reservoir[i].depth_m -
                        acc->reservoir[i - window].depth_m;
            float dz2 = acc->reservoir[i + window].depth_m -
                        acc->reservoir[i].depth_m;
            if (fabsf(dz1) < 1e-6f || fabsf(dz2) < 1e-6f) continue;
            float df1 = (acc->reservoir[i].force_n -
                         acc->reservoir[i - window].force_n) / dz1;
            float df2 = (acc->reservoir[i + window].force_n -
                         acc->reservoir[i].force_n) / dz2;
            float d2f = fabsf(df2 - df1) / ((dz1 + dz2) * 0.5f);
            if (d2f > maxSecondDeriv) {
                maxSecondDeriv = d2f;
                breakDepth = acc->reservoir[i].depth_m;
            }
        }
        if (maxSecondDeriv > LRMH_PROC_LAYER_BREAK_D2F_THRESHOLD) {
            r->layer_break_depth_m = breakDepth;
            CFE_EVS_SendEvent(LRMH_PROC_LAYER_BREAK_EID,
                              CFE_EVS_EventType_INFORMATION,
                              "LRMH_PROC: layer break at %.4f m (streaming)",
                              (double)breakDepth);
        }
        return;
    }

    if (g_ProcApp.SampleCount < 20) return;

    float maxSecondDeriv = 0.0f;
    float breakDepth = 0.0f;
    uint32 window = 5;

    for (uint32 i = window; i < g_ProcApp.SampleCount - window; i++) {
        float dz1 = g_ProcApp.Samples[i].depth_m -
                    g_ProcApp.Samples[i - window].depth_m;
        float dz2 = g_ProcApp.Samples[i + window].depth_m -
                    g_ProcApp.Samples[i].depth_m;
        if (fabsf(dz1) < 1e-6f || fabsf(dz2) < 1e-6f) continue;

        float df1 = (g_ProcApp.Samples[i].force_n -
                     g_ProcApp.Samples[i - window].force_n) / dz1;
        float df2 = (g_ProcApp.Samples[i + window].force_n -
                     g_ProcApp.Samples[i].force_n) / dz2;
        float d2f = fabsf(df2 - df1) / ((dz1 + dz2) * 0.5f);

        if (d2f > maxSecondDeriv) {
            maxSecondDeriv = d2f;
            breakDepth = g_ProcApp.Samples[i].depth_m;
        }
    }

    if (maxSecondDeriv > LRMH_PROC_LAYER_BREAK_D2F_THRESHOLD) {
        r->layer_break_depth_m = breakDepth;
        CFE_EVS_SendEvent(LRMH_PROC_LAYER_BREAK_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "LRMH_PROC: layer break at %.4f m",
                          (double)breakDepth);
    }
}

void LRMH_PROC_ComputeStiffness(LRMH_ProcPhaseResult_t *r)
{
    if (g_ProcApp.IsStreaming) {
        /* Linear regression on the middle 60% of the depth-sorted
         * reservoir. DetectLayerBreak above already sorted in
         * place; if it ran first the sort is reused (qsort is
         * idempotent on a sorted array). Otherwise sort here. */
        LRMH_ProcStreamingAccumulator_t *acc = &g_ProcApp.StreamingAcc;
        uint32 n_total = acc->reservoir_count;
        if (n_total < 10) return;
        qsort(acc->reservoir, n_total, sizeof(acc->reservoir[0]),
              LRMH_PROC_CompareByDepth);
        uint32 start = n_total * LRMH_PROC_REGRESSION_LOW_TENTHS  / 10;
        uint32 end   = n_total * LRMH_PROC_REGRESSION_HIGH_TENTHS / 10;
        uint32 n     = end - start;
        if (n < 5) return;
        double sumZ = 0.0, sumF = 0.0, sumZZ = 0.0, sumZF = 0.0;
        for (uint32 i = start; i < end; i++) {
            double z = (double)acc->reservoir[i].depth_m;
            double f = (double)acc->reservoir[i].force_n;
            sumZ  += z;
            sumF  += f;
            sumZZ += z * z;
            sumZF += z * f;
        }
        double denom = (double)n * sumZZ - sumZ * sumZ;
        if (fabs(denom) < 1e-12) return;
        double slope = ((double)n * sumZF - sumZ * sumF) / denom;
        r->stiffness_proxy_npm = (float)slope;
        return;
    }

    if (g_ProcApp.SampleCount < 10) return;

    /* Linear regression on the middle band of samples (default
     * 20%–80% via LRMH_PROC_REGRESSION_*_TENTHS) to avoid
     * surface-contact and hard-stop artifacts at the ends. */
    uint32 start = g_ProcApp.SampleCount * LRMH_PROC_REGRESSION_LOW_TENTHS  / 10;
    uint32 end   = g_ProcApp.SampleCount * LRMH_PROC_REGRESSION_HIGH_TENTHS / 10;
    uint32 n     = end - start;
    if (n < 5) return;

    double sumZ = 0.0, sumF = 0.0, sumZZ = 0.0, sumZF = 0.0;
    for (uint32 i = start; i < end; i++) {
        double z = (double)g_ProcApp.Samples[i].depth_m;
        double f = (double)g_ProcApp.Samples[i].force_n;
        sumZ  += z;
        sumF  += f;
        sumZZ += z * z;
        sumZF += z * f;
    }

    double denom = (double)n * sumZZ - sumZ * sumZ;
    if (fabs(denom) < 1e-12) return;

    double slope = ((double)n * sumZF - sumZ * sumF) / denom;
    r->stiffness_proxy_npm = (float)slope;
}

/* ============================================================
 * Run processing for a completed phase
 * ============================================================ */
void LRMH_PROC_RunPhase(uint8 phase, LRMH_ProcPhaseResult_t *result_out)
{
    /* Phase wallclock duration: now (cFE TIME) minus the segment
     * start stamped by the sequencer's PHASE_START command. */
    CFE_TIME_SysTime_t t = CFE_TIME_GetTime();
    uint64 now_us = (uint64)t.Seconds * 1000000ULL +
                    (((uint64)t.Subseconds * 1000000ULL) >> 32);
    float phase_duration_s = 0.0f;
    if (g_ProcApp.CurrentPhaseStartTime_us > 0 &&
        now_us > g_ProcApp.CurrentPhaseStartTime_us) {
        phase_duration_s =
            (float)(now_us - g_ProcApp.CurrentPhaseStartTime_us) / 1e6f;
    }

    LRMH_ProcPhaseResult_t result;
    memset(&result, 0, sizeof(result));
    result.phase        = phase;
    /* sample_count reports the contributing total: raw samples
     * always, plus the streaming-mode addition if engaged. */
    result.sample_count = g_ProcApp.SampleCount;
    if (g_ProcApp.IsStreaming) {
        result.sample_count += g_ProcApp.StreamingAcc.samples_in_streaming;
    }
    result.quality      = LRMH_QUALITY_VALID;
    result.duration_s   = phase_duration_s;
    result.storage_mode = g_ProcApp.IsStreaming ? 1 : 0;

    if (g_ProcApp.SampleCount < 10) {
        result.quality = LRMH_QUALITY_PARTIAL;
        CFE_EVS_SendEvent(LRMH_PROC_PHASE_PARTIAL_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "LRMH_PROC: phase %u PARTIAL (%u samples)",
                          (unsigned)phase, (unsigned)g_ProcApp.SampleCount);
    } else {
        LRMH_PROC_ComputePenetrationProfile(&result);
        LRMH_PROC_ComputeLoadSinkage(&result);
        LRMH_PROC_DetectLayerBreak(&result);
        LRMH_PROC_ComputeStiffness(&result);

        /* Compaction delta: post-pen vs baseline. ADR-0008 says
         * RMH-F-003 disturbance sensitivity quantifies this. */
        if (phase == (uint8)LRMH_PHASE_POST_PEN &&
            g_ProcApp.BaselinePenResult.sample_count > 0 &&
            g_ProcApp.BaselinePenResult.stiffness_proxy_npm > 0.0f) {
            result.compaction_delta =
                (result.stiffness_proxy_npm -
                 g_ProcApp.BaselinePenResult.stiffness_proxy_npm) /
                g_ProcApp.BaselinePenResult.stiffness_proxy_npm;
        }
    }

    /* Latch into the per-phase storage */
    switch (phase) {
    case LRMH_PHASE_BASELINE_PEN:
        g_ProcApp.BaselinePenResult = result; break;
    case LRMH_PHASE_LOAD_RAMP:
        g_ProcApp.LoadRampResult = result; break;
    case LRMH_PHASE_REPEAT_LOAD:
        g_ProcApp.RepeatLoadResult = result; break;
    case LRMH_PHASE_POST_PEN:
        g_ProcApp.PostPenResult = result; break;
    default: break;
    }

    if (result_out) *result_out = result;

    LRMH_PROC_PublishResult(&result);

    CFE_EVS_SendEvent(LRMH_PROC_PHASE_DONE_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_PROC: phase %u done MaxF=%.1f Stiff=%.1f",
                      (unsigned)phase,
                      (double)result.max_force_n,
                      (double)result.stiffness_proxy_npm);

    /* Clear buffer + streaming accumulator for the next phase */
    g_ProcApp.SampleCount = 0;
    LRMH_PROC_ResetStreamingAccumulator();
}

/* ============================================================
 * Result publication
 * ============================================================ */
static void LRMH_PROC_PublishResult(const LRMH_ProcPhaseResult_t *result)
{
    LRMH_ProcResultTlm_t tlm;
    memset(&tlm, 0, sizeof(tlm));
    CFE_MSG_Init(CFE_MSG_PTR(tlm.TlmHeader),
                 CFE_SB_ValueToMsgId(LRMH_PROC_TLM_MID), sizeof(tlm));
    tlm.SiteId = g_ProcApp.CurrentSiteId;
    tlm.Result = *result;
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(tlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(tlm.TlmHeader), true);
}

/* ============================================================
 * Message router
 * ============================================================ */
static void LRMH_PROC_ProcessMsg(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_SB_MsgId_t MsgId;
    CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId)) {

    case LRMH_PROC_CMD_MID: {
        /* Ground: NOOP and RESET only (phases are triggered by lrmh_seq). */
        CFE_MSG_FcnCode_t gfcn = 0;
        CFE_MSG_GetFcnCode(&BufPtr->Msg, &gfcn);
        if (gfcn == LRMH_PROC_NOOP_CC) {
            g_ProcApp.CmdCount++;
            CFE_EVS_SendEvent(LRMH_PROC_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "LRMH_PROC: NOOP Cmd=%u Err=%u buffered=%u",
                              (unsigned)g_ProcApp.CmdCount, (unsigned)g_ProcApp.ErrCount,
                              (unsigned)g_ProcApp.SampleCount);
        } else if (gfcn == LRMH_PROC_RESET_CC) {
            g_ProcApp.CmdCount = 0;
            g_ProcApp.ErrCount = 0;
            g_ProcApp.StaleSampleCount = 0;
            CFE_EVS_SendEvent(LRMH_PROC_RESET_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "LRMH_PROC: counters reset");
        } else {
            g_ProcApp.ErrCount++;
        }
        break;
    }

    case LRMH_SENSE_TLM_MID:
        LRMH_PROC_IngestSample((const LRMH_SampleTlm_t *)BufPtr);
        break;

    case LRMH_SEQ_TO_PROC_MID: {
        const LRMH_ProcTriggerCmd_t *cmd =
            (const LRMH_ProcTriggerCmd_t *)BufPtr;
        CFE_MSG_FcnCode_t fcn = 0;
        CFE_MSG_GetFcnCode(&BufPtr->Msg, &fcn);
        LRMH_SeqToken_t incoming;
        incoming.Epoch    = LRMH_SEQ_TOKEN_UNPACK_EPOCH(cmd->SeqTokenPacked);
        incoming.SeqState = LRMH_SEQ_TOKEN_UNPACK_STATE(cmd->SeqTokenPacked);
        incoming.SeqCount = LRMH_SEQ_TOKEN_UNPACK_COUNT(cmd->SeqTokenPacked);

        bool accept;
        if (g_ProcApp.ResetObserved) {
            g_ProcApp.AcceptedToken = incoming;
            g_ProcApp.ResetObserved = false;
            accept = true;
        } else {
            LRMH_TokenValidationResult_t r = LRMH_SeqToken_Validate(
                &incoming, &g_ProcApp.AcceptedToken, false);
            accept = (r == LRMH_TOKEN_ACCEPT || r == LRMH_TOKEN_UNCONDITIONAL_ACCEPT);
            if (!accept) {
                CFE_EVS_SendEvent(LRMH_PROC_STALE_CMD_EID,
                                  CFE_EVS_EventType_INFORMATION,
                                  "LRMH_PROC: stale cmd dropped (rule=%d)",
                                  (int)r);
            }
        }
        if (accept) {
            g_ProcApp.CurrentSiteId = cmd->SiteId;
            if (fcn == LRMH_PROC_PHASE_START_CC) {
                LRMH_PROC_OnSegmentStart(cmd->PhaseComplete);
            } else {
                LRMH_PROC_RunPhase(cmd->PhaseComplete, NULL);
            }
        }
        break;
    }

    case LRMH_TOKEN_RESET_MID: {
        const LRMH_TokenResetTlm_t *reset =
            (const LRMH_TokenResetTlm_t *)BufPtr;
        LRMH_PROC_OnTokenReset(reset->PhaseStartTime_us);
        break;
    }

    case LRMH_SEQ_SAFE_MID:
        /* Per ADR-0008: PARTIAL site result with whatever data we
         * have. The processing pass runs as soon as the sequencer
         * sends the trigger; for SAFE we just clear the buffer. */
        g_ProcApp.SampleCount = 0;
        break;

    default:
        break;
    }
}
