/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_proc_app.h
 * @brief LRMH lrmh_proc application — public interface
 *
 * Per ADR-0002 (regolith opaque): signal-derived engineering
 * quantities only. No soil model. The signal processing
 * functions consume the raw sample stream and produce:
 *
 *   - Penetration resistance vs depth (max + curve)
 *   - Load-sinkage curve (bearing proxy)
 *   - Layer break depth via second-derivative inflection
 *   - Stiffness proxy via linear regression
 *   - Compaction delta vs baseline phase
 *   - Variability index across samples
 *
 * Per ADR-0009 (phase-start sample rejection): samples whose
 * acquisition timestamp is older than the current phase start
 * time are rejected.
 */

#ifndef LRMH_PROC_APP_H
#define LRMH_PROC_APP_H

#include "cfe.h"
#include "lrmh_msgids.h"
#include "lrmh_msg.h"
#include "lrmh_seq_token.h"
#include "lrmh_platform_cfg.h"

/* Bring in the lrmh_sense sample TLM shape so the proc app can
 * subscribe to it. This is a deliberate cross-app dependency:
 * lrmh_proc consumes lrmh_sense's published sample TLM. */
#include "lrmh_sense_msg.h"

#include "lrmh_proc_msg.h"
#include "lrmh_proc_events.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * App data
 * ============================================================ */
#define LRMH_PROC_BUFFER_SIZE  LRMH_MAX_SAMPLES_PER_PHASE

/* ── ADR-0015 streaming accumulator ─────────────────────────
 * Engaged when the raw sample buffer fills. Maintains per-channel
 * running statistics, a Vitter-R reservoir, a last-N raw ring
 * (the first 10000 raw samples in g_ProcApp.Samples are the
 * first-N anchor by themselves), and a 2D force-vs-depth
 * histogram.
 *
 * Storage: ~52 KB. Lazily zeroed on transition; reset on phase
 * boundary in LRMH_PROC_OnTokenReset.
 * ============================================================ */
#define LRMH_PROC_HIST_DEPTH_BINS  LRMH_HIST_DEPTH_BINS
#define LRMH_PROC_HIST_FORCE_BINS  LRMH_HIST_FORCE_BINS

typedef struct {
    uint32  count;
    double  sum;
    double  sum_sq;
    float   min;
    float   max;
    uint64  min_time_us;
    uint64  max_time_us;
} LRMH_ProcChannelStats_t;

/* Channel indices for the running stats array. Match the order
 * of fields actually compared in the compute helpers; not the
 * full LRMH_HAL_CH_* set since proc only ingests force / depth /
 * stroke / current. */
enum {
    LRMH_PROC_CH_FORCE   = 0,
    LRMH_PROC_CH_DEPTH   = 1,
    LRMH_PROC_CH_STROKE  = 2,
    LRMH_PROC_CH_CURRENT = 3,
    LRMH_PROC_N_CHANNELS = 4
};

typedef struct {
    /* Per-channel running statistics */
    LRMH_ProcChannelStats_t channels[LRMH_PROC_N_CHANNELS];

    /* Vitter-R reservoir of raw samples */
    LRMH_ProcRawSample_t reservoir[LRMH_RESERVOIR_SIZE];
    uint32 reservoir_count;
    uint32 total_samples_seen;

    /* Last-N raw ring buffer — final 500 samples of the phase */
    LRMH_ProcRawSample_t last_n_ring[LRMH_PROVENANCE_FIRST_LAST_N];
    uint32 last_n_head;
    uint32 last_n_count;

    /* Force-vs-depth histogram */
    float  histogram[LRMH_PROC_HIST_DEPTH_BINS][LRMH_PROC_HIST_FORCE_BINS];
    float  depth_bin_width;
    float  force_bin_width;

    /* Metadata */
    uint64 streaming_start_time_us;
    uint32 samples_in_streaming;
    bool   warning_published;
} LRMH_ProcStreamingAccumulator_t;

typedef struct {
    CFE_SB_PipeId_t  CmdPipe;

    /* Per-phase sample buffer (RAW mode per ADR-0015) */
    LRMH_ProcRawSample_t  Samples[LRMH_PROC_BUFFER_SIZE];
    uint32                SampleCount;

    /* Streaming accumulator (engaged on RAW buffer overflow) */
    LRMH_ProcStreamingAccumulator_t StreamingAcc;
    bool                            IsStreaming;

    /* Phase-start sample rejection (ADR-0009) */
    uint64                CurrentPhaseStartTime_us;
    uint32                StaleSampleCount;

    /* Per-phase results — kept across phases for compaction
     * delta and site-level aggregation */
    LRMH_ProcPhaseResult_t  BaselinePenResult;
    LRMH_ProcPhaseResult_t  LoadRampResult;
    LRMH_ProcPhaseResult_t  RepeatLoadResult;
    LRMH_ProcPhaseResult_t  PostPenResult;

    uint16                CurrentSiteId;
    uint8                 CurrentPhase;       /* LRMH_Phase_t of latest accepted sample */
    uint8                 Reserved8;

    /* Counters */
    uint32                CmdCount;
    uint32                ErrCount;

    /* Receiver token state */
    LRMH_SeqToken_t       AcceptedToken;
    bool                  ResetObserved;
} LRMH_PROC_AppData_t;

/* ============================================================
 * Public entry point
 * ============================================================ */
void LRMH_PROC_AppMain(void);

/* ============================================================
 * Test-exposed internal API
 * ============================================================ */
extern LRMH_PROC_AppData_t g_ProcApp;

void LRMH_PROC_IngestSample(const LRMH_SampleTlm_t *sample);
void LRMH_PROC_RunPhase(uint8 phase, LRMH_ProcPhaseResult_t *result_out);
void LRMH_PROC_OnTokenReset(uint64 phase_start_time_us);
void LRMH_PROC_OnSegmentStart(uint8 phase);

/* Streaming accumulator (ADR-0015) — exposed for unit testing */
void LRMH_PROC_TransitionToStreaming(uint64 now_us);
void LRMH_PROC_AppendStreaming(const LRMH_ProcRawSample_t *sample);
void LRMH_PROC_ResetStreamingAccumulator(void);

/* Signal processing helpers (exposed for unit testing) */
void LRMH_PROC_ComputePenetrationProfile(LRMH_ProcPhaseResult_t *r);
void LRMH_PROC_ComputeLoadSinkage(LRMH_ProcPhaseResult_t *r);
void LRMH_PROC_DetectLayerBreak(LRMH_ProcPhaseResult_t *r);
void LRMH_PROC_ComputeStiffness(LRMH_ProcPhaseResult_t *r);

#ifdef __cplusplus
}
#endif

#endif /* LRMH_PROC_APP_H */
