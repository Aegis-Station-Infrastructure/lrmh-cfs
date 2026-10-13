/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_proc_msg.h
 * @brief LRMH lrmh_proc command and telemetry messages
 *
 * Per ADRs 0002 (regolith opaque — signal-derived only),
 * 0009 (phase-start sample rejection), and 0015 (sample buffer
 * overflow / streaming accumulator interface).
 */

#ifndef LRMH_PROC_MSG_H
#define LRMH_PROC_MSG_H

#include "cfe.h"
#include "lrmh_msg.h"

/* ============================================================
 * Command codes
 * ============================================================ */
#define LRMH_PROC_NOOP_CC          0
#define LRMH_PROC_RESET_CC         1
#define LRMH_PROC_RUN_PHASE_CC     2
#define LRMH_PROC_PHASE_START_CC   3   /* start of a measured segment: clear the buffer, stamp phase start */

/* ============================================================
 * Sequencer-issued processing trigger
 *
 * Sent by lrmh_seq with LRMH_PROC_PHASE_START_CC when a measured
 * segment begins (the buffer is cleared and the phase start time
 * stamped) and with LRMH_PROC_RUN_PHASE_CC when it completes
 * (lrmh_proc runs the engineering quantity computation).
 * ============================================================ */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
    uint32  SeqTokenPacked;
    uint16  SiteId;
    uint8   PhaseComplete;          /* LRMH_Phase_t */
    uint8   Reserved;
    uint32  SampleCount;
} LRMH_ProcTriggerCmd_t;

/* ============================================================
 * Per-phase result struct
 *
 * Internal to lrmh_proc; copied into LRMH_ProcResultTlm_t for
 * publication and into LRMH_SiteResultRecord_t (ADR-0016) by
 * lrmh_report.
 * ============================================================ */
typedef struct {
    float   max_force_n;
    float   max_sinkage_m;
    float   stiffness_proxy_npm;     /* N/m, from linear regression */
    float   bearing_proxy;           /* dimensionless ratio */
    float   layer_break_depth_m;     /* 0 if no break detected */
    float   compaction_delta;        /* dimensionless */
    float   variability_index;       /* dimensionless */
    float   duration_s;              /* phase wallclock duration */
    uint32  sample_count;
    uint8   phase;                   /* LRMH_Phase_t */
    uint8   quality;                 /* LRMH_Quality_t */
    uint8   storage_mode;            /* 0=RAW, 1=STREAMING (ADR-0015) */
    uint8   reserved;
} LRMH_ProcPhaseResult_t;

/* ============================================================
 * Phase result telemetry (LRMH_PROC_TLM_MID)
 * ============================================================ */
typedef struct {
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint16                    SiteId;
    uint16                    Reserved16;
    LRMH_ProcPhaseResult_t    Result;
} LRMH_ProcResultTlm_t;

/* ============================================================
 * Phase input interface — per ADR-0015
 *
 * lrmh_sense (or eventually a streaming accumulator) hands a
 * phase's worth of samples to lrmh_proc via this interface.
 *
 * Mode 1 (RAW):       raw_samples + raw_count populated
 * Mode 2 (STREAMING): is_streaming=true, reservoir/histogram/
 *                     first_n/last_n populated
 *
 * Only the RAW path is implemented. The streaming accumulator
 * path is reserved for when lrmh_sense grows the accumulator.
 * ============================================================ */
#define LRMH_PROC_RAW_SAMPLE_CHANNELS 6   /* matches LRMH_HAL_N_CHANNELS */

typedef struct {
    uint64   time_us;
    float    force_n;
    float    depth_m;
    float    stroke_m;
    float    current_a;
    uint8    phase;                  /* LRMH_Phase_t */
    uint8    reserved[3];
} LRMH_ProcRawSample_t;

/* ============================================================
 * RAW → STREAMING transition warning TLM
 * (LRMH_PROC_PHASE_EXTENDED_TLM_MID, ADR-0015)
 *
 * Published once when the proc raw sample buffer fills and the
 * phase transitions to streaming-mode accumulation. Warns ground
 * that the published per-phase result for this phase is derived
 * from a Vitter-R reservoir + histogram rather than the full raw
 * sample stream. The instrument continues normally; this is a
 * notice, not a fault.
 * ============================================================ */
typedef struct {
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint16   SiteId;
    uint8    PhaseId;                 /* LRMH_Phase_t at the moment of overflow */
    uint8    Reason;                  /* only BUFFER_FULL is defined */
    uint32   Reserved32a;             /* explicit padding to the 8-byte field below */
    uint64   TransitionTime_us;       /* cFE TIME at transition */
    uint32   RawSamplesAtTransition;  /* should equal LRMH_PROC_BUFFER_SIZE */
    uint32   Reserved32;
} LRMH_PhaseExtendedTlm_t;

#define LRMH_PHASE_EXTENDED_REASON_BUFFER_FULL  1

#endif /* LRMH_PROC_MSG_H */
