/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_report_msg.h
 * @brief LRMH lrmh_report command and telemetry messages
 *
 * Per ADR-0016 (report I/O via DS): no in-FSW file writes, no
 * in-FSW JSON serialization. Site results publish on two SB MIDs
 * (binary record + human text) for the standard cFS DS app to
 * capture into rotated files. JSON moves ground-side.
 *
 * The binary `LRMH_SiteResultRecord_t` is the source of truth.
 * Ground tools render JSON from binary: the bridge for the live
 * console, scripts/lrmh_read_record.py for captured files.
 */

#ifndef LRMH_REPORT_MSG_H
#define LRMH_REPORT_MSG_H

#include "cfe.h"
#include "lrmh_msg.h"

/* ============================================================
 * Command codes
 * ============================================================ */
#define LRMH_REPORT_NOOP_CC          0
#define LRMH_REPORT_RESET_CC         1
#define LRMH_REPORT_GENERATE_CC      2
#define LRMH_REPORT_TAG_PARTIAL_CC  30   /* ground: publish the site so far as PARTIAL */

/* ============================================================
 * Sequencer-issued report trigger
 * ============================================================ */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
    uint32  SeqTokenPacked;
    uint16  SiteId;
    uint8   QualityFlag;            /* LRMH_Quality_t */
    uint8   Reserved;
    uint16  SessionId;              /* from START_SITE_TEST, ADR-0014 */
    uint16  SequenceNumber;         /* from START_SITE_TEST, ADR-0014 */
    uint16  PairedSiteId;           /* RMH-F-003 pairing, 0 = standalone */
    uint16  Reserved16;
    /* Test parameters threaded through so lrmh_report can emit them
     * in the site-result record. lrmh_seq latches them from
     * START_SITE_TEST and forwards here at end-of-test. */
    uint8   TestProfileId;
    uint8   ToolId;
    uint8   Reserved8b[6];          /* align uint64 below to 8-byte boundary */
    uint64  TestStartTime_us;       /* stamped when lrmh_seq received START */
} LRMH_ReportTriggerCmd_t;

/* ============================================================
 * Per-phase summary (embedded in LRMH_SiteResultRecord_t)
 *
 * Compact summary of one phase's outcome — what ground sees
 * without needing to read every sample. Maps to a row in the
 * human-text format's "Per-phase" section.
 * ============================================================ */
typedef struct {
    uint8   phase;                  /* LRMH_Phase_t */
    uint8   quality;                /* LRMH_Quality_t */
    uint8   reserved[2];
    uint32  sample_count;
    float   max_force_n;
    float   max_sinkage_m;
    float   stiffness_proxy_npm;
    float   bearing_proxy;
    float   layer_break_depth_m;
    float   duration_s;
} LRMH_SitePhaseSummary_t;

/* ============================================================
 * LRMH_SiteResultRecord_t — binary site result, ADR-0016
 *
 * Published on LRMH_REPORT_SITE_RECORD_MID (0x08AA). DS captures
 * this into the binary file set (1 MB / 24 h rotation). Ground
 * tools deserialize from binary. JSON is generated ground-side.
 *
 * Fixed size, 368 bytes including the CCSDS header. The first byte after the header
 * is `version` for forward-compat: future field additions
 * increment this and ground tools select the deserializer.
 * ============================================================ */
typedef struct {
    CFE_MSG_TelemetryHeader_t TelemetryHeader;

    /* ── Versioning ─────────────────────────────────────── */
    uint8    version;                /* current = 1 */
    uint8    reserved8[3];

    /* ── Identification ─────────────────────────────────── */
    uint16   session_id;
    uint16   sequence_number;
    uint16   site_id;
    uint16   paired_site_id;        /* RMH-F-003 pairing, 0 = standalone */
    uint8    test_profile_id;
    uint8    tool_id;
    uint8    outcome;                /* LRMH_Quality_t */
    uint8    storage_mode;           /* 0=RAW 1=STREAMING per ADR-0015 */

    /* ── Timing ─────────────────────────────────────────── */
    uint64   start_time_us;
    uint64   end_time_us;

    /* ── Host alignment (ADR-0011) ──────────────────────── */
    float    host_x_m, host_y_m, host_z_m;
    float    host_roll_rad, host_pitch_rad, host_yaw_rad;

    /* ── Per-phase summaries indexed by phase enum ─────── */
    LRMH_SitePhaseSummary_t phases[LRMH_MAX_PHASES_PER_TEST];
    uint8                   phase_count;
    uint8                   reserved8b[3];

    /* ── Aggregated derived science quantities ─────────── */
    float    penetration_resistance_mean_kpa;
    float    penetration_resistance_peak_kpa;
    float    load_sinkage_stiffness_kn_per_m;
    float    bearing_proxy_kpa;
    float    layer_break_depth_m;
    float    compaction_delta;
    float    variability;

    /* ── Fault state ─────────────────────────────────────── */
    uint16   fault_flags;
    uint8    recovery_tier;          /* 0=none, 1/2/3 = T1/T2/T3 */
    uint8    reserved8c;
} LRMH_SiteResultRecord_t;

/* ============================================================
 * Human-readable site result (LRMH_REPORT_SITE_HUMAN_MID)
 *
 * Plain ASCII formatted per the ADR-0016 template. DS captures
 * into the human text file set. Operators can read the file
 * directly without any decoder tool — useful as a first-pass
 * debugging artifact when something goes wrong.
 * ============================================================ */
typedef struct {
    CFE_MSG_TelemetryHeader_t TelemetryHeader;
    uint16  site_id;
    uint16  text_len;
    char    text[LRMH_MAX_REPORT_TEXT_LEN];
} LRMH_SiteResultHumanTlm_t;

#endif /* LRMH_REPORT_MSG_H */
