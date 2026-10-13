/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_sense_msg.h
 * @brief LRMH lrmh_sense command and telemetry messages
 *
 * Per ADR-0009 (sensor freshness floor) and ADR-0011 (host
 * vehicle telemetry subscription). The sample TLM is the
 * largest TLM packet in the LRMH image — ~120 bytes per
 * sample at 100 Hz.
 */

#ifndef LRMH_SENSE_MSG_H
#define LRMH_SENSE_MSG_H

#include "cfe.h"
#include "lrmh_msg.h"

/* ============================================================
 * Command codes
 * ============================================================ */
#define LRMH_SENSE_NOOP_CC           0
#define LRMH_SENSE_RESET_CC          1
#define LRMH_SENSE_START_CC          2   /* begin acquisition */
#define LRMH_SENSE_STOP_CC           3   /* end acquisition */
#define LRMH_SENSE_ZERO_FORCE_CC     4
#define LRMH_SENSE_ZERO_DEPTH_CC     5

/* ============================================================
 * Sequencer-issued sense command (carries token)
 * ============================================================ */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
    uint32  SeqTokenPacked;
} LRMH_SenseSeqCmd_t;

/* ============================================================
 * Sample telemetry (LRMH_SENSE_TLM_MID)
 *
 * Published on every successful read at 100 Hz. Carries:
 *   - Raw sensor channels with per-channel freshness mask
 *   - Computed force-rate (referenced by LC WP per ADR-0017)
 *   - Host pose/vel alignment fields per ADR-0011
 *   - Phase tag (correlates sample with current sequencer phase)
 * ============================================================ */
typedef struct {
    CFE_MSG_TelemetryHeader_t TlmHeader;

    /* ── Sample core ─────────────────────────────────────── */
    uint64  sample_time_us;          /* host monotonic, ADR-0009 */
    uint32  sample_index;            /* monotonic per phase */
    uint16  channel_valid;           /* per-channel freshness mask */
    uint8   phase;                   /* LRMH_Phase_t at acquisition */
    uint8   reserved8;

    float   force_n;
    float   depth_m;
    float   stroke_m;
    float   current_a;
    float   bus_v;
    float   temp_c;
    float   force_rate_nps;          /* dF/dt — LC watchpoint per ADR-0017 */

    /* ── Host pose alignment (ADR-0011) ─────────────────── */
    uint8   host_pose_valid;
    uint8   host_nav_state;          /* AEGIS_NAV_STATE_* */
    uint16  host_pose_age_ms;
    float   host_x_m;
    float   host_y_m;
    float   host_z_m;
    float   host_roll_rad;
    float   host_pitch_rad;
    float   host_yaw_rad;

    /* ── Host velocity alignment (ADR-0011) ─────────────── */
    uint8   host_vel_valid;
    uint8   reserved8b[1];
    uint16  host_vel_age_ms;
    float   host_vx_mps;
    float   host_vy_mps;
    float   host_vz_mps;
    float   host_omega_z_rps;
} LRMH_SampleTlm_t;

#endif /* LRMH_SENSE_MSG_H */
