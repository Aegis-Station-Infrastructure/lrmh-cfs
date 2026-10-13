/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_sense_app.h
 * @brief LRMH lrmh_sense application — public interface
 *
 * Per ADR-0006: 100 Hz sensor acquisition. First app to consume
 * the HAL freshness contract (ADR-0009) and the host telemetry
 * subscription (ADR-0011). Each sample published to the SB
 * carries the rover pose/velocity at acquisition time so ground
 * tools can reconstruct the test conditions.
 *
 * The freshness floor enforcement is the headline. Per channel:
 *   - HAL backend's `valid` aggregate flag is checked
 *   - HAL backend's per-channel `channel_valid` bit is checked
 *   - 3 consecutive failures on any channel are counted and evented;
 *     lrmh_fault raises SENSOR_TIMEOUT from the sample age (ADR-0013)
 *
 * Per-channel max-age compares against HAL time would require
 * cross-epoch calibration between the HAL clock domain (sim
 * CLOCK_MONOTONIC, hardware tick) and FSW cFE TIME — deferred
 * to a later revision. Today the backend's validity bits are the floor.
 */

#ifndef LRMH_SENSE_APP_H
#define LRMH_SENSE_APP_H

#include "cfe.h"
#include "lrmh_msgids.h"
#include "lrmh_msg.h"
#include "lrmh_seq_token.h"
#include "lrmh_hal.h"
#include "lrmh_platform_cfg.h"

#include "lrmh_sense_msg.h"
#include "lrmh_sense_events.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Host pose cache (ADR-0011)
 * ============================================================ */
typedef struct {
    bool      valid;
    uint64_t  receipt_time_us;
    uint8     nav_state;
    uint8     reserved[3];
    float     x_m, y_m, z_m;
    float     roll_rad, pitch_rad, yaw_rad;
} LRMH_SENSE_HostPoseCache_t;

typedef struct {
    bool      valid;
    uint64_t  receipt_time_us;
    float     vx_mps, vy_mps, vz_mps;
    float     omega_z_rps;
} LRMH_SENSE_HostVelCache_t;

/* ============================================================
 * Per-channel freshness state (ADR-0009)
 *
 * Tracks consecutive-failure counts per channel for the
 * 3-cycle timeout escalation rule.
 * ============================================================ */
typedef struct {
    uint8   consecutive_stale[LRMH_HAL_N_CHANNELS];
    uint32  total_stale[LRMH_HAL_N_CHANNELS];
} LRMH_SENSE_FreshnessState_t;

/* ============================================================
 * App data
 * ============================================================ */
typedef struct {
    CFE_SB_PipeId_t  CmdPipe;

    /* Acquisition state */
    uint8   AcquiringActive;
    uint8   CurrentPhase;            /* LRMH_Phase_t */
    uint8   Reserved8[2];
    uint16  SiteId;
    uint32  SampleIndex;
    uint32  BufferCount;

    /* Counters */
    uint32  CmdCount;
    uint32  ErrCount;
    uint32  HalErrCount;
    uint32  TimeoutCount;            /* SENSOR_TIMEOUT escalations */

    /* Receiver token state */
    LRMH_SeqToken_t AcceptedToken;
    bool            ResetObserved;

    /* Last sample force value (for force-rate computation) */
    float           LastForce_N;
    uint64          LastForceTime_us;
    bool            LastForceValid;

    /* Freshness tracking */
    LRMH_SENSE_FreshnessState_t Freshness;

    /* Host nav caches */
    LRMH_SENSE_HostPoseCache_t  HostPose;
    LRMH_SENSE_HostVelCache_t   HostVel;
} LRMH_SENSE_AppData_t;

/* ============================================================
 * Public entry point
 * ============================================================ */
void LRMH_SENSE_AppMain(void);

/* ============================================================
 * Test-exposed internal API
 * ============================================================ */
extern LRMH_SENSE_AppData_t g_SenseApp;

void LRMH_SENSE_AcquireSample(uint64 now_us);
bool LRMH_SENSE_ValidateChannelFreshness(const LRMH_HAL_SensorData_t *data,
                                         uint8 channel);
void LRMH_SENSE_OnTokenReset(void);
void LRMH_SENSE_StartAcquisition(void);
void LRMH_SENSE_StopAcquisition(void);
void LRMH_SENSE_HandleHostPose(uint8 nav_state,
                                float x, float y, float z,
                                float roll, float pitch, float yaw,
                                uint64 now_us);
void LRMH_SENSE_HandleHostVel(float vx, float vy, float vz, float omega_z,
                               uint64 now_us);

#ifdef __cplusplus
}
#endif

#endif /* LRMH_SENSE_APP_H */
