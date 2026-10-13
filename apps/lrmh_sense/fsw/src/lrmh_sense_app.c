/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_sense_app.c
 * @brief LRMH lrmh_sense application — implementation
 *
 * 100 Hz sensor acquisition. Reads HAL on every wakeup, validates
 * per-channel freshness per ADR-0009, stamps the resulting sample
 * with the host pose/velocity from the rover subscription cache
 * per ADR-0011, and publishes the rich sample TLM that
 * `lrmh_proc` consumes.
 *
 * The freshness floor is the headline: a sample with stale
 * channels has those channels masked off in `channel_valid`, and
 * 3 consecutive stale cycles on any channel are counted and
 * evented. lrmh_fault raises SENSOR_TIMEOUT from the sample age.
 */

#include "lrmh_sense_app.h"

#include <string.h>
#include <stddef.h>

LRMH_SENSE_AppData_t g_SenseApp;

#define LRMH_SENSE_CONSECUTIVE_TIMEOUT 3   /* ADR-0009 timeout escalation */

/* ============================================================
 * Forward declarations
 * ============================================================ */
static int32 LRMH_SENSE_Init(void);
static void  LRMH_SENSE_ProcessMsg(const CFE_SB_Buffer_t *BufPtr);
static void  LRMH_SENSE_HandleSenseCmd(uint8 fcn_code);
static uint64 LRMH_SENSE_NowUs(void);

/* ============================================================
 * Time helper
 * ============================================================ */
static uint64 LRMH_SENSE_NowUs(void)
{
    CFE_TIME_SysTime_t t = CFE_TIME_GetTime();
    uint64 us = (uint64)t.Seconds * 1000000ULL;
    us += ((uint64)t.Subseconds * 1000000ULL) >> 32;
    return us;
}

/* ============================================================
 * Channel freshness check (ADR-0009)
 *
 * Returns true if the channel is fresh per the HAL backend's
 * data.valid aggregate flag and the channel_valid bitmask. The
 * backend is the authority on per-channel staleness because the
 * HAL clock domain (sim CLOCK_MONOTONIC, hardware tick) and the
 * FSW cFE TIME domain have no cross-epoch calibration — an
 * FSW-side age compare against data.time_us would not measure
 * elapsed time. Cross-domain calibration is future work; if
 * added, restore a per-channel max-age compare here.
 *
 * Defense-in-depth comes from the 3-consecutive-stale escalation
 * in LRMH_SENSE_AcquireSample: a broken backend that pins
 * channel_valid bits will trip SENSOR_TIMEOUT within 30 ms at
 * 100 Hz.
 * ============================================================ */
bool LRMH_SENSE_ValidateChannelFreshness(const LRMH_HAL_SensorData_t *data,
                                         uint8 channel)
{
    if (!data->valid) return false;
    if ((data->channel_valid & LRMH_HAL_CH_BIT(channel)) == 0) return false;
    return true;
}

/* ============================================================
 * Acquire one sample
 * ============================================================ */
void LRMH_SENSE_AcquireSample(uint64 now_us)
{
    LRMH_HAL_SensorData_t data;
    int32 status = LRMH_HAL_ReadSensors(&data);
    if (status != LRMH_HAL_OK) {
        g_SenseApp.HalErrCount++;
        CFE_EVS_SendEvent(LRMH_SENSE_HAL_ERR_EID, CFE_EVS_EventType_ERROR,
                          "LRMH_SENSE: HAL read failed (0x%08lX)",
                          (unsigned long)status);
        return;
    }

    /* Rebase the HAL sample timestamp into the FSW (cFE TIME) clock
     * domain at the read boundary. The HAL backend (sim or flight)
     * runs on its own clock — sim uses CLOCK_MONOTONIC, hardware
     * could use a microcontroller tick — and there's no cross-epoch
     * calibration. Everything downstream (force-rate dt, sample TLM
     * publish, the lrmh_proc phase_start comparison) lives in FSW
     * time, so we collapse the two domains here. Per-channel
     * staleness is the HAL backend's responsibility, surfaced via
     * data.valid and channel_valid (see ValidateChannelFreshness). */
    data.time_us = now_us;

    /* Per-channel freshness check (ADR-0009) */
    uint16 fsw_channel_valid = 0;
    bool any_timeout = false;
    for (uint8 c = 0; c < LRMH_HAL_N_CHANNELS; c++) {
        bool fresh = LRMH_SENSE_ValidateChannelFreshness(&data, c);
        if (fresh) {
            fsw_channel_valid |= LRMH_HAL_CH_BIT(c);
            /* Only log recovery from a stale streak that mattered.
             * Single-cycle blips happen routinely from sensor noise
             * (the sim's sensor model randomly clears valid bits) and
             * don't escalate to SENSOR_TIMEOUT, so logging them
             * just floods EVS. Streaks of 2+ are still under the
             * 3-cycle escalation threshold but worth seeing. */
            if (g_SenseApp.Freshness.consecutive_stale[c] >= 2) {
                CFE_EVS_SendEvent(LRMH_SENSE_FRESH_RECOVERED_EID,
                                  CFE_EVS_EventType_INFORMATION,
                                  "LRMH_SENSE: ch %u recovered "
                                  "(was stale %u cycles)",
                                  (unsigned)c,
                                  (unsigned)g_SenseApp.Freshness.consecutive_stale[c]);
            }
            g_SenseApp.Freshness.consecutive_stale[c] = 0;
        } else {
            /* Saturating: a channel stale for seconds must not wrap. */
            if (g_SenseApp.Freshness.consecutive_stale[c] < 255) {
                g_SenseApp.Freshness.consecutive_stale[c]++;
            }
            g_SenseApp.Freshness.total_stale[c]++;
            /* Event once, on the cycle the streak reaches the floor. */
            if (g_SenseApp.Freshness.consecutive_stale[c] ==
                LRMH_SENSE_CONSECUTIVE_TIMEOUT) {
                any_timeout = true;
            }
        }
    }

    if (any_timeout) {
        g_SenseApp.TimeoutCount++;
        CFE_EVS_SendEvent(LRMH_SENSE_TIMEOUT_EID, CFE_EVS_EventType_ERROR,
                          "LRMH_SENSE: SENSOR_TIMEOUT (channels stale)");
        /* Count and event the timeout here; the count is reported in
         * HK. lrmh_fault's polled path raises SENSOR_TIMEOUT on its own
         * from the sample age (ADR-0013). */
    }

    /* Compute force-rate from consecutive samples (LC WP per
     * ADR-0017). Only meaningful when force is fresh on both
     * the previous and current sample. */
    float force_rate_nps = 0.0f;
    if ((fsw_channel_valid & LRMH_HAL_CH_BIT(LRMH_HAL_CH_FORCE)) &&
        g_SenseApp.LastForceValid) {
        uint64 dt_us = data.time_us - g_SenseApp.LastForceTime_us;
        if (dt_us > 0) {
            float dt_s = (float)dt_us / 1e6f;
            force_rate_nps = (data.force_n - g_SenseApp.LastForce_N) / dt_s;
        }
    }
    if (fsw_channel_valid & LRMH_HAL_CH_BIT(LRMH_HAL_CH_FORCE)) {
        g_SenseApp.LastForce_N    = data.force_n;
        g_SenseApp.LastForceTime_us = data.time_us;
        g_SenseApp.LastForceValid = true;
    }

    /* Samples this acquisition (reported at STOP) */
    if (g_SenseApp.BufferCount < LRMH_MAX_SAMPLES_PER_PHASE) {
        g_SenseApp.BufferCount++;
    }

    /* Publish to SB with the FSW-validated channel mask and the
     * force-rate field, plus host alignment from the cache. */
    LRMH_SampleTlm_t tlm;
    memset(&tlm, 0, sizeof(tlm));
    CFE_MSG_Init(CFE_MSG_PTR(tlm.TlmHeader),
                 CFE_SB_ValueToMsgId(LRMH_SENSE_TLM_MID), sizeof(tlm));

    tlm.sample_time_us = data.time_us;
    tlm.sample_index   = g_SenseApp.SampleIndex++;
    tlm.channel_valid  = fsw_channel_valid;
    tlm.phase          = g_SenseApp.CurrentPhase;
    tlm.force_n        = data.force_n;
    tlm.depth_m        = data.depth_m;
    tlm.stroke_m       = data.stroke_m;
    tlm.current_a      = data.current_a;
    tlm.bus_v          = data.bus_v;
    tlm.temp_c         = data.temp_c;
    tlm.force_rate_nps = force_rate_nps;

    /* Host pose alignment (ADR-0011) */
    if (g_SenseApp.HostPose.valid) {
        tlm.host_pose_valid  = 1;
        tlm.host_nav_state   = g_SenseApp.HostPose.nav_state;
        uint64 pose_age_us = (now_us > g_SenseApp.HostPose.receipt_time_us)
                             ? (now_us - g_SenseApp.HostPose.receipt_time_us)
                             : 0;
        tlm.host_pose_age_ms = (uint16)(pose_age_us / 1000ULL);
        tlm.host_x_m         = g_SenseApp.HostPose.x_m;
        tlm.host_y_m         = g_SenseApp.HostPose.y_m;
        tlm.host_z_m         = g_SenseApp.HostPose.z_m;
        tlm.host_roll_rad    = g_SenseApp.HostPose.roll_rad;
        tlm.host_pitch_rad   = g_SenseApp.HostPose.pitch_rad;
        tlm.host_yaw_rad     = g_SenseApp.HostPose.yaw_rad;
    }

    /* Host velocity alignment */
    if (g_SenseApp.HostVel.valid) {
        tlm.host_vel_valid   = 1;
        uint64 vel_age_us = (now_us > g_SenseApp.HostVel.receipt_time_us)
                            ? (now_us - g_SenseApp.HostVel.receipt_time_us)
                            : 0;
        tlm.host_vel_age_ms  = (uint16)(vel_age_us / 1000ULL);
        tlm.host_vx_mps      = g_SenseApp.HostVel.vx_mps;
        tlm.host_vy_mps      = g_SenseApp.HostVel.vy_mps;
        tlm.host_vz_mps      = g_SenseApp.HostVel.vz_mps;
        tlm.host_omega_z_rps = g_SenseApp.HostVel.omega_z_rps;
    }

    CFE_SB_TimeStampMsg(CFE_MSG_PTR(tlm.TlmHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(tlm.TlmHeader), true);
}

/* ============================================================
 * Acquisition control
 * ============================================================ */
void LRMH_SENSE_StartAcquisition(void)
{
    g_SenseApp.AcquiringActive = 1;
    g_SenseApp.SampleIndex     = 0;
    g_SenseApp.BufferCount     = 0;
    g_SenseApp.LastForceValid  = false;
    memset(&g_SenseApp.Freshness, 0, sizeof(g_SenseApp.Freshness));
    g_SenseApp.CmdCount++;
    CFE_EVS_SendEvent(LRMH_SENSE_START_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_SENSE: acquisition started");
}

void LRMH_SENSE_StopAcquisition(void)
{
    g_SenseApp.AcquiringActive = 0;
    g_SenseApp.CmdCount++;
    CFE_EVS_SendEvent(LRMH_SENSE_STOP_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_SENSE: acquisition stopped (%u samples)",
                      (unsigned)g_SenseApp.BufferCount);
}

/* ============================================================
 * Token reset
 * ============================================================ */
void LRMH_SENSE_OnTokenReset(void)
{
    memset(&g_SenseApp.AcceptedToken, 0, sizeof(g_SenseApp.AcceptedToken));
    g_SenseApp.ResetObserved = true;
    CFE_EVS_SendEvent(LRMH_SENSE_TOKEN_RESET_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_SENSE: token reset observed");
}

/* ============================================================
 * Host nav cache update (ADR-0011)
 * ============================================================ */
void LRMH_SENSE_HandleHostPose(uint8 nav_state,
                                float x, float y, float z,
                                float roll, float pitch, float yaw,
                                uint64 now_us)
{
    g_SenseApp.HostPose.valid           = true;
    g_SenseApp.HostPose.receipt_time_us = now_us;
    g_SenseApp.HostPose.nav_state       = nav_state;
    g_SenseApp.HostPose.x_m             = x;
    g_SenseApp.HostPose.y_m             = y;
    g_SenseApp.HostPose.z_m             = z;
    g_SenseApp.HostPose.roll_rad        = roll;
    g_SenseApp.HostPose.pitch_rad       = pitch;
    g_SenseApp.HostPose.yaw_rad         = yaw;
}

void LRMH_SENSE_HandleHostVel(float vx, float vy, float vz, float omega_z,
                               uint64 now_us)
{
    g_SenseApp.HostVel.valid           = true;
    g_SenseApp.HostVel.receipt_time_us = now_us;
    g_SenseApp.HostVel.vx_mps          = vx;
    g_SenseApp.HostVel.vy_mps          = vy;
    g_SenseApp.HostVel.vz_mps          = vz;
    g_SenseApp.HostVel.omega_z_rps     = omega_z;
}

/* ============================================================
 * App main
 * ============================================================ */
void LRMH_SENSE_AppMain(void)
{
    int32 Status;
    CFE_SB_Buffer_t *BufPtr = NULL;
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = LRMH_SENSE_Init();
    if (Status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog("LRMH_SENSE: Init failed\n");
        CFE_ES_ExitApp(CFE_ES_RunStatus_APP_ERROR);
        return;
    }

    CFE_EVS_SendEvent(LRMH_SENSE_INIT_INF_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_SENSE: initialized @ 100 Hz");

    while (CFE_ES_RunLoop(&RunStatus) == true) {
        /* Short timeout matches the 100 Hz wakeup rate */
        Status = CFE_SB_ReceiveBuffer(&BufPtr, g_SenseApp.CmdPipe, 10);
        if (Status == CFE_SUCCESS && BufPtr != NULL) {
            LRMH_SENSE_ProcessMsg(BufPtr);
        }
        else if (Status != CFE_SB_TIME_OUT) {
            /* A software-bus receive error is not something the app can
             * recover from in place: event it and let ES restart the app
             * (HS policy) rather than spin on a broken pipe. */
            g_SenseApp.ErrCount++;
            CFE_EVS_SendEvent(LRMH_SENSE_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "LRMH_SENSE: SB receive error 0x%08lX, exiting",
                              (unsigned long)Status);
            RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
        if (g_SenseApp.AcquiringActive) {
            LRMH_SENSE_AcquireSample(LRMH_SENSE_NowUs());
        }
    }

    CFE_ES_ExitApp(RunStatus);
}

/* ============================================================
 * Initialization
 * ============================================================ */
static int32 LRMH_SENSE_Init(void)
{
    int32 Status;
    memset(&g_SenseApp, 0, sizeof(g_SenseApp));

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) return Status;

    Status = CFE_SB_CreatePipe(&g_SenseApp.CmdPipe,
                               LRMH_SENSE_PIPE_DEPTH, "LRMH_SENSE_PIPE");
    if (Status != CFE_SUCCESS) return Status;

    /* Subscriptions */
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SENSE_CMD_MID),
                     g_SenseApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEQ_TO_SENSE_MID),
                     g_SenseApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SENSE_WAKEUP_MID),
                     g_SenseApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEQ_SAFE_MID),
                     g_SenseApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_TOKEN_RESET_MID),
                     g_SenseApp.CmdPipe);

    /* Host vehicle telemetry — ADR-0011 */
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_HOST_POSE_MID_VALUE),
                     g_SenseApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_HOST_VEL_MID_VALUE),
                     g_SenseApp.CmdPipe);

    return CFE_SUCCESS;
}

/* ============================================================
 * Message router
 * ============================================================ */
static void LRMH_SENSE_HandleSenseCmd(uint8 fcn_code)
{
    switch (fcn_code) {
    case LRMH_SENSE_NOOP_CC:
        g_SenseApp.CmdCount++;
        CFE_EVS_SendEvent(LRMH_SENSE_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "LRMH_SENSE: NOOP Cmd=%u Err=%u acquiring=%u",
                          (unsigned)g_SenseApp.CmdCount, (unsigned)g_SenseApp.ErrCount,
                          (unsigned)g_SenseApp.AcquiringActive);
        break;
    case LRMH_SENSE_RESET_CC:
        g_SenseApp.CmdCount = 0;
        g_SenseApp.ErrCount = 0;
        g_SenseApp.HalErrCount = 0;
        CFE_EVS_SendEvent(LRMH_SENSE_RESET_INF_EID, CFE_EVS_EventType_INFORMATION,
                          "LRMH_SENSE: counters reset");
        break;
    case LRMH_SENSE_START_CC: LRMH_SENSE_StartAcquisition(); break;
    case LRMH_SENSE_STOP_CC:  LRMH_SENSE_StopAcquisition();  break;
    case LRMH_SENSE_ZERO_FORCE_CC:
        CFE_EVS_SendEvent(LRMH_SENSE_ZERO_FORCE_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "LRMH_SENSE: zero force: not implemented (backend zeroes at Init)");
        break;
    case LRMH_SENSE_ZERO_DEPTH_CC:
        CFE_EVS_SendEvent(LRMH_SENSE_ZERO_DEPTH_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "LRMH_SENSE: zero depth: not implemented (backend zeroes at Init)");
        break;
    default:
        g_SenseApp.ErrCount++;
        break;
    }
}

static void LRMH_SENSE_ProcessMsg(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_SB_MsgId_t MsgId;
    CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId)) {

    case LRMH_SENSE_CMD_MID: {
        CFE_MSG_FcnCode_t fcn;
        CFE_MSG_GetFcnCode(&BufPtr->Msg, &fcn);
        LRMH_SENSE_HandleSenseCmd((uint8)fcn);
        break;
    }

    case LRMH_SEQ_TO_SENSE_MID: {
        const LRMH_SenseSeqCmd_t *cmd = (const LRMH_SenseSeqCmd_t *)BufPtr;
        LRMH_SeqToken_t incoming;
        incoming.Epoch    = LRMH_SEQ_TOKEN_UNPACK_EPOCH(cmd->SeqTokenPacked);
        incoming.SeqState = LRMH_SEQ_TOKEN_UNPACK_STATE(cmd->SeqTokenPacked);
        incoming.SeqCount = LRMH_SEQ_TOKEN_UNPACK_COUNT(cmd->SeqTokenPacked);

        CFE_MSG_FcnCode_t fcn;
        CFE_MSG_GetFcnCode(&BufPtr->Msg, &fcn);

        if (g_SenseApp.ResetObserved) {
            g_SenseApp.AcceptedToken = incoming;
            g_SenseApp.ResetObserved = false;
            LRMH_SENSE_HandleSenseCmd((uint8)fcn);
            break;
        }

        LRMH_TokenValidationResult_t r = LRMH_SeqToken_Validate(
            &incoming, &g_SenseApp.AcceptedToken, fcn == LRMH_SENSE_STOP_CC);
        if (r == LRMH_TOKEN_ACCEPT || r == LRMH_TOKEN_UNCONDITIONAL_ACCEPT) {
            LRMH_SENSE_HandleSenseCmd((uint8)fcn);
            /* Update phase tag from token state */
            g_SenseApp.CurrentPhase = incoming.SeqState;
        } else {
            CFE_EVS_SendEvent(LRMH_SENSE_STALE_CMD_EID,
                              CFE_EVS_EventType_INFORMATION,
                              "LRMH_SENSE: stale cmd dropped (rule=%d)",
                              (int)r);
        }
        break;
    }

    case LRMH_TOKEN_RESET_MID:
        LRMH_SENSE_OnTokenReset();
        break;

    case LRMH_SEQ_SAFE_MID:
        LRMH_SENSE_StopAcquisition();
        break;

    case LRMH_HOST_POSE_MID_VALUE: {
        /* Rover pose TLM — see ADR-0011. The struct shape comes
         * from the rover's aegis_nav_msg.h; we use a local view
         * with just the fields we care about. Validate the
         * message size first so a shrunken rover struct doesn't
         * read garbage at NavState's now-shifted offset. */
        struct host_pose_view {
            CFE_MSG_TelemetryHeader_t hdr;
            float X, Y, Z, Roll, Pitch, Yaw;
            uint32 CmdCount, ErrCount, HalTimeoutCount;
            uint8 NavState;
        };
        CFE_MSG_Size_t msg_size = 0;
        CFE_MSG_GetSize(&BufPtr->Msg, &msg_size);
        if (msg_size < sizeof(struct host_pose_view)) {
            g_SenseApp.ErrCount++;
            CFE_EVS_SendEvent(LRMH_SENSE_LEN_ERR_EID,
                              CFE_EVS_EventType_ERROR,
                              "LRMH_SENSE: host pose TLM too short "
                              "(%u < %u)",
                              (unsigned)msg_size,
                              (unsigned)sizeof(struct host_pose_view));
            break;
        }
        struct host_pose_view *pose = (struct host_pose_view *)BufPtr;
        LRMH_SENSE_HandleHostPose(pose->NavState,
                                   pose->X, pose->Y, pose->Z,
                                   pose->Roll, pose->Pitch, pose->Yaw,
                                   LRMH_SENSE_NowUs());
        break;
    }

    case LRMH_HOST_VEL_MID_VALUE: {
        struct host_vel_view {
            CFE_MSG_TelemetryHeader_t hdr;
            float Vx, Vy, Vz, OmegaZ;
        };
        CFE_MSG_Size_t msg_size = 0;
        CFE_MSG_GetSize(&BufPtr->Msg, &msg_size);
        if (msg_size < sizeof(struct host_vel_view)) {
            g_SenseApp.ErrCount++;
            CFE_EVS_SendEvent(LRMH_SENSE_LEN_ERR_EID,
                              CFE_EVS_EventType_ERROR,
                              "LRMH_SENSE: host vel TLM too short "
                              "(%u < %u)",
                              (unsigned)msg_size,
                              (unsigned)sizeof(struct host_vel_view));
            break;
        }
        struct host_vel_view *vel = (struct host_vel_view *)BufPtr;
        LRMH_SENSE_HandleHostVel(vel->Vx, vel->Vy, vel->Vz, vel->OmegaZ,
                                  LRMH_SENSE_NowUs());
        break;
    }

    default:
        break;
    }
}
