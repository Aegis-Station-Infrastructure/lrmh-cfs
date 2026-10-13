/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_hk_msg.h
 * @brief LRMH lrmh_hk command and telemetry messages
 *
 * Per ADR-0006 (app split): lrmh_hk is the instrument-level
 * housekeeping aggregator: it subscribes to the per-app status
 * packets and caches the fields the ground needs. (The standard
 * cFS HK app with a loadable table could replace it.)
 *
 * lrmh_hk does not call the HAL; it reads
 * from `LRMH_CTRL_STATUS_MID` and `LRMH_SENSE_TLM_MID`
 * subscriptions instead.
 */

#ifndef LRMH_HK_MSG_H
#define LRMH_HK_MSG_H

#include "cfe.h"
#include "lrmh_msg.h"

/* lrmh_hk has no ground command MID: it is driven by LRMH_SEND_HK_MID
 * (from SCH_LAB) and by the status packets it subscribes to. */

/* ============================================================
 * Instrument-level housekeeping packet (LRMH_HK_TLM_MID)
 *
 * Aggregates state from every mission app. Published at 1 Hz
 * on the wakeup tick and on demand via LRMH_SEND_HK_MID.
 * ============================================================ */
typedef struct {
    CFE_MSG_TelemetryHeader_t TlmHeader;

    /* ── Mode FSM (from lrmh_cmd, ADR-0004) ─────────────── */
    uint8   InstrumentMode;          /* LRMH_InstrumentMode_t */
    uint8   ActiveTool;
    uint16  Reserved16a;

    /* ── Sequencer state (from LRMH_SEQ_STATUS_MID) ─────── */
    uint8   SeqPhase;                /* LRMH_Phase_t */
    uint8   SeqQuality;              /* LRMH_Quality_t */
    uint8   SeqWaitingForCtrl;
    uint8   T1AttemptsCurrentPhase;
    uint16  CurrentSiteId;
    uint16  CurrentSessionId;
    uint16  Epoch;                   /* current sequence-token epoch */
    uint16  Reserved16b;

    /* ── Controller state (from LRMH_CTRL_STATUS_MID) ───── */
    uint8   CtrlActiveMode;
    uint8   CtrlCommandPending;
    uint8   HalState;                /* LRMH_HAL_State_t */
    uint8   Reserved8;
    uint16  HalFlags;
    uint16  CtrlStaleCount;
    float   StrokePosition_m;
    float   LastCommandedForce_N;

    /* ── Sensor state (from LRMH_SENSE_TLM_MID) ─────────── */
    uint16  SensorChannelValid;
    uint16  Reserved16c;
    float   Force_N;
    float   Depth_m;
    float   MotorCurrent_A;
    float   BusVoltage_V;
    float   ActuatorTemp_C;
    float   ForceRate_NpS;

    /* ── Fault state (from LRMH_FAULT_TLM_MID) ──────────── */
    uint16  FaultFlags;
    uint8   FaultActive;
    uint8   RecoveryTier;            /* 0/1/2/3 */
    uint32  TimeoutCount;            /* sensor timeout escalations */
    uint32  T2EventsInWindow;        /* reserved: not populated (see LRMH_FAULT_TLM_MID) */

    /* ── Aggregated counters ────────────────────────────── */
    uint32  TotalCmdCount;
    uint32  TotalErrCount;
    uint32  TotalRejectedCount;      /* mode FSM rejections */
    uint32  HkCycleCount;             /* monotonic publish counter */
} LRMH_HkTlm_t;

#endif /* LRMH_HK_MSG_H */
