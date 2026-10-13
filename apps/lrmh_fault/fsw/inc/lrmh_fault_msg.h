/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_fault_msg.h
 * @brief LRMH lrmh_fault command and telemetry messages
 *
 * Per ADR-0017 (FDIR via LC + SC + HS, with lrmh_fault as thin
 * orchestrator). The 11 fault flags are 10 hardware categories plus the
 * HOST_NAV_TIMEOUT from ADR-0011.
 */

#ifndef LRMH_FAULT_MSG_H
#define LRMH_FAULT_MSG_H

#include "cfe.h"
#include "lrmh_msg.h"

/* ============================================================
 * Fault flags (bitmask) — 11 categories
 *
 * HOST_NAV_TIMEOUT (bit 0x0400) is per ADR-0011.
 *
 * Per ADR-0008, each category maps to one recovery tier; the
 * mapping is in the static `tier_table` array in lrmh_fault_app.c.
 * ============================================================ */
#define LRMH_FAULT_FLAG_STALL              (0x0001)  /* T2 */
#define LRMH_FAULT_FLAG_OVERCURRENT        (0x0002)  /* T3 */
#define LRMH_FAULT_FLAG_FORCE_SATURATION   (0x0004)  /* T2 */
#define LRMH_FAULT_FLAG_ENCODER_LOSS       (0x0008)  /* T1 */
#define LRMH_FAULT_FLAG_THERMAL            (0x0010)  /* T3 */
#define LRMH_FAULT_FLAG_FORCE_RATE         (0x0020)  /* T1 */
#define LRMH_FAULT_FLAG_HARD_STOP          (0x0040)  /* T2 */
#define LRMH_FAULT_FLAG_SENSOR_TIMEOUT     (0x0080)  /* T1 */
#define LRMH_FAULT_FLAG_DEPTH_DISAGREE     (0x0100)  /* T2 */
#define LRMH_FAULT_FLAG_BUS_VOLTAGE        (0x0200)  /* T3 */
#define LRMH_FAULT_FLAG_HOST_NAV_TIMEOUT   (0x0400)  /* T2 — ADR-0011 */

#define LRMH_FAULT_FLAG_COUNT  11

/* ============================================================
 * Command codes for LRMH_FAULT_CMD_MID
 * ============================================================ */
#define LRMH_FAULT_NOOP_CC           0
#define LRMH_FAULT_RESET_CC          1
#define LRMH_FAULT_ENABLE_CC         2
#define LRMH_FAULT_DISABLE_CC        3
#define LRMH_FAULT_LC_NOTIFY_CC     10  /* from LC actionpoint via SC RTS */
#define LRMH_FAULT_PHASE_ADVANCE_CC 11  /* from lrmh_seq on phase change */

/* ============================================================
 * LC notification command — sent by SC RTS_NOTIFY_FAULT
 *
 * The LC actionpoint fires this RTS when any threshold WP
 * crosses; the RTS is a single CCSDS command carrying the
 * fault flag in the payload. lrmh_fault dispatches the recovery
 * tier per the static table.
 * ============================================================ */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
    uint16  FaultFlag;          /* one of LRMH_FAULT_FLAG_* */
    uint16  Reserved;
} LRMH_FaultLcNotifyCmd_t;

/* ============================================================
 * Phase advance notification — sent by lrmh_seq on every
 * TransitionTo so lrmh_fault can reset T1AttemptsCurrentPhase
 * ============================================================ */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
    uint8   NewPhase;           /* LRMH_Phase_t */
    uint8   Reserved[3];
} LRMH_FaultPhaseAdvanceCmd_t;

/* ============================================================
 * Fault status telemetry (LRMH_FAULT_TLM_MID)
 *
 * Subscribed by lrmh_hk for the aggregated HK packet. Updated
 * on every fault evaluation cycle.
 * ============================================================ */
typedef struct {
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint16  FaultFlags;          /* active fault bitmask */
    uint8   FaultActive;
    uint8   RecoveryTier;        /* most recent dispatch tier */
    uint8   T1AttemptsCurrentPhase;
    uint8   T2EventsInWindow;
    uint8   FaultEnabled;
    uint8   Reserved8;
    uint32  TimeoutCount;        /* SENSOR_TIMEOUT escalations */
    uint32  TotalT1Events;
    uint32  TotalT2Events;
    uint32  TotalT3Events;
    uint32  CmdCount;
    uint32  ErrCount;
} LRMH_FaultTlm_t;

#endif /* LRMH_FAULT_MSG_H */
