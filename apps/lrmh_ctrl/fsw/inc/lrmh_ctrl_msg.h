/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_ctrl_msg.h
 * @brief LRMH lrmh_ctrl command and telemetry messages
 *
 * Per ADR-0010 (actuator HAL surface) and ADR-0007 (sequence
 * token). lrmh_seq builds LRMH_CtrlCmd_t from this header.
 */

#ifndef LRMH_CTRL_MSG_H
#define LRMH_CTRL_MSG_H

#include "cfe.h"
#include "lrmh_msg.h"

/* ============================================================
 * Command modes (passed in CtrlCmd.Mode field)
 *
 * Each mode maps to one or more LRMH_HAL_* function calls.
 * Position and force are first-class primitives per ADR-0003.
 * ============================================================ */
#define LRMH_CTRL_MODE_POSITION    0   /* → LRMH_HAL_CommandPosition */
#define LRMH_CTRL_MODE_FORCE       1   /* → LRMH_HAL_CommandForce */
#define LRMH_CTRL_MODE_RETRACT     2   /* → LRMH_HAL_Retract */
#define LRMH_CTRL_MODE_HOLD        3   /* → LRMH_HAL_Hold */
#define LRMH_CTRL_MODE_ESTOP       4   /* → LRMH_HAL_EStop */
#define LRMH_CTRL_MODE_PRECHECK    5   /* → LRMH_HAL_Init self-test */

/* ============================================================
 * Command codes for LRMH_CTRL_CMD_MID (ground commands)
 * ============================================================ */
#define LRMH_CTRL_NOOP_CC           0
#define LRMH_CTRL_RESET_COUNTERS_CC 1
/* The control commands themselves come on LRMH_SEQ_TO_CTRL_MID
 * from the sequencer with no FcnCode (the Mode field carries
 * the dispatch). Ground commands on LRMH_CTRL_CMD_MID are
 * limited to NOOP and RESET. */

/* ============================================================
 * Control command — sent on LRMH_SEQ_TO_CTRL_MID and
 * LRMH_FAULT_TO_CTRL_MID (reserved: no sender today)
 * ============================================================ */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
    uint32  SeqTokenPacked;             /* validated by lrmh_ctrl, ADR-0007 */
    uint8   Mode;                       /* LRMH_CTRL_MODE_* */
    uint8   Reserved[3];
    float   TargetPosition_m;           /* tool-tip depth, not stroke */
    float   TargetForce_N;
    float   RateLimit;                  /* m/s for position, N/s for force */
    float   Timeout_s;
} LRMH_CtrlCmd_t;

/* ============================================================
 * Controller status TLM (LRMH_CTRL_STATUS_MID)
 *
 * Per ADR-0010: caller polls this to detect command completion.
 * The CommandComplete bit is the primary signal lrmh_seq watches
 * for to clear its WaitingForCtrl flag.
 * ============================================================ */
typedef struct {
    CFE_MSG_TelemetryHeader_t TlmHeader;
    uint8   ActiveMode;                 /* LRMH_CTRL_MODE_* */
    uint8   CommandPending;             /* 1 = waiting for completion */
    uint8   CommandComplete;            /* 1 = last cmd done */
    uint8   HalState;                   /* LRMH_HAL_State_t */
    uint16  HalFlags;                   /* LRMH_HAL_FLAG_* bitmask */
    uint16  StaleCount;                 /* token validation rejects */
    uint32  CmdCount;
    uint32  ErrCount;
    uint16  AcceptedEpoch;              /* current accepted token state */
    uint8   AcceptedSeqState;
    uint8   AcceptedSeqCount;
    float   LastCommandedPos_m;
    float   LastCommandedForce_N;
    float   CurrentStroke_m;            /* from HAL health */
} LRMH_CtrlStatusTlm_t;

#endif /* LRMH_CTRL_MSG_H */
