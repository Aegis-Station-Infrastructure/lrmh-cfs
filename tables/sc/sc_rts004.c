/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file sc_rts004.c
 * @brief LRMH SC RTS 004 — RETRACT_AND_SAFE (T3 fallback)
 *
 * Per ADR-0017 / ADR-0008. T3 SAFE escalation is dispatched by
 * lrmh_fault. This RTS is a ground-startable stub: no LC actionpoint
 * references it, and HS cannot start an RTS on an app restart (see
 * tables/hs_table.c).
 *
 * Single LRMH_SAFE_CC command to lrmh_cmd. The mode FSM
 * transition to SAFE encompasses the retract (lrmh_cmd issues
 * the retract as part of the SAFE entry handler) and broadcasts
 * the token reset (ADR-0007), so a single command is sufficient.
 *
 * The retract here is conservative: full retract to home, no
 * speed-limiting. The HAL safety clamps catch any in-progress
 * runaway during the retract motion.
 */

#include "cfe.h"
#include "cfe_tbl_filedef.h"

#include "sc_tbldefs.h"
#include "sc_platform_cfg.h"
#include "sc_msgdefs.h"
#include "sc_msgids.h"
#include "sc_msg.h"

#include "lrmh_msgids.h"

typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
} LRMH_RtsNoArgsCmd_t;

typedef struct {
    SC_RtsEntryHeader_t  hdr1;
    LRMH_RtsNoArgsCmd_t  cmd1;
} SC_RtsStruct004_t;

typedef union {
    SC_RtsStruct004_t rts;
    uint16            buf[SC_RTS_BUFF_SIZE];
} SC_RtsTable004_t;

#define SC_MEMBER_SIZE(member) (sizeof(((SC_RtsStruct004_t *)0)->member))

/* LRMH_SAFE_CC = 10 */
#define LRMH_RTS_SAFE_CC 10

SC_RtsTable004_t SC_Rts004 = {
    .rts.hdr1.WakeupCount = 0,
    .rts.cmd1 = {
        CFE_MSG_CMD_HDR_INIT(LRMH_CMD_MID,
                             SC_MEMBER_SIZE(cmd1),
                             LRMH_RTS_SAFE_CC,
                             0)
    }
};

CFE_TBL_FILEDEF(SC_Rts004, SC.RTS_TBL004, LRMH RETRACT_AND_SAFE RTS, sc_rts004.tbl)
