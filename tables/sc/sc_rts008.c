/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file sc_rts008.c
 * @brief LRMH SC RTS 008 — NOTIFY_FAULT (BUS_VOLTAGE)
 *
 * Per ADR-0017. Wired to LRMH_AP_BUS_VOLTAGE, which OR-aggregates
 * LRMH_WP_BUS_LOW and LRMH_WP_BUS_HIGH (under- and over-voltage
 * both raise the same fault flag). Carries
 * LRMH_FAULT_FLAG_BUS_VOLTAGE (T3); lrmh_fault dispatches SAFE.
 */

#include "cfe.h"
#include "cfe_tbl_filedef.h"

#include "sc_tbldefs.h"
#include "sc_platform_cfg.h"
#include "sc_msgdefs.h"
#include "sc_msgids.h"
#include "sc_msg.h"

#include "lrmh_msgids.h"
#include "lrmh_fault_msg.h"

typedef struct {
    SC_RtsEntryHeader_t      hdr1;
    LRMH_FaultLcNotifyCmd_t  cmd1;
} SC_RtsStruct008_t;

typedef union {
    SC_RtsStruct008_t rts;
    uint16            buf[SC_RTS_BUFF_SIZE];
} SC_RtsTable008_t;

#define SC_MEMBER_SIZE(member) (sizeof(((SC_RtsStruct008_t *)0)->member))

SC_RtsTable008_t SC_Rts008 = {
    .rts.hdr1.WakeupCount = 0,
    .rts.cmd1 = {
        CFE_MSG_CMD_HDR_INIT(LRMH_FAULT_CMD_MID,
                             SC_MEMBER_SIZE(cmd1),
                             LRMH_FAULT_LC_NOTIFY_CC,
                             0)
    },
    .rts.cmd1.FaultFlag = LRMH_FAULT_FLAG_BUS_VOLTAGE,
    .rts.cmd1.Reserved  = 0
};

CFE_TBL_FILEDEF(SC_Rts008, SC.RTS_TBL008, LRMH NOTIFY BUS_VOLTAGE, sc_rts008.tbl)
