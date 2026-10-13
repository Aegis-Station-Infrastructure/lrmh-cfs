/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file sc_rts009.c
 * @brief LRMH SC RTS 009 — NOTIFY_FAULT (FORCE_RATE)
 *
 * Per ADR-0017. Wired to LRMH_AP_FORCE_RATE, which watches
 * LRMH_WP_FORCE_RATE. Carries LRMH_FAULT_FLAG_FORCE_RATE (T1);
 * lrmh_fault dispatches RESUME_PHASE.
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
} SC_RtsStruct009_t;

typedef union {
    SC_RtsStruct009_t rts;
    uint16            buf[SC_RTS_BUFF_SIZE];
} SC_RtsTable009_t;

#define SC_MEMBER_SIZE(member) (sizeof(((SC_RtsStruct009_t *)0)->member))

SC_RtsTable009_t SC_Rts009 = {
    .rts.hdr1.WakeupCount = 0,
    .rts.cmd1 = {
        CFE_MSG_CMD_HDR_INIT(LRMH_FAULT_CMD_MID,
                             SC_MEMBER_SIZE(cmd1),
                             LRMH_FAULT_LC_NOTIFY_CC,
                             0)
    },
    .rts.cmd1.FaultFlag = LRMH_FAULT_FLAG_FORCE_RATE,
    .rts.cmd1.Reserved  = 0
};

CFE_TBL_FILEDEF(SC_Rts009, SC.RTS_TBL009, LRMH NOTIFY FORCE_RATE, sc_rts009.tbl)
