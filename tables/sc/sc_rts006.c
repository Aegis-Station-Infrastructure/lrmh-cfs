/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file sc_rts006.c
 * @brief LRMH SC RTS 006 — NOTIFY_FAULT (HARD_STOP)
 *
 * Per ADR-0017. Wired to LRMH_AP_HARD_STOP, which watches
 * LRMH_WP_STROKE. Carries LRMH_FAULT_FLAG_HARD_STOP (T2);
 * lrmh_fault dispatches SKIP_SITE on receipt.
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
} SC_RtsStruct006_t;

typedef union {
    SC_RtsStruct006_t rts;
    uint16            buf[SC_RTS_BUFF_SIZE];
} SC_RtsTable006_t;

#define SC_MEMBER_SIZE(member) (sizeof(((SC_RtsStruct006_t *)0)->member))

SC_RtsTable006_t SC_Rts006 = {
    .rts.hdr1.WakeupCount = 0,
    .rts.cmd1 = {
        CFE_MSG_CMD_HDR_INIT(LRMH_FAULT_CMD_MID,
                             SC_MEMBER_SIZE(cmd1),
                             LRMH_FAULT_LC_NOTIFY_CC,
                             0)
    },
    .rts.cmd1.FaultFlag = LRMH_FAULT_FLAG_HARD_STOP,
    .rts.cmd1.Reserved  = 0
};

CFE_TBL_FILEDEF(SC_Rts006, SC.RTS_TBL006, LRMH NOTIFY HARD_STOP, sc_rts006.tbl)
