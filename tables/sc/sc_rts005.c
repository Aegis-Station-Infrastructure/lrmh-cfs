/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file sc_rts005.c
 * @brief LRMH SC RTS 005 — NOTIFY_FAULT (OVERCURRENT)
 *
 * Per ADR-0017 (FDIR via LC + SC + HS).
 *
 * Wired to the LRMH_AP_OVERCURRENT actionpoint, which watches
 * LRMH_WP_CURRENT. Carries LRMH_FAULT_FLAG_OVERCURRENT (T3) in
 * the LRMH_FaultLcNotifyCmd_t payload. lrmh_fault's tier table
 * dispatches T3 SAFE recovery on receipt.
 *
 * See sc_rts001.c header for the per-flag NOTIFY RTS pattern.
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
} SC_RtsStruct005_t;

typedef union {
    SC_RtsStruct005_t rts;
    uint16            buf[SC_RTS_BUFF_SIZE];
} SC_RtsTable005_t;

#define SC_MEMBER_SIZE(member) (sizeof(((SC_RtsStruct005_t *)0)->member))

SC_RtsTable005_t SC_Rts005 = {
    .rts.hdr1.WakeupCount = 0,
    .rts.cmd1 = {
        CFE_MSG_CMD_HDR_INIT(LRMH_FAULT_CMD_MID,
                             SC_MEMBER_SIZE(cmd1),
                             LRMH_FAULT_LC_NOTIFY_CC,
                             0)
    },
    .rts.cmd1.FaultFlag = LRMH_FAULT_FLAG_OVERCURRENT,
    .rts.cmd1.Reserved  = 0
};

CFE_TBL_FILEDEF(SC_Rts005, SC.RTS_TBL005, LRMH NOTIFY OVERCURRENT, sc_rts005.tbl)
