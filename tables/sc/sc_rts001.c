/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file sc_rts001.c
 * @brief LRMH SC RTS 001 — NOTIFY_FAULT (FORCE_SATURATION)
 *
 * Per ADR-0017 (FDIR via LC + SC + HS).
 *
 * Single-command RTS that fires LRMH_FAULT_LC_NOTIFY_CC against
 * lrmh_fault carrying LRMH_FAULT_FLAG_FORCE_SATURATION. Wired
 * to the LRMH_AP_FORCE_SATURATION actionpoint (lc_actionpoints.c),
 * which watches LRMH_WP_FORCE.
 *
 * Per-fault-flag NOTIFY RTSes occupy slots 1 and 5–9
 * (slots 2–4 hold the recovery fallbacks). Each RTS carries a
 * static fault flag because SC RTSes cannot inject the
 * per-watchpoint CustomFuncArgument at fire time.
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

/* The LRMH_FaultLcNotifyCmd_t struct from lrmh_fault_msg.h:
 *     CFE_MSG_CommandHeader_t CmdHeader;
 *     uint16  FaultFlag;
 *     uint16  Reserved;
 *
 * Total cmd size = sizeof(LRMH_FaultLcNotifyCmd_t).
 *
 * checksum is set to 0 — cFS native build doesn't validate cmd
 * checksums on the SB delivery path; ground commands sent via
 * CI_LAB also use cksum=0 and are accepted. */

typedef struct {
    SC_RtsEntryHeader_t      hdr1;
    LRMH_FaultLcNotifyCmd_t  cmd1;
} SC_RtsStruct001_t;

typedef union {
    SC_RtsStruct001_t rts;
    uint16            buf[SC_RTS_BUFF_SIZE];
} SC_RtsTable001_t;

#define SC_MEMBER_SIZE(member) (sizeof(((SC_RtsStruct001_t *)0)->member))

SC_RtsTable001_t SC_Rts001 = {
    .rts.hdr1.WakeupCount = 0,
    .rts.cmd1 = {
        CFE_MSG_CMD_HDR_INIT(LRMH_FAULT_CMD_MID,
                             SC_MEMBER_SIZE(cmd1),
                             LRMH_FAULT_LC_NOTIFY_CC,
                             0)
    },
    .rts.cmd1.FaultFlag = LRMH_FAULT_FLAG_FORCE_SATURATION,
    .rts.cmd1.Reserved  = 0
};

CFE_TBL_FILEDEF(SC_Rts001, SC.RTS_TBL001, LRMH NOTIFY FORCE_SATURATION, sc_rts001.tbl)
