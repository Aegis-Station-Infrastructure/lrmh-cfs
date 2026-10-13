/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file sc_rts003.c
 * @brief LRMH SC RTS 003 — SKIP_SITE (T2 fallback)
 *
 * Per ADR-0017 / ADR-0008. T2 SKIP_SITE recovery is dispatched by
 * lrmh_fault. This RTS is a ground-startable stub (a single
 * LRMH_ABORT_CC to lrmh_cmd): no LC actionpoint references it and
 * nothing starts it automatically. ABORT moves the mode FSM out of
 * TESTING and broadcasts the token reset, which is enough to
 * stop in-progress sample acquisition and prevent further
 * recovery attempts on the same site.
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
} SC_RtsStruct003_t;

typedef union {
    SC_RtsStruct003_t rts;
    uint16            buf[SC_RTS_BUFF_SIZE];
} SC_RtsTable003_t;

#define SC_MEMBER_SIZE(member) (sizeof(((SC_RtsStruct003_t *)0)->member))

/* LRMH_ABORT_CC = 5 */
#define LRMH_RTS_ABORT_CC 5

SC_RtsTable003_t SC_Rts003 = {
    .rts.hdr1.WakeupCount = 0,
    .rts.cmd1 = {
        CFE_MSG_CMD_HDR_INIT(LRMH_CMD_MID,
                             SC_MEMBER_SIZE(cmd1),
                             LRMH_RTS_ABORT_CC,
                             0)
    }
};

CFE_TBL_FILEDEF(SC_Rts003, SC.RTS_TBL003, LRMH SKIP_SITE RTS, sc_rts003.tbl)
