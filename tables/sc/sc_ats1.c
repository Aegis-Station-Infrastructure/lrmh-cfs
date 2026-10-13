/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file sc_ats1.c
 * @brief LRMH SC ATS table 1 — empty (RTS-only operation)
 *
 * Per ADR-0017, LRMH uses SC for RTSes only — RTS_NOTIFY_FAULT
 * (LC dispatch) and the three recovery fallbacks. The ATS
 * (Absolute Time Sequence) mechanism is unused; LRMH does not
 * have any time-scheduled command sequences.
 *
 * cFS SC requires both ATS and RTS tables to be present at
 * load time. This file provides a single no-op LRMH NOOP
 * command at a far-future time so the ATS validator passes
 * but the command never actually fires in normal operation.
 */

#include "cfe.h"
#include "cfe_tbl_filedef.h"

#include "sc_tbl.h"
#include "sc_platform_cfg.h"
#include "sc_msgdefs.h"
#include "sc_msgids.h"
#include "sc_msg.h"

#include "lrmh_msgids.h"

typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
} LRMH_AtsNoArgsCmd_t;

typedef struct {
    SC_AtsEntryHeader_t  hdr1;
    LRMH_AtsNoArgsCmd_t  cmd1;
} SC_AtsStruct1_t;

typedef union {
    SC_AtsStruct1_t ats;
    uint16          buf[SC_ATS_BUFF_SIZE];
} SC_AtsTable1_t;

#define SC_MEMBER_SIZE(member) (sizeof(((SC_AtsStruct1_t *)0)->member))

#define LRMH_ATS_NOOP_CC 0

/* Far-future timestamp so the ATS never actually fires during
 * a normal LRMH session. 0x7FFF0000 ≈ 2.1B seconds. */
#define LRMH_ATS_FAR_FUTURE 0x7FFF0000U

SC_AtsTable1_t SC_Ats1 = {
    .ats.hdr1 = {
        .CmdNumber  = SC_COMMAND_NUM_INITIALIZER(1),
        .TimeTag_MS = LRMH_ATS_FAR_FUTURE >> 16,
        .TimeTag_LS = LRMH_ATS_FAR_FUTURE & 0xFFFF
    },
    .ats.cmd1 = {
        CFE_MSG_CMD_HDR_INIT(LRMH_CMD_MID,
                             SC_MEMBER_SIZE(cmd1),
                             LRMH_ATS_NOOP_CC,
                             0)
    }
};

CFE_TBL_FILEDEF(SC_Ats1, SC.AtsTable1, LRMH ATS placeholder, sc_ats1.tbl)
