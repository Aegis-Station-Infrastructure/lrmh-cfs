/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file sc_rts002.c
 * @brief LRMH SC RTS 002 — RESUME_PHASE (T1 fallback)
 *
 * Per ADR-0017 (FDIR via LC + SC + HS) and ADR-0008 (recovery
 * tiers). T1 RESUME_PHASE recovery is handled by lrmh_fault's
 * in-app orchestration. This RTS is a ground-startable stub: no
 * LC actionpoint references it and nothing starts it automatically.
 *
 * Stub: a single LRMH_NOOP_CC to lrmh_cmd. The full T1 sequence
 * (Hold → wait → retract 5 mm → wait → resume) cannot be
 * expressed as an SC RTS without parameter injection from the
 * sequencer; the orchestrator path is the canonical T1
 * mechanism. This RTS exists to keep the SC table set complete
 * and to give HS a target ID to reference if the policy ever
 * needs a NOOP-class fallback for an HS restart.
 */

#include "cfe.h"
#include "cfe_tbl_filedef.h"

#include "sc_tbldefs.h"
#include "sc_platform_cfg.h"
#include "sc_msgdefs.h"
#include "sc_msgids.h"
#include "sc_msg.h"

#include "lrmh_msgids.h"

/* Mirrors LRMH_NoArgsCmd_t — CmdHeader only, no payload */
typedef struct {
    CFE_MSG_CommandHeader_t CmdHeader;
} LRMH_RtsNoArgsCmd_t;

typedef struct {
    SC_RtsEntryHeader_t  hdr1;
    LRMH_RtsNoArgsCmd_t  cmd1;
} SC_RtsStruct002_t;

typedef union {
    SC_RtsStruct002_t rts;
    uint16            buf[SC_RTS_BUFF_SIZE];
} SC_RtsTable002_t;

#define SC_MEMBER_SIZE(member) (sizeof(((SC_RtsStruct002_t *)0)->member))

/* LRMH_NOOP_CC = 0 (apps/lrmh_cmd/fsw/inc/lrmh_cmd_msg.h) */
#define LRMH_RTS_NOOP_CC 0

SC_RtsTable002_t SC_Rts002 = {
    .rts.hdr1.WakeupCount = 0,
    .rts.cmd1 = {
        CFE_MSG_CMD_HDR_INIT(LRMH_CMD_MID,
                             SC_MEMBER_SIZE(cmd1),
                             LRMH_RTS_NOOP_CC,
                             0)
    }
};

CFE_TBL_FILEDEF(SC_Rts002, SC.RTS_TBL002, LRMH RESUME_PHASE RTS, sc_rts002.tbl)
