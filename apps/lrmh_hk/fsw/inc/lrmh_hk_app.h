/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_hk_app.h
 * @brief LRMH lrmh_hk application — public interface
 *
 * Per ADR-0006: housekeeping aggregator. Subscribes to per-app
 * status TLMs and caches the relevant fields. Publishes
 * `LRMH_HK_TLM_MID` at 1 Hz.
 *
 * Does not call the HAL; the controller and sense apps do.
 */

#ifndef LRMH_HK_APP_H
#define LRMH_HK_APP_H

#include "cfe.h"
#include "lrmh_msgids.h"
#include "lrmh_msg.h"
#include "lrmh_platform_cfg.h"

/* Bring in the per-app TLM shapes that lrmh_hk subscribes to */
#include "lrmh_seq_msg.h"
#include "lrmh_ctrl_msg.h"
#include "lrmh_sense_msg.h"
#include "lrmh_cmd_msg.h"
#include "lrmh_fault_msg.h"

#include "lrmh_hk_msg.h"
#include "lrmh_hk_events.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * App data
 * ============================================================ */
typedef struct {
    CFE_SB_PipeId_t  CmdPipe;

    /* The aggregated HK packet (rebuilt on every publish) */
    LRMH_HkTlm_t     HkPacket;

    /* Cached snapshots from each subscribed TLM. Updated on
     * receipt; copied into HkPacket on publish. */
    struct {
        bool    valid;
        uint8   phase;
        uint8   quality;
        uint8   waiting_for_ctrl;
        uint8   t1_attempts;
        uint16  site_id;
        uint16  session_id;
        uint16  epoch;
        uint32  cmd_count;
        uint32  err_count;
    } SeqCache;

    struct {
        bool    valid;
        uint8   active_mode;
        uint8   command_pending;
        uint8   hal_state;
        uint16  hal_flags;
        uint16  stale_count;
        uint32  cmd_count;
        uint32  err_count;
        float   stroke_m;
        float   last_force_n;
    } CtrlCache;

    struct {
        bool    valid;
        uint16  channel_valid;
        float   force_n;
        float   depth_m;
        float   current_a;
        float   bus_v;
        float   temp_c;
        float   force_rate_nps;
    } SenseCache;

    /* lrmh_cmd cache */
    struct {
        bool    valid;
        uint8   mode;
        uint32  cmd_count;
        uint32  err_count;
        uint32  rejected_count;
        uint32  dispatch_cache_hits;
    } CmdCache;

    /* Cmd-derived fields propagated to HK packet */
    uint8   InstrumentMode;
    uint8   ActiveTool;              /* reserved: not populated */
    uint16  FaultFlags;
    uint8   FaultActive;
    uint8   RecoveryTier;
    uint32  TimeoutCount;

    uint32  HkCycleCount;
    uint32  TotalRejectedCount;
} LRMH_HK_AppData_t;

/* ============================================================
 * Public entry point
 * ============================================================ */
void LRMH_HK_AppMain(void);

/* ============================================================
 * Test-exposed internal API
 * ============================================================ */
extern LRMH_HK_AppData_t g_HkApp;

void LRMH_HK_OnSeqStatus(const LRMH_SeqStatusTlm_t *seq);
void LRMH_HK_OnCtrlStatus(const LRMH_CtrlStatusTlm_t *ctrl);
void LRMH_HK_OnSenseSample(const LRMH_SampleTlm_t *sense);
void LRMH_HK_OnCmdStatus(const LRMH_CmdTlm_t *cmd);
void LRMH_HK_OnFaultStatus(const LRMH_FaultTlm_t *fault);
void LRMH_HK_PublishHk(void);

#ifdef __cplusplus
}
#endif

#endif /* LRMH_HK_APP_H */
