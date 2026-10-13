/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_msgids.h
 * @brief LRMH Mission Message ID registry
 *
 * Central message ID registry for all LRMH cFS apps. See ADR-0005
 * (MID block allocation) in docs/LRMH_Design_Decisions.md.
 *
 *   Command MIDs    : 0x18A0–0x18A7  (LRMH-owned)
 *   Internal MIDs   : 0x18B0–0x18B7  (LRMH-owned, app-to-app)
 *   Scheduler MIDs  : 0x18C0–0x18C2  (LRMH-owned, scheduler wakeups)
 *   Telemetry MIDs  : 0x08A0–0x08AA  (LRMH-owned)
 *   Inbound subscribe : 0x08C5, 0x08C6  (rover Pose/Vel, ADR-0011)

 */

#ifndef LRMH_MSGIDS_H
#define LRMH_MSGIDS_H

/* ===========================================================
 * Command Message IDs  (ground/host → instrument)
 * Per ADR-0005.
 * =========================================================== */
#define LRMH_CMD_MID                0x18A0  /* Instrument-level commands         */
#define LRMH_SEND_HK_MID            0x18A1  /* Request HK telemetry              */
#define LRMH_SEQ_CMD_MID            0x18A2  /* Sequencer commands                */
#define LRMH_CTRL_CMD_MID           0x18A3  /* Actuator control commands         */
#define LRMH_SENSE_CMD_MID          0x18A4  /* Sensor acquisition commands       */
#define LRMH_PROC_CMD_MID           0x18A5  /* Processing commands               */
#define LRMH_REPORT_CMD_MID         0x18A6  /* Report generation commands        */
#define LRMH_FAULT_CMD_MID          0x18A7  /* Fault management commands         */

/* ===========================================================
 * Internal Command MIDs  (app-to-app on SB)
 * =========================================================== */
#define LRMH_SEQ_TO_CTRL_MID        0x18B0  /* Sequencer → Controller            */
#define LRMH_SEQ_TO_SENSE_MID       0x18B1  /* Sequencer → Sensor acq start/stop */
#define LRMH_SEQ_TO_PROC_MID        0x18B2  /* Sequencer → Processing trigger    */
#define LRMH_SEQ_TO_REPORT_MID      0x18B3  /* Sequencer → Report generation     */
#define LRMH_FAULT_TO_SEQ_MID       0x18B4  /* Fault mgr → Sequencer (recovery)  */
#define LRMH_FAULT_TO_CTRL_MID      0x18B5  /* Fault mgr → Controller (retract)  */
#define LRMH_SEQ_SAFE_MID           0x18B6  /* Sequencer → ALL (SAFE broadcast)  */
#define LRMH_TOKEN_RESET_MID        0x18B7  /* CMD → ALL (token reset, ADR-0007) */
#define LRMH_SEQ_TO_CMD_MID         0x18B8  /* Sequencer → cmd FSM (test-done etc.) */

/* ===========================================================
 * Scheduler Wakeup MIDs
 * Per ADR-0006 app-split rate analysis.
 * =========================================================== */
#define LRMH_SEQ_WAKEUP_MID         0x18C0  /* lrmh_seq    @ 10 Hz */
#define LRMH_SENSE_WAKEUP_MID       0x18C1  /* lrmh_sense  @ 100 Hz */
#define LRMH_FAULT_WAKEUP_MID       0x18C2  /* lrmh_fault  @ 10 Hz (ADR-0013 polled path) */

/* ===========================================================
 * Telemetry Message IDs  (instrument → ground/rover)
 * =========================================================== */
#define LRMH_HK_TLM_MID             0x08A0  /* Housekeeping telemetry            */
#define LRMH_SEQ_STATUS_MID         0x08A1  /* Sequencer state telemetry         */
#define LRMH_CTRL_STATUS_MID        0x08A2  /* Controller status telemetry       */
#define LRMH_SENSE_TLM_MID          0x08A3  /* Raw sensor sample telemetry       */
#define LRMH_PROC_TLM_MID           0x08A4  /* Processed/derived data telemetry  */
/* 0x08A5 reserved (lrmh_report publishes on 0x08AA and 0x08AB). */
#define LRMH_FAULT_TLM_MID          0x08A6  /* Fault status telemetry            */
/* 0x08A7 and 0x08A8 reserved. */

/* Per ADRs 0015 and 0016 */
#define LRMH_PROC_PHASE_EXTENDED_TLM_MID   0x08A9  /* RAW→STREAMING transition warning, ADR-0015 */
#define LRMH_REPORT_SITE_RECORD_MID        0x08AA  /* Binary site result record, ADR-0016 (DS captures) */
#define LRMH_REPORT_SITE_HUMAN_MID         0x08AB  /* Human-readable site result, ADR-0016 (DS captures) */
#define LRMH_CMD_STATUS_MID                0x08AC  /* lrmh_cmd status TLM */

/* NASA LC command MID as installed by install.sh, which relocates LC's
 * CMD topic from the upstream 0xA4 to 0xC3 to clear the LRMH block.
 * lrmh_fault uses it to re-arm actionpoints (LC_SET_AP_STATE_CC). */
#define LRMH_LC_CMD_MID             0x18C3

/* ===========================================================
 * Inbound subscriptions  (LRMH does NOT own these MIDs;
 * they are published by the host vehicle and LRMH subscribes.
 * Per ADR-0011 host-vehicle telemetry subscription.)
 * =========================================================== */
/* The host pose and velocity MIDs (0x08C5, 0x08C6) are defined as
 * LRMH_HOST_POSE_MID_VALUE / LRMH_HOST_VEL_MID_VALUE in
 * cfs_defs/lrmh_platform_cfg.h, where a host-vehicle build can
 * override them. */

/* Reserved range for future host vehicles whose own MIDs collide
 * with LRMH's allocation. Bridge-republished into this range. */
#define LRMH_HOST_RESERVED_BASE     0x08D0
#define LRMH_HOST_RESERVED_END      0x08D7

#endif /* LRMH_MSGIDS_H */
