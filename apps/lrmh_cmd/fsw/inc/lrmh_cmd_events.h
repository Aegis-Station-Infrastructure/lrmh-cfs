/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_cmd_events.h
 * @brief LRMH lrmh_cmd EVS event ID registry
 */

#ifndef LRMH_CMD_EVENTS_H
#define LRMH_CMD_EVENTS_H

/* Lifecycle */
#define LRMH_CMD_INIT_INF_EID            1
#define LRMH_CMD_NOOP_INF_EID            2
#define LRMH_CMD_RESET_INF_EID           3
#define LRMH_CMD_HOST_POSE_SUB_EID       4  /* host pose subscription unavailable (non-fatal) */

/* Mode FSM (ADR-0004) */
#define LRMH_CMD_MODE_OK_EID            10  /* legal transition */
#define LRMH_CMD_MODE_REJECT_EID        11  /* illegal transition rejected */
#define LRMH_CMD_INIT_INSTRUMENT_EID    12  /* INIT → epoch bump + token reset */
#define LRMH_CMD_DEPLOY_EID             13
#define LRMH_CMD_STOW_EID               14
#define LRMH_CMD_POWER_OFF_EID          15  /* STANDBY → OFF via POWER_OFF */

/* Site test dispatch (ADR-0014) */
#define LRMH_CMD_DISPATCH_OK_EID        20
#define LRMH_CMD_DISPATCH_REJECT_EID    21
#define LRMH_CMD_DISPATCH_RETRY_EID     22  /* idempotent retry absorbed */
#define LRMH_CMD_DISPATCH_NAV_EID       23  /* host nav precondition fail */

/* Safing (ADR-0008) */
#define LRMH_CMD_ABORT_EID              30
#define LRMH_CMD_SAFE_ENTRY_EID         31  /* mode FSM SAFE entry → token reset */
#define LRMH_CMD_FAULT_ESCALATE_EID     32  /* SAFE → FAULT terminal */
#define LRMH_CMD_FAULT_RESET_EID        33  /* manual ground reset of FAULT mode */

/* Errors */
#define LRMH_CMD_PIPE_ERR_EID           40
#define LRMH_CMD_LEN_ERR_EID            41
#define LRMH_CMD_FCN_ERR_EID            42
#define LRMH_CMD_HAL_ERR_EID            43
#define LRMH_CMD_INTERNAL_ERR_EID       44
#define LRMH_CMD_MID_ERR_EID            45  /* unknown message ID on the command pipe */
#define LRMH_CMD_NOTIFY_ERR_EID         46  /* unknown sequencer notify type */

#endif /* LRMH_CMD_EVENTS_H */
