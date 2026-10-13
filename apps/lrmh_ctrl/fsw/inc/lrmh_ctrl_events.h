/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_ctrl_events.h
 * @brief lrmh_ctrl EVS event ID registry
 */

#ifndef LRMH_CTRL_EVENTS_H
#define LRMH_CTRL_EVENTS_H

#define LRMH_CTRL_INIT_INF_EID         1
#define LRMH_CTRL_NOOP_INF_EID         2
#define LRMH_CTRL_RESET_INF_EID        3

/* Command dispatch */
#define LRMH_CTRL_CMD_OK_EID          10
#define LRMH_CTRL_CMD_HAL_ERR_EID     11
#define LRMH_CTRL_CMD_BAD_MODE_EID    12
#define LRMH_CTRL_CMD_COMPLETE_EID    13

/* Token validation (ADR-0007) */
#define LRMH_CTRL_TOKEN_RESET_EID     20
#define LRMH_CTRL_STALE_CMD_EID       21
#define LRMH_CTRL_TOKEN_BASELINE_EID  22

/* Errors */
#define LRMH_CTRL_PIPE_ERR_EID        30
#define LRMH_CTRL_LEN_ERR_EID         31

#endif /* LRMH_CTRL_EVENTS_H */
