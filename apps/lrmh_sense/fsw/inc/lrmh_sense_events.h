/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_sense_events.h
 * @brief lrmh_sense EVS event ID registry
 */

#ifndef LRMH_SENSE_EVENTS_H
#define LRMH_SENSE_EVENTS_H

#define LRMH_SENSE_INIT_INF_EID         1
#define LRMH_SENSE_RESET_INF_EID        3
#define LRMH_SENSE_NOOP_INF_EID         2

/* Acquisition lifecycle */
#define LRMH_SENSE_START_EID           10
#define LRMH_SENSE_STOP_EID            11
#define LRMH_SENSE_ZERO_FORCE_EID      12
#define LRMH_SENSE_ZERO_DEPTH_EID      13

/* Freshness floor (ADR-0009) */
#define LRMH_SENSE_STALE_CHANNEL_EID   20
#define LRMH_SENSE_TIMEOUT_EID         21
#define LRMH_SENSE_FRESH_RECOVERED_EID 22

/* Token validation (ADR-0007) */
#define LRMH_SENSE_TOKEN_RESET_EID     30
#define LRMH_SENSE_STALE_CMD_EID       31

/* Errors */
#define LRMH_SENSE_HAL_ERR_EID         40
#define LRMH_SENSE_PIPE_ERR_EID        41
#define LRMH_SENSE_LEN_ERR_EID         42

#endif /* LRMH_SENSE_EVENTS_H */
