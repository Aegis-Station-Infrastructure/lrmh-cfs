/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_seq_events.h
 * @brief lrmh_seq EVS event ID registry
 */

#ifndef LRMH_SEQ_EVENTS_H
#define LRMH_SEQ_EVENTS_H

/* Lifecycle */
#define LRMH_SEQ_INIT_INF_EID            1
#define LRMH_SEQ_NOOP_INF_EID            2
#define LRMH_SEQ_RESET_INF_EID           3

/* Phase machine */
#define LRMH_SEQ_PHASE_ADVANCE_EID      10
#define LRMH_SEQ_PHASE_REJECT_EID       11
#define LRMH_SEQ_TEST_START_EID         12
#define LRMH_SEQ_TEST_COMPLETE_EID      13
#define LRMH_SEQ_PHASE_TIMEOUT_EID      14

/* Recovery (ADR-0008) */
#define LRMH_SEQ_RECOVERY_T1_EID        20
#define LRMH_SEQ_RECOVERY_T2_EID        21
#define LRMH_SEQ_RECOVERY_T3_EID        22
#define LRMH_SEQ_RECOVERY_REJECT_EID    23

/* Token (ADR-0007) */
#define LRMH_SEQ_TOKEN_BUMP_EID         30

/* Manual ground actuator command */
#define LRMH_SEQ_MANUAL_CTRL_EID        35
#define LRMH_SEQ_MANUAL_CTRL_REJECT_EID 36

/* Errors */
#define LRMH_SEQ_PIPE_ERR_EID           40
#define LRMH_SEQ_LEN_ERR_EID            41
#define LRMH_SEQ_FCN_ERR_EID            42

#endif /* LRMH_SEQ_EVENTS_H */
