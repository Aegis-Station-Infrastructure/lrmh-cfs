/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_proc_events.h
 * @brief lrmh_proc EVS event ID registry
 */

#ifndef LRMH_PROC_EVENTS_H
#define LRMH_PROC_EVENTS_H

#define LRMH_PROC_INIT_INF_EID         1
#define LRMH_PROC_NOOP_INF_EID         2

#define LRMH_PROC_PHASE_DONE_EID      10
#define LRMH_PROC_PHASE_PARTIAL_EID   11  /* insufficient samples */
#define LRMH_PROC_LAYER_BREAK_EID     12

#define LRMH_PROC_STALE_SAMPLE_EID    20  /* phase-start rejection per ADR-0009 */
#define LRMH_PROC_TOKEN_RESET_EID     21
#define LRMH_PROC_STALE_CMD_EID       22
#define LRMH_PROC_SEGMENT_START_EID   23
#define LRMH_PROC_RESET_INF_EID       24  /* counters reset */  /* measured segment started, buffer cleared */

#define LRMH_PROC_PIPE_ERR_EID        30
#define LRMH_PROC_BUFFER_FULL_EID     31

#endif /* LRMH_PROC_EVENTS_H */
