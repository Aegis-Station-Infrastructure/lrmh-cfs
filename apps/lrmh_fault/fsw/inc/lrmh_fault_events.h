/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_fault_events.h
 * @brief lrmh_fault EVS event ID registry
 */

#ifndef LRMH_FAULT_EVENTS_H
#define LRMH_FAULT_EVENTS_H

#define LRMH_FAULT_INIT_INF_EID         1
#define LRMH_FAULT_NOOP_INF_EID         2
#define LRMH_FAULT_RESET_EID            3

/* Tier dispatch (ADR-0008) */
#define LRMH_FAULT_T1_DISPATCH_EID     10
#define LRMH_FAULT_T2_DISPATCH_EID     11
#define LRMH_FAULT_T3_DISPATCH_EID     12
#define LRMH_FAULT_ESCALATE_T1_T2_EID  13
#define LRMH_FAULT_ESCALATE_T2_T3_EID  14

/* Detection paths (ADR-0013) */
#define LRMH_FAULT_LC_NOTIFY_EID       20
#define LRMH_FAULT_POLLED_RAISE_EID    21
#define LRMH_FAULT_COALESCED_EID       22
#define LRMH_FAULT_SENSOR_TIMEOUT_EID  23
#define LRMH_FAULT_MODE_TRACK_EID      24  /* instrument mode tracker changed */
#define LRMH_FAULT_AP_REARM_EID        25  /* LC actionpoints re-armed */

/* Errors */
#define LRMH_FAULT_PIPE_ERR_EID        30
#define LRMH_FAULT_BAD_FLAG_EID        31

#endif /* LRMH_FAULT_EVENTS_H */
