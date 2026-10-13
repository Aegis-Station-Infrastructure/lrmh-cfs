/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_report_events.h
 * @brief lrmh_report EVS event ID registry
 */

#ifndef LRMH_REPORT_EVENTS_H
#define LRMH_REPORT_EVENTS_H

#define LRMH_REPORT_INIT_INF_EID         1
#define LRMH_REPORT_RESET_INF_EID       3
#define LRMH_REPORT_NOOP_INF_EID         2

#define LRMH_REPORT_PHASE_INGEST_EID    10
#define LRMH_REPORT_GENERATE_EID        11
#define LRMH_REPORT_PARTIAL_TAGGED_EID  12
#define LRMH_REPORT_RECORD_PUB_EID      13  /* binary site record published */
#define LRMH_REPORT_HUMAN_PUB_EID       14  /* human-readable summary published */
#define LRMH_REPORT_PUB_ERR_EID         15  /* site record publication failed */
#define LRMH_REPORT_PHASE_ECHO_EID      16  /* per-phase values echoed at publish */
#define LRMH_REPORT_AGG_ECHO_EID        17  /* aggregate values echoed at publish */

#define LRMH_REPORT_TOKEN_RESET_EID     20
#define LRMH_REPORT_STALE_CMD_EID       21

#define LRMH_REPORT_PIPE_ERR_EID        30
#define LRMH_REPORT_TEXT_OVERFLOW_EID   31

#endif /* LRMH_REPORT_EVENTS_H */
