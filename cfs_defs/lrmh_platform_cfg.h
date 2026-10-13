/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_platform_cfg.h
 * @brief LRMH platform configuration — single source of truth
 *
 * Per ADR-0012 (environmental envelopes). Every envelope value here
 * is consumed by:
 *   - HAL safety clamps (motor controller below the line)
 *   - LC watchpoint thresholds (tables/lc_watchpoints.c)
 *   - lrmh_fault polled-path checks (ADR-0013)
 *   - Ground console gauge limits (via build-time config export)
 *
 * Each value is one of:
 *   HW   — hardware physical or absolute electrical limit
 *   OP   — operational limit
 *   SCI  — science envelope
 *
 * Each envelope has an LC margin — the watchpoint threshold sits
 * below the envelope so the FSW faults BEFORE the limit is crossed.
 *
 * NONE OF THESE NUMBERS ARE CANONICAL. The canonical RMH-class
 * capability page contains zero hard numbers; these are project-
 * level engineering choices for the Aegis realization. See ADR-0012.
 */

#ifndef LRMH_PLATFORM_CFG_H
#define LRMH_PLATFORM_CFG_H

/* ===========================================================
 * Force envelope (ADR-0012)
 * =========================================================== */
#define LRMH_MAX_FORCE_N                   650.0f    /* OP: motor continuous duty rating × tool gear */
#define LRMH_LC_FORCE_THRESHOLD_N          600.0f    /* LC margin: 7.7% below envelope */
#define LRMH_MAX_FORCE_RATE_NPS           1000.0f    /* OP: force loop trackable dF/dt */
#define LRMH_LC_FORCE_RATE_THRESHOLD_NPS   800.0f    /* LC margin */

/* ===========================================================
 * Current envelope
 * =========================================================== */
#define LRMH_MAX_CURRENT_A                  5.0f     /* HW: motor controller continuous rating */
#define LRMH_LC_CURRENT_THRESHOLD_A         4.5f     /* LC margin: 10% */

/* ===========================================================
 * Stroke envelope
 * =========================================================== */
#define LRMH_MAX_STROKE_M                   0.35f    /* HW: mechanical hard stop */
#define LRMH_LC_STROKE_THRESHOLD_M          0.33f    /* LC margin: 5.7%, about 2 s of travel at 10 mm/s */

/* ===========================================================
 * Temperature envelope
 * =========================================================== */
#define LRMH_MAX_TEMP_C                    60.0f     /* OP: motor winding insulation derate point */
#define LRMH_LC_TEMP_THRESHOLD_C           55.0f     /* LC margin: ~30 s warning at typical rise */

/* ===========================================================
 * Bus voltage envelope (28 V nominal, ±20% operating window)
 * =========================================================== */
#define LRMH_BUS_V_MIN                     22.0f     /* HW: motor controller min operating */
#define LRMH_BUS_V_MAX                     34.0f     /* HW: motor controller max input */
#define LRMH_LC_BUS_V_LOW_THRESHOLD        23.0f     /* LC margin */
#define LRMH_LC_BUS_V_HIGH_THRESHOLD       33.0f     /* LC margin */

/* ===========================================================
 * Sensor freshness floors (ADR-0009)
 *
 * LRMH_SENSOR_TIMEOUT_MS is the polled-path aggregate floor used
 * by lrmh_fault to escalate to SENSOR_TIMEOUT when the sample
 * cache stops updating. Per-channel max-age compares would
 * require cross-epoch calibration between the HAL clock domain
 * (sim CLOCK_MONOTONIC, hardware tick) and FSW cFE TIME —
 * deferred to a later revision. Until then, lrmh_sense relies on the
 * HAL backend's data.valid + channel_valid bits.
 * =========================================================== */
#define LRMH_SENSOR_TIMEOUT_MS              200

/* ===========================================================
 * Host nav freshness (ADR-0011)
 * =========================================================== */
#define LRMH_HOST_NAV_MAX_AGE_MS            500      /* 10 missed publications at rover 20 Hz */
#define LRMH_HOST_NAV_TIMEOUT_S               5      /* TESTING-only fault threshold */

/* ===========================================================
 * Actuator timeouts and depth disagreement
 * =========================================================== */
#define LRMH_ACTUATOR_TIMEOUT_S            30.0f     /* max single-move duration */
#define LRMH_DEPTH_DISAGREE_THRESHOLD_M    0.005f    /* 5 mm: far above the 0.1 mm encoder noise; catches a slipped or frozen encoder */

/* ===========================================================
 * Recovery tier parameters (ADR-0008)
 * =========================================================== */
#define LRMH_T1_RETRACT_DISTANCE_M         0.005f    /* 5 mm partial retract on RESUME_PHASE */
#define LRMH_T1_RECOVERY_PAUSE_MS            200     /* pause before re-arming */
#define LRMH_T1_MAX_ATTEMPTS_PER_PHASE         3     /* T1→T2 escalation threshold */
#define LRMH_T2_ESCALATION_WINDOW              5     /* sliding site window for T2→T3 */
#define LRMH_T2_ESCALATION_THRESHOLD           3     /* T2 events in window → T3 */

/* ===========================================================
 * Signal-processing thresholds (lrmh_proc)
 *
 * Project-level tuning knobs, surfaced as named constants so they
 * are not magic literals in the source.
 * =========================================================== */

/* Layer-break detection: minimum |d²F/dz²| to register as an
 * inflection. Set against the simulator; bench data would recalibrate
 * it. Units: N/m². */
#define LRMH_PROC_LAYER_BREAK_D2F_THRESHOLD   1000.0f

/* Bearing-proxy normalization: force-at-1cm divided by this
 * value yields the dimensionless bearing index. 500 N
 * corresponds to a ~5 MPa contact at the 1 cm² nominal tool
 * tip; the index is scaled to ~1.0 for typical regolith. */
#define LRMH_PROC_BEARING_REF_N               500.0f
#define LRMH_TOOL_TIP_AREA_M2                 1.0e-4f  /* nominal 1 cm^2 cone tip (ToolId 1); used to express force as pressure */

/* Stiffness regression window: middle band of depth-sorted
 * samples used for the linear-regression slope, expressed as
 * tenths of the sample count. The 20–80% window avoids
 * surface-contact and hard-stop artifacts at the ends. */
#define LRMH_PROC_REGRESSION_LOW_TENTHS         2
#define LRMH_PROC_REGRESSION_HIGH_TENTHS        8

/* ===========================================================
 * Sample buffer sizing (ADR-0015 hybrid streaming accumulator)
 * =========================================================== */
#define LRMH_MAX_SAMPLES_PER_PHASE        10000     /* raw buffer capacity, ~100 s @ 100 Hz */
#define LRMH_RESERVOIR_SIZE                1000     /* Vitter R reservoir size */
#define LRMH_PROVENANCE_FIRST_LAST_N        500     /* first/last raw provenance anchors */
#define LRMH_HIST_DEPTH_BINS                 32     /* 2-D histogram depth axis */
#define LRMH_HIST_FORCE_BINS                 32     /* 2-D histogram force axis */

/* ===========================================================
 * Multi-site session sizing
 * =========================================================== */
#define LRMH_MAX_PHASES_PER_TEST              8
#define LRMH_MAX_SITES                      256
#define LRMH_MAX_REPORT_TEXT_LEN           2048

/* ===========================================================
 * Pipe depths (ADR-0006 / ADR-0012)
 * =========================================================== */
#define LRMH_CMD_PIPE_DEPTH                  16
#define LRMH_SEQ_PIPE_DEPTH                  16
#define LRMH_CTRL_PIPE_DEPTH                 16
#define LRMH_SENSE_PIPE_DEPTH                32     /* 100 Hz rate */
#define LRMH_PROC_PIPE_DEPTH                 48   /* sized for 100 Hz; capped at OSAL queue max (50) */
#define LRMH_REPORT_PIPE_DEPTH                8
#define LRMH_HK_PIPE_DEPTH                    8
#define LRMH_FAULT_PIPE_DEPTH                16

/* ===========================================================
 * Timing rates
 * =========================================================== */
#define LRMH_MAIN_LOOP_RATE_HZ               10     /* lrmh_seq, lrmh_ctrl, lrmh_fault */
#define LRMH_SENSE_RATE_HZ                  100     /* lrmh_sense (matches HAL sample rate) */
#define LRMH_HK_PERIOD_S                    1.0     /* HK packet publication */

/* ===========================================================
 * Inbound subscription MIDs (ADR-0011 host vehicle)
 * Defaults are rover values; override per host vehicle build.
 * =========================================================== */
#ifndef LRMH_HOST_POSE_MID_OVERRIDE
#define LRMH_HOST_POSE_MID_VALUE          0x08C5    /* AEGIS_NAV_PoseTlmMsg_t */
#endif
#ifndef LRMH_HOST_VEL_MID_OVERRIDE
#define LRMH_HOST_VEL_MID_VALUE           0x08C6    /* AEGIS_NAV_VelTlmMsg_t */
#endif

/* ===========================================================
 * File paths (sim defaults; bench/flight override)
 * =========================================================== */
#define LRMH_DATA_OUTPUT_DIR        "/cf/lrmh/data/"
#define LRMH_REPORT_OUTPUT_DIR      "/cf/lrmh/reports/"

/* ===========================================================
 * DS file rotation (ADR-0016)
 * =========================================================== */
#define LRMH_DS_FILE_RECORDS_IDX               0    /* binary site records */
#define LRMH_DS_FILE_HUMAN_IDX                 1    /* human-readable text */
#define LRMH_DS_MAX_FILE_SIZE_BYTES      1048576    /* 1 MB rotation */
#define LRMH_DS_MAX_FILE_AGE_SEC           86400    /* 24 h rotation */

/* ===========================================================
 * Table sizes
 * =========================================================== */

#endif /* LRMH_PLATFORM_CFG_H */
