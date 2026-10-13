/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_hal.h
 * @brief LRMH Hardware Abstraction Layer — public API
 *
 * The contract between LRMH mission apps and whatever is below the
 * HAL line. Per ADR-0001 (apps unchanged across environments), this
 * header is identical in sim, bench, and flight builds — only the
 * backend implementation that links against it changes.
 *
 * Backends:
 *   hal/sim/    — UDP client → standalone sim executable
 *   hal/bench/  — bench hardware drivers (future)
 *   hal/flight/ — flight motor controller + EDAC sensor MMIO (future)
 *
 * Contracts:
 *   ADR-0003  HAL abstraction level (position/force setpoints, not duty cycle)
 *   ADR-0009  Sensor freshness floor (valid + time_us + channel_valid on every read)
 *   ADR-0010  Actuator HAL surface (per-command contract, state machine)
 */

#ifndef LRMH_HAL_H
#define LRMH_HAL_H

#include "cfe.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Status / Error Codes
 * ============================================================ */
#define LRMH_HAL_OK                  ((int32)0)
#define LRMH_HAL_ERROR               ((int32)-1)
#define LRMH_HAL_NOT_INITIALIZED     ((int32)-2)
#define LRMH_HAL_REJECTED_ESTOP      ((int32)-3)
#define LRMH_HAL_REJECTED_FAULT      ((int32)-4)
#define LRMH_HAL_REJECTED_BAD_PARAM  ((int32)-5)
#define LRMH_HAL_REJECTED_BUSY       ((int32)-6)
#define LRMH_HAL_SELFTEST_FAIL       ((int32)-7)
#define LRMH_HAL_BACKEND_TIMEOUT     ((int32)-8)

/* ============================================================
 * Channel indices for the channel_valid bitmask (ADR-0009)
 * ============================================================ */
#define LRMH_HAL_CH_FORCE     0
#define LRMH_HAL_CH_DEPTH     1
#define LRMH_HAL_CH_STROKE    2
#define LRMH_HAL_CH_CURRENT   3
#define LRMH_HAL_CH_BUS_V     4
#define LRMH_HAL_CH_TEMP      5
#define LRMH_HAL_N_CHANNELS   6

#define LRMH_HAL_CH_BIT(i)    (1u << (i))
#define LRMH_HAL_CH_ALL       ((1u << LRMH_HAL_N_CHANNELS) - 1u)

/* ============================================================
 * Actuator state machine (ADR-0010 §"State machine")
 * ============================================================ */
typedef enum {
    LRMH_HAL_STATE_COLD     = 0,   /* power-on, no Init yet */
    LRMH_HAL_STATE_IDLE     = 1,   /* initialized, no command in progress */
    LRMH_HAL_STATE_MOVING   = 2,   /* position or force command in progress */
    LRMH_HAL_STATE_HOLDING  = 3,   /* force command completed, holding force */
    LRMH_HAL_STATE_FAULT    = 4,   /* recoverable fault, Init+Retract clears */
    LRMH_HAL_STATE_ESTOP    = 5    /* emergency stop, Init required */
} LRMH_HAL_State_t;

/* ============================================================
 * Health flag bits (ADR-0010 §"Per-command contract" / flags)
 * ============================================================ */
#define LRMH_HAL_FLAG_POSITION_REACHED         (0x0001)
#define LRMH_HAL_FLAG_STOPPED_AT_FORCE_CLAMP   (0x0002)
#define LRMH_HAL_FLAG_FORCE_REACHED            (0x0004)
#define LRMH_HAL_FLAG_STOPPED_AT_STROKE_CLAMP  (0x0008)
#define LRMH_HAL_FLAG_RETRACT_BLOCKED          (0x0010)
#define LRMH_HAL_FLAG_HARD_STOP_HIT            (0x0020)
#define LRMH_HAL_FLAG_ENCODER_LOSS             (0x0040)
#define LRMH_HAL_FLAG_OVERCURRENT              (0x0080)
#define LRMH_HAL_FLAG_THERMAL                  (0x0100)
#define LRMH_HAL_FLAG_BUS_VOLTAGE              (0x0200)
#define LRMH_HAL_FLAG_BACKEND_LOST             (0x0400)

/* ============================================================
 * Read structs — sensor data + health
 *
 * Both structs carry the freshness contract from ADR-0009:
 *   valid          — false = discard the read entirely
 *   time_us        — host monotonic at sample acquisition (NOT receipt)
 *   channel_valid  — per-channel bitmask, set bit = channel is fresh
 * ============================================================ */
typedef struct {
    bool      valid;             /* aggregate: any channel valid */
    uint8_t   reserved8[3];      /* alignment */
    uint16_t  channel_valid;     /* bit per LRMH_HAL_CH_*, ADR-0009 */
    uint16_t  reserved16;        /* alignment */
    uint64_t  time_us;           /* host monotonic, sample acquisition time */
    float     force_n;
    float     depth_m;
    float     stroke_m;
    float     current_a;
    float     bus_v;
    float     temp_c;
} LRMH_HAL_SensorData_t;

typedef struct {
    bool      valid;
    uint8_t   reserved8[3];
    uint32_t  reserved32;        /* explicit padding to the 8-byte field below */
    uint64_t  time_us;
    uint8_t   state;             /* LRMH_HAL_State_t */
    uint8_t   reserved8b[1];
    uint16_t  flags;             /* LRMH_HAL_FLAG_* bitmask */
    float     stroke_m;          /* current stroke for state inspection */
    float     last_force_n;
} LRMH_HAL_Health_t;

/* ============================================================
 * Command structs — position and force setpoints (ADR-0010)
 * ============================================================ */
typedef struct {
    float     target_stroke_m;   /* commanded stroke from retracted home */
    float     max_force_n;       /* abort the move if force exceeds this */
    float     max_speed_mps;     /* speed limit during the move */
    uint32_t  reserved;          /* alignment / future flags */
} LRMH_HAL_PositionCmd_t;

typedef struct {
    float     target_force_n;    /* commanded reaction force at the tool tip */
    float     max_stroke_m;      /* abort the move if stroke exceeds this */
    float     ramp_rate_nps;     /* force ramp rate; 0 = step */
    uint32_t  reserved;
} LRMH_HAL_ForceCmd_t;

/* ============================================================
 * Public API (implemented by each backend)
 * ============================================================ */

/**
 * @brief Initialize the HAL backend and run the actuator self-test.
 *
 * On success, actuator state is LRMH_HAL_STATE_IDLE at its current
 * stroke (the actuator is NOT moved to home — caller must Retract
 * explicitly). Self-test failures return LRMH_HAL_SELFTEST_FAIL.
 *
 * Called by lrmh_cmd on INIT_INSTRUMENT (startup and on recovery
 * from SAFE), and by lrmh_ctrl for the PRECHECK self-test at the
 * start of every site test. Init clears an ESTOP latch.
 */
int32 LRMH_HAL_Init(void);

/**
 * @brief Read the most recent sensor sample.
 *
 * The HAL backend returns whatever the most recent sample was, with
 * valid + time_us + channel_valid populated per ADR-0009. The FSW
 * is responsible for applying the per-channel freshness floor in
 * lrmh_sense.
 *
 * @param[out] out  Sensor data struct, populated on success
 * @return LRMH_HAL_OK or error code
 */
int32 LRMH_HAL_ReadSensors(LRMH_HAL_SensorData_t *out);

/**
 * @brief Read the actuator health/state.
 *
 * Used by lrmh_ctrl to detect command completion and to publish
 * the controller status (state, flags, stroke).
 */
int32 LRMH_HAL_GetHealth(LRMH_HAL_Health_t *out);

/**
 * @brief Command a position move.
 *
 * Drives the actuator from its current stroke toward target_stroke_m
 * at no more than max_speed_mps, with an inner force clamp at
 * max_force_n. If force exceeds the clamp, the move stops at the
 * stroke where the clamp fired (NOT retracted) and reports
 * LRMH_HAL_FLAG_STOPPED_AT_FORCE_CLAMP. ADR-0010.
 *
 * Returns LRMH_HAL_OK on accept (NOT completion). Caller polls
 * GetHealth for completion.
 */
int32 LRMH_HAL_CommandPosition(const LRMH_HAL_PositionCmd_t *cmd);

/**
 * @brief Command a force application.
 *
 * Drives the actuator to apply target_force_n at the tool tip,
 * ramping at ramp_rate_nps if non-zero. Inner stroke clamp at
 * max_stroke_m. On clamp the actuator HOLDS the last commanded
 * force (does not retract). ADR-0010.
 *
 * On successful completion, actuator transitions to HOLDING state.
 * To exit hold, issue another command (Retract, Hold, etc.).
 */
int32 LRMH_HAL_CommandForce(const LRMH_HAL_ForceCmd_t *cmd);

/**
 * @brief Retract the actuator to stroke = 0 at maximum safe speed.
 *
 * Implicit force clamp at LRMH_MAX_FORCE_N × 1.5. On clamp the
 * actuator stops where it is and reports LRMH_HAL_FLAG_RETRACT_BLOCKED
 * (T2 fault per ADR-0008).
 *
 * Permitted from any state except ESTOP. Used as the recovery
 * path on every fault tier and on every mode-FSM transition out
 * of TESTING.
 */
int32 LRMH_HAL_Retract(void);

/**
 * @brief Stop active move and hold current stroke + current force.
 *
 * Reachable from ground manual control through lrmh_seq. Hold
 * completes immediately; state becomes HOLDING.
 */
int32 LRMH_HAL_Hold(void);

/**
 * @brief Emergency stop — kill motor power, latch the actuator.
 *
 * Issued by lrmh_cmd on a ground ABORT and reachable from ground
 * manual control. Recovery requires Init. ADR-0010.
 */
int32 LRMH_HAL_EStop(void);

/**
 * @brief Sim-only: advance the simulation by DeltaTime_s.
 *
 * Implemented as a UDP request in hal/sim/, no-op in bench/flight
 * backends. The sim steps itself on the wall clock; no app calls
 * this today. Kept for a lock-step test harness.
 */
int32 LRMH_HAL_StepSim(double DeltaTime_s);

#ifdef __cplusplus
}
#endif

#endif /* LRMH_HAL_H */
