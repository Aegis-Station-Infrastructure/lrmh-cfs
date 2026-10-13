/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_hal.h (cfe_stubs override)
 * @brief Stub HAL header for standalone unit tests
 *
 * Provides inline no-op implementations of every LRMH_HAL_*
 * function so tests can #include the app source under test
 * without requiring a working HAL backend.
 *
 * The shape of the structs and the return-code constants must
 * match the real hal/include/lrmh_hal.h (a test asserts the struct
 * sizes agree). The test targets put `cfe_stubs/test_inc/` ahead of
 * `hal/include/` on the include path so this header wins.
 */

#ifndef LRMH_HAL_H
#define LRMH_HAL_H

#include "cfe.h"
#include <stdbool.h>
#include <stdint.h>

#define LRMH_HAL_OK                  ((int32_t)0)
#define LRMH_HAL_ERROR               ((int32_t)-1)
#define LRMH_HAL_NOT_INITIALIZED     ((int32_t)-2)
#define LRMH_HAL_REJECTED_ESTOP      ((int32_t)-3)
#define LRMH_HAL_REJECTED_FAULT      ((int32_t)-4)
#define LRMH_HAL_REJECTED_BAD_PARAM  ((int32_t)-5)
#define LRMH_HAL_REJECTED_BUSY       ((int32_t)-6)
#define LRMH_HAL_SELFTEST_FAIL       ((int32_t)-7)
#define LRMH_HAL_BACKEND_TIMEOUT     ((int32_t)-8)

#define LRMH_HAL_CH_FORCE     0
#define LRMH_HAL_CH_DEPTH     1
#define LRMH_HAL_CH_STROKE    2
#define LRMH_HAL_CH_CURRENT   3
#define LRMH_HAL_CH_BUS_V     4
#define LRMH_HAL_CH_TEMP      5
#define LRMH_HAL_N_CHANNELS   6
#define LRMH_HAL_CH_BIT(i)    (1u << (i))
#define LRMH_HAL_CH_ALL       ((1u << LRMH_HAL_N_CHANNELS) - 1u)

typedef enum {
    LRMH_HAL_STATE_COLD     = 0,
    LRMH_HAL_STATE_IDLE     = 1,
    LRMH_HAL_STATE_MOVING   = 2,
    LRMH_HAL_STATE_HOLDING  = 3,
    LRMH_HAL_STATE_FAULT    = 4,
    LRMH_HAL_STATE_ESTOP    = 5
} LRMH_HAL_State_t;

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

typedef struct {
    bool      valid;
    uint8_t   reserved8[3];
    uint16_t  channel_valid;
    uint16_t  reserved16;
    uint64_t  time_us;
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
    uint64_t  time_us;
    uint8_t   state;
    uint8_t   reserved8b[1];
    uint16_t  flags;
    float     stroke_m;
    float     last_force_n;
} LRMH_HAL_Health_t;

typedef struct {
    float     target_stroke_m;
    float     max_force_n;
    float     max_speed_mps;
    uint32_t  reserved;
} LRMH_HAL_PositionCmd_t;

typedef struct {
    float     target_force_n;
    float     max_stroke_m;
    float     ramp_rate_nps;
    uint32_t  reserved;
} LRMH_HAL_ForceCmd_t;

/* ── No-op stub implementations ───────────────────────────────────── */

/* Tests can override these per case by setting the corresponding
 * counter or return-value variables. Default behavior: return OK,
 * count the call, latch the most-recent command struct. */
extern int32_t lrmh_hal_stub_init_count;
extern int32_t lrmh_hal_stub_init_return;
extern int32_t lrmh_hal_stub_estop_count;
extern int32_t lrmh_hal_stub_retract_count;
extern int32_t lrmh_hal_stub_retract_return;
extern int32_t lrmh_hal_stub_hold_count;
extern int32_t lrmh_hal_stub_position_count;
extern int32_t lrmh_hal_stub_force_count;
extern LRMH_HAL_PositionCmd_t lrmh_hal_stub_last_position;
extern LRMH_HAL_ForceCmd_t    lrmh_hal_stub_last_force;
extern LRMH_HAL_SensorData_t  lrmh_hal_stub_inject_sensor;
extern LRMH_HAL_Health_t      lrmh_hal_stub_inject_health;

static inline int32_t LRMH_HAL_Init(void) {
    lrmh_hal_stub_init_count++;
    return lrmh_hal_stub_init_return;
}
static inline int32_t LRMH_HAL_ReadSensors(LRMH_HAL_SensorData_t *out) {
    if (out) *out = lrmh_hal_stub_inject_sensor;
    return LRMH_HAL_OK;
}
static inline int32_t LRMH_HAL_GetHealth(LRMH_HAL_Health_t *out) {
    if (out) *out = lrmh_hal_stub_inject_health;
    return LRMH_HAL_OK;
}
static inline int32_t LRMH_HAL_CommandPosition(const LRMH_HAL_PositionCmd_t *cmd) {
    if (cmd) lrmh_hal_stub_last_position = *cmd;
    lrmh_hal_stub_position_count++;
    return LRMH_HAL_OK;
}
static inline int32_t LRMH_HAL_CommandForce(const LRMH_HAL_ForceCmd_t *cmd) {
    if (cmd) lrmh_hal_stub_last_force = *cmd;
    lrmh_hal_stub_force_count++;
    return LRMH_HAL_OK;
}
static inline int32_t LRMH_HAL_Retract(void) {
    lrmh_hal_stub_retract_count++;
    return lrmh_hal_stub_retract_return;
}
static inline int32_t LRMH_HAL_Hold(void) {
    lrmh_hal_stub_hold_count++; return LRMH_HAL_OK;
}
static inline int32_t LRMH_HAL_EStop(void) {
    lrmh_hal_stub_estop_count++; return LRMH_HAL_OK;
}
static inline int32_t LRMH_HAL_StepSim(double dt) {
    (void)dt; return LRMH_HAL_OK;
}

#endif /* LRMH_HAL_H */
