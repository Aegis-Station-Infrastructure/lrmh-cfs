/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_hal_sim.c
 * @brief LRMH HAL sim — standalone UDP-served physics simulator
 *
 * Standalone executable that runs as a separate process from the
 * cFS image. The cFS image's hal/client/ UDP backend sends requests
 * here; this process dispatches them to the physics
 * models under sim/ and returns results.
 *
 * Per ADR-0001 (apps unchanged) and ADR-0002 (regolith opaque),
 * This file is the bridge between the sim model API
 * (Actuator_*, Regolith_*, Sensor_*)
 * and the LRMH HAL contract (LRMH_HAL_SensorData_t,
 * LRMH_HAL_Health_t, LRMH_HAL_PositionCmd_t, etc.).
 *
 * UDP envelope: fixed-size request + response with sequence
 * numbers, matching hal/client/lrmh_hal_client.c on the cFS side.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>

#include "lrmh_hal.h"

/* Sim model API (ADR-0002) */
#include "actuator_model.h"
#include "regolith_model.h"
#include "sensor_model.h"

/* ============================================================
 * Configuration
 *
 * Sim-only tunables. The tool-tip area belongs in the future
 * lrmh_tool_tbl.json (per ADR-0010 §"Tool table"); for now it's
 * a single nominal value used for the regolith reaction-force
 * computation. The actuator-command timeouts are sim-side
 * defaults — the HAL contract takes a per-command timeout, but
 * this bridge passes a fixed value into the sim model. Real
 * flight/bench backends will use the
 * caller-supplied timeout from the LRMH_HAL_*Cmd_t structs.
 * ============================================================ */
#define LRMH_HAL_SIM_PORT             17011
#define LRMH_HAL_SIM_TICK_NSEC        10000000L  /* 10 ms = 100 Hz */
#define LRMH_TOOL_TIP_AREA_M2         0.0001f    /* 1 cm² nominal */
#define LRMH_SIM_MAX_FORCE_N          650.0f     /* LRMH_MAX_FORCE_N (ADR-0012); a retract clamps at 1.5x */

/* Default timeouts passed into the actuator model.
 * The 60 s position/force ceiling is derived from the longest
 * expected single-axis traverse at the slowest clamp speed.
 * Retract uses 30 s because home travel is bounded by stroke. */
#define LRMH_SIM_DEFAULT_MOVE_TIMEOUT_S      60.0f
#define LRMH_SIM_DEFAULT_RETRACT_TIMEOUT_S   30.0f
#define LRMH_SIM_DEFAULT_RETRACT_RATE_MPS    0.01f

/* A force command's target is ramped at its ramp_rate_nps in
 * physics_tick and the actuator tracks it at its top speed
 * (LRMH_SIM_DEFAULT_RETRACT_RATE_MPS, the model's MaxVelocity). */

/* ============================================================
 * UDP envelope
 * ============================================================ */
typedef enum {
    HAL_REQ_INIT             = 0,
    HAL_REQ_READ_SENSORS     = 1,
    HAL_REQ_GET_HEALTH       = 2,
    HAL_REQ_CMD_POSITION     = 3,
    HAL_REQ_CMD_FORCE        = 4,
    HAL_REQ_RETRACT          = 5,
    HAL_REQ_HOLD             = 6,
    HAL_REQ_ESTOP            = 7,
    HAL_REQ_STEP_SIM         = 8,
    HAL_REQ_INJECT_FAULT     = 9,   /* sim-only; bench/flight ack as no-op */
    HAL_REQ_SET_SITE         = 10,  /* sim-only: draw and load a regolith site */
    HAL_REQ_ACTUATOR_FAULT   = 11,  /* sim-only: a fault in the actuator model itself */
    HAL_REQ_SIM_FAULT        = 12   /* sim-only: a fault in the sensor or the ground (frozen sensor, bad zero, hard layer, tool absent) */
} HalReqType_t;

/* HAL_REQ_SIM_FAULT payload, the same shape as HAL_REQ_INJECT_FAULT's:
 * kind 0 = the force sensor freezes at its last reading (still reported valid);
 *      1 = a sudden hard layer: the ground below the tip's current depth resists `value` times more;
 *      2 = a bad zero calibration: the force reading carries an offset of `value` N;
 *      3 = the tool is not present: the ground gives no reaction at all.
 * duration_ms 0 = until cleared with kind 0xFFFF. */
typedef struct {
    uint32_t  kind;
    float     value;
    uint32_t  duration_ms;
    uint32_t  reserved;
} HalSimFaultPayload_t;
#define LRMH_SIM_FAULT_NONE  0xFFFFu

/* HAL_REQ_SET_SITE payload: the regolith preset (RegolithPreset_t) and the
 * seed the site is drawn from. The same preset and seed draw the same site. */
typedef struct {
    uint32_t  preset;
    uint32_t  seed;
} HalSetSitePayload_t;

/* HAL_REQ_ACTUATOR_FAULT payload: a fault in the actuator physics, not a
 * sensor override. kind: 0 = stall (no motion, current pinned above the
 * stall threshold), 1 = overcurrent, 2 = encoder loss. active 0 clears it. */
typedef struct {
    uint32_t  kind;
    uint32_t  active;
} HalActuatorFaultPayload_t;

/* Fault-injection kinds (HAL_REQ_INJECT_FAULT payload).
 * Each kind overrides one or more sensor channels on the next
 * READ_SENSORS calls so the corresponding LC watchpoint trips and
 * the per-fault-flag actionpoint dispatches the right RTS. Used
 * to validate the FDIR table set end-to-end against fault stimuli
 * without needing a real bench fixture. */
typedef enum {
    LRMH_INJECT_NONE          = 0,
    LRMH_INJECT_FORCE_SAT     = 1,    /* force_n   > LC FORCE threshold */
    LRMH_INJECT_OVERCURRENT   = 2,    /* current_a > LC CURRENT threshold */
    LRMH_INJECT_HARD_STOP     = 3,    /* stroke_m  > LC STROKE threshold */
    LRMH_INJECT_THERMAL       = 4,    /* temp_c    > LC TEMP threshold */
    LRMH_INJECT_BUS_VOLTAGE   = 5,    /* bus_v     < LC BUS_LOW threshold */
    LRMH_INJECT_FORCE_RATE    = 6     /* dF/dt     > LC FORCE_RATE (toggle) */
} HalInjectKind_t;

typedef struct {
    uint32_t  kind;             /* HalInjectKind_t */
    float     override_value;   /* explicit override; 0 = past-threshold default */
    uint32_t  duration_ms;      /* 0 = persistent until INJECT_NONE */
    uint32_t  reserved;
} HalInjectFaultPayload_t;

typedef struct {
    uint32_t  magic;
    uint32_t  seq_no;
    uint32_t  req_type;
    uint32_t  payload_len;
    uint8_t   payload[256];
} HalRequest_t;

typedef struct {
    uint32_t  magic;
    uint32_t  seq_no;
    uint32_t  req_type;
    int32_t   status;
    uint32_t  payload_len;
    uint8_t   payload[256];
} HalResponse_t;

#define LRMH_HAL_MAGIC  0x4C524D48u

/* ============================================================
 * Sim state
 * ============================================================ */
static struct {
    int       udp_fd;
    bool      running;
    uint64_t  time_us;
    struct timespec last_tick;
    bool      initialized;

    /* The safety clamps of the command in progress (ADR-0010): a
     * position move stops where the force reaches max_force_n, a force
     * command where the stroke reaches max_stroke_m, a retract where the
     * force reaches 1.5 x LRMH_MAX_FORCE_N. The actuator holds there (not
     * retracted) and the flag says why it stopped. 0 = no clamp. */
    float     max_force_n;
    float     max_stroke_m;
    uint16_t  cmd_flags;            /* LRMH_HAL_FLAG_* of the move in progress */

    /* A force command ramps its target at ramp_rate_nps (ADR-0010); the
     * actuator tracks the moving target at its own speed. 0 rate = step. */
    float     force_final_n;
    float     force_rate_nps;
    float     force_now_n;

    /* A site set by HAL_REQ_SET_SITE survives HAL_REQ_INIT (the flight
     * software inits on INIT_INSTRUMENT and on every recovery); without
     * this every run was the default draw. */
    bool      site_set;

    /* Encoder loss (HAL_REQ_ACTUATOR_FAULT kind 2): the depth channel
     * freezes at its last reading and is reported invalid. */
    float     last_depth_m;

    /* HAL_REQ_SIM_FAULT state */
    uint32_t  sim_fault_kind;       /* LRMH_SIM_FAULT_NONE when none */
    float     sim_fault_value;
    uint64_t  sim_fault_until_us;   /* 0 = until cleared */
    float     frozen_force_n;       /* kind 0: the reading it froze at (NaN until captured) */
    float     hard_layer_from_m;    /* kind 1: the depth the layer begins at */

    /* Fault-injection state (HAL_REQ_INJECT_FAULT). Set by the
     * inject handler, consumed by hal_read_sensors when applying
     * overrides. inject_until_us = 0 means persistent until cleared
     * by INJECT_NONE; otherwise auto-clears at that sim time. */
    uint32_t  inject_kind;          /* HalInjectKind_t */
    float     inject_value;
    uint64_t  inject_until_us;
    float     inject_ramp_force_n;      /* FORCE_RATE: current ramp value */
} g_sim;

/* ============================================================
 * Time helpers
 * ============================================================ */
static uint64_t monotonic_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

/* ============================================================
 * Physics tick — advance the actuator + regolith models
 *
 * Each tick:
 *   1. Read current actuator state
 *   2. Compute regolith reaction force at the current depth
 *   3. Step the actuator with that external force
 *   4. Advance the sim clock
 * ============================================================ */
static void physics_tick(double dt_s)
{
    if (!g_sim.initialized) return;

    ActuatorState_t actuator_state;
    Actuator_GetState(&actuator_state);

    /* Compute regolith reaction at current penetration depth */
    RegolithResponse_t regolith;
    memset(&regolith, 0, sizeof(regolith));
    Regolith_GetPenetrationResponse(actuator_state.Position_m,
                                    LRMH_TOOL_TIP_AREA_M2,
                                    actuator_state.Velocity_mps,
                                    &regolith);

    /* Ramp a force command's target toward its final value */
    if (actuator_state.ControlMode == 1 && g_sim.force_now_n < g_sim.force_final_n) {
        g_sim.force_now_n += g_sim.force_rate_nps * (float)dt_s;
        if (g_sim.force_now_n > g_sim.force_final_n) g_sim.force_now_n = g_sim.force_final_n;
        Actuator_SetForceTarget(g_sim.force_now_n);
    }

    /* A fault in the ground (HAL_REQ_SIM_FAULT): a hard layer, or no tool to feel it */
    if (g_sim.sim_fault_kind != LRMH_SIM_FAULT_NONE && g_sim.sim_fault_until_us > 0 &&
        g_sim.time_us >= g_sim.sim_fault_until_us) {
        g_sim.sim_fault_kind = LRMH_SIM_FAULT_NONE;
    }
    if (g_sim.sim_fault_kind == 1 && actuator_state.Position_m >= g_sim.hard_layer_from_m) {
        regolith.ReactionForce_N *= g_sim.sim_fault_value;
    } else if (g_sim.sim_fault_kind == 3) {
        regolith.ReactionForce_N = 0.0f;
    }

    /* Step the actuator with the reaction force */
    Actuator_Step(dt_s, regolith.ReactionForce_N);

    /* A force command is not complete while its target is still ramping,
     * even when the tip happens to bear the target of the moment */
    Actuator_GetState(&actuator_state);
    if (actuator_state.ControlMode == 1 && g_sim.force_now_n < g_sim.force_final_n) {
        Actuator_SetForceTarget(g_sim.force_now_n);
        Actuator_GetState(&actuator_state);
    }

    /* The clamps (ADR-0010): stop and hold where a limit is reached */
    if (!actuator_state.CommandComplete) {
        if (actuator_state.ControlMode == 0 && g_sim.max_force_n > 0.0f &&
            actuator_state.AppliedForce_N >= g_sim.max_force_n) {
            Actuator_CmdHold();
            g_sim.cmd_flags |= LRMH_HAL_FLAG_STOPPED_AT_FORCE_CLAMP;
        } else if (actuator_state.ControlMode == 1 && g_sim.max_stroke_m > 0.0f &&
                   actuator_state.Position_m >= g_sim.max_stroke_m) {
            Actuator_CmdHold();
            g_sim.cmd_flags |= LRMH_HAL_FLAG_STOPPED_AT_STROKE_CLAMP;
        } else if (actuator_state.ControlMode == 2 &&
                   actuator_state.AppliedForce_N >= 1.5f * LRMH_SIM_MAX_FORCE_N) {
            Actuator_CmdHold();
            g_sim.cmd_flags |= LRMH_HAL_FLAG_RETRACT_BLOCKED;
        }
    } else if (actuator_state.ControlMode == 0) {
        g_sim.cmd_flags |= LRMH_HAL_FLAG_POSITION_REACHED;
    } else if (actuator_state.ControlMode == 1) {
        g_sim.cmd_flags |= LRMH_HAL_FLAG_FORCE_REACHED;
    }

    g_sim.time_us += (uint64_t)(dt_s * 1e6);
}

/* ============================================================
 * Read sensors — bridge from the sim models to LRMH_HAL_SensorData_t
 * ============================================================ */
static int32 hal_read_sensors(LRMH_HAL_SensorData_t *out)
{
    if (!out || !g_sim.initialized) return LRMH_HAL_NOT_INITIALIZED;

    ActuatorState_t actuator_state;
    if (Actuator_GetState(&actuator_state) != 0) return LRMH_HAL_ERROR;

    /* Apply the sensor model to the true actuator state */
    SensorReadings_t readings;
    memset(&readings, 0, sizeof(readings));
    double sim_time_s = (double)g_sim.time_us / 1e6;
    Sensor_ApplyModel(sim_time_s,
                      actuator_state.AppliedForce_N,
                      actuator_state.Position_m,    /* depth ≈ stroke for now */
                      actuator_state.Position_m,
                      actuator_state.MotorCurrent_A,
                      actuator_state.Temperature_C,
                      &readings);

    memset(out, 0, sizeof(*out));
    out->valid    = true;
    out->time_us  = g_sim.time_us;
    out->force_n  = readings.Force_N;

    /* A fault in the force sensor (HAL_REQ_SIM_FAULT): frozen, or a bad zero */
    if (g_sim.sim_fault_kind == 0) {
        if (isnan(g_sim.frozen_force_n)) g_sim.frozen_force_n = out->force_n;   /* NaN marks "not captured yet" */
        out->force_n = g_sim.frozen_force_n;
    } else if (g_sim.sim_fault_kind == 2) {
        out->force_n += g_sim.sim_fault_value;
    }
    out->depth_m  = readings.Depth_m;
    out->stroke_m = readings.StrokePosition_m;
    out->current_a = readings.MotorCurrent_A;
    out->bus_v    = 28.0f;  /* sim doesn't model bus voltage */
    out->temp_c   = readings.Temperature_C;

    /* Encoder loss: the tip's depth reading freezes and its channel goes
     * invalid; the actuator's own stroke encoder is unaffected */
    bool stroke_valid = readings.DepthValid;
    if (actuator_state.InjectEncoderLoss) {
        out->depth_m = g_sim.last_depth_m;
        readings.DepthValid = 0;
    } else {
        g_sim.last_depth_m = out->depth_m;
    }

    /* Per-channel valid mask from the sim's per-channel valid bits */
    out->channel_valid = 0;
    if (readings.ForceValid)   out->channel_valid |= LRMH_HAL_CH_BIT(LRMH_HAL_CH_FORCE);
    if (readings.DepthValid)   out->channel_valid |= LRMH_HAL_CH_BIT(LRMH_HAL_CH_DEPTH);
    if (stroke_valid)          out->channel_valid |= LRMH_HAL_CH_BIT(LRMH_HAL_CH_STROKE);
    if (readings.CurrentValid) out->channel_valid |= LRMH_HAL_CH_BIT(LRMH_HAL_CH_CURRENT);
    out->channel_valid |= LRMH_HAL_CH_BIT(LRMH_HAL_CH_BUS_V);  /* always */
    if (readings.TempValid)    out->channel_valid |= LRMH_HAL_CH_BIT(LRMH_HAL_CH_TEMP);

    /* Fault-injection overrides (HAL_REQ_INJECT_FAULT, sim-only).
     * Applied after the normal sensor model so the override values
     * are what lrmh_sense publishes to the SB and the LC
     * watchpoints evaluate. The override default values are
     * deliberately past the LC margin thresholds in
     * lrmh_platform_cfg.h so the watchpoint trips on the next
     * sample. */
    if (g_sim.inject_kind != LRMH_INJECT_NONE) {
        if (g_sim.inject_until_us > 0 &&
            g_sim.time_us >= g_sim.inject_until_us) {
            g_sim.inject_kind = LRMH_INJECT_NONE;
        }
        switch (g_sim.inject_kind) {
        case LRMH_INJECT_FORCE_SAT:
            out->force_n = (g_sim.inject_value > 0.0f)
                         ? g_sim.inject_value : 700.0f;
            break;
        case LRMH_INJECT_OVERCURRENT:
            out->current_a = (g_sim.inject_value > 0.0f)
                           ? g_sim.inject_value : 5.5f;
            break;
        case LRMH_INJECT_HARD_STOP:
            out->stroke_m = (g_sim.inject_value > 0.0f)
                          ? g_sim.inject_value : 0.34f;
            break;
        case LRMH_INJECT_THERMAL:
            out->temp_c = (g_sim.inject_value > 0.0f)
                        ? g_sim.inject_value : 60.0f;
            break;
        case LRMH_INJECT_BUS_VOLTAGE:
            out->bus_v = (g_sim.inject_value > 0.0f)
                       ? g_sim.inject_value : 21.0f;
            break;
        case LRMH_INJECT_FORCE_RATE:
            /* Ramp the force at a sustained 1500 N/s (15 N per 10 ms
             * sample) so lrmh_sense's computed force_rate_nps sits
             * above the LC FORCE_RATE threshold on consecutive
             * evaluations; the actionpoint needs several consecutive
             * fails before it fires. (A two-value toggle gave an
             * alternating sign, so LC's 10 Hz sampling saw either all
             * passes or all fails depending on phase.) Wrap back to
             * the floor when the ramp reaches the force envelope. */
            if (g_sim.inject_ramp_force_n < 50.0f ||
                g_sim.inject_ramp_force_n > 600.0f) {
                g_sim.inject_ramp_force_n = 50.0f;
            }
            out->force_n = g_sim.inject_ramp_force_n;
            g_sim.inject_ramp_force_n += 15.0f;
            break;
        default:
            break;
        }
    }

    return LRMH_HAL_OK;
}

/* ============================================================
 * Get health — bridge from ActuatorState_t → LRMH_HAL_Health_t
 * ============================================================ */
static int32 hal_get_health(LRMH_HAL_Health_t *out)
{
    if (!out || !g_sim.initialized) return LRMH_HAL_NOT_INITIALIZED;

    ActuatorState_t st;
    if (Actuator_GetState(&st) != 0) return LRMH_HAL_ERROR;

    memset(out, 0, sizeof(*out));
    out->valid    = true;
    out->time_us  = g_sim.time_us;
    out->stroke_m = st.Position_m;
    out->last_force_n = st.AppliedForce_N;

    /* Map the model's ControlMode + status flags to LRMH_HAL_State_t */
    if (st.IsEStoped) {
        out->state = LRMH_HAL_STATE_ESTOP;
    } else if (st.IsStalled) {
        out->state = LRMH_HAL_STATE_FAULT;
    } else if (st.CommandComplete) {
        /* Hold mode (3) maps to HOLDING; everything else done → IDLE */
        out->state = (st.ControlMode == 3) ? LRMH_HAL_STATE_HOLDING
                                           : LRMH_HAL_STATE_IDLE;
    } else {
        out->state = LRMH_HAL_STATE_MOVING;
    }

    /* The move's own flags: reached, or stopped at a clamp */
    out->flags |= g_sim.cmd_flags;
    if (st.InjectEncoderLoss) out->flags |= LRMH_HAL_FLAG_ENCODER_LOSS;

    /* Pack hard-stop and stall flags */
    if (st.AtHardStopMin || st.AtHardStopMax) {
        out->flags |= LRMH_HAL_FLAG_HARD_STOP_HIT;
    }
    if (st.IsStalled) {
        out->flags |= LRMH_HAL_FLAG_RETRACT_BLOCKED;
    }

    return LRMH_HAL_OK;
}

/* ============================================================
 * Request dispatch
 * ============================================================ */
static void handle_request(const HalRequest_t *req, HalResponse_t *resp)
{
    resp->magic       = LRMH_HAL_MAGIC;
    resp->seq_no      = req->seq_no;
    resp->req_type    = req->req_type;
    resp->status      = LRMH_HAL_OK;
    resp->payload_len = 0;

    switch ((HalReqType_t)req->req_type) {

    case HAL_REQ_INIT: {
        RegolithProfile_t site = *Regolith_GetCurrentProfile();
        Actuator_Init();
        Regolith_Init();
        g_sim.sim_fault_kind = LRMH_SIM_FAULT_NONE;
        if (g_sim.site_set) Regolith_LoadProfile(&site);
        Sensor_Init();
        g_sim.initialized = true;
        break;
    }

    case HAL_REQ_READ_SENSORS: {
        LRMH_HAL_SensorData_t data;
        resp->status = hal_read_sensors(&data);
        if (resp->status == LRMH_HAL_OK) {
            memcpy(resp->payload, &data, sizeof(data));
            resp->payload_len = sizeof(data);
        }
        break;
    }

    case HAL_REQ_GET_HEALTH: {
        LRMH_HAL_Health_t health;
        resp->status = hal_get_health(&health);
        if (resp->status == LRMH_HAL_OK) {
            memcpy(resp->payload, &health, sizeof(health));
            resp->payload_len = sizeof(health);
        }
        break;
    }

    case HAL_REQ_CMD_POSITION: {
        if (!g_sim.initialized) { resp->status = LRMH_HAL_NOT_INITIALIZED; break; }
        LRMH_HAL_PositionCmd_t cmd;
        memcpy(&cmd, req->payload, sizeof(cmd));
        Actuator_CmdPosition(cmd.target_stroke_m,
                             cmd.max_speed_mps,
                             LRMH_SIM_DEFAULT_MOVE_TIMEOUT_S);
        /* No clamp given means the envelope, never no clamp at all */
        g_sim.max_force_n  = (cmd.max_force_n > 0.0f) ? cmd.max_force_n : LRMH_SIM_MAX_FORCE_N;
        g_sim.max_stroke_m = 0.0f;
        g_sim.cmd_flags    = 0;
        break;
    }

    case HAL_REQ_CMD_FORCE: {
        if (!g_sim.initialized) { resp->status = LRMH_HAL_NOT_INITIALIZED; break; }
        LRMH_HAL_ForceCmd_t cmd;
        memcpy(&cmd, req->payload, sizeof(cmd));
        /* The target ramps at ramp_rate_nps from the force the tip bears
         * now (physics_tick moves it); the actuator tracks it at its own
         * top speed. The move's timeout allows for the ramp. */
        ActuatorState_t st;
        Actuator_GetState(&st);
        g_sim.force_final_n = cmd.target_force_n;
        g_sim.force_rate_nps = (cmd.ramp_rate_nps > 0.0f) ? cmd.ramp_rate_nps : 0.0f;
        g_sim.force_now_n = (g_sim.force_rate_nps > 0.0f && st.AppliedForce_N < cmd.target_force_n)
                          ? (st.AppliedForce_N > 0.0f ? st.AppliedForce_N : 0.0f) : cmd.target_force_n;
        float timeout = LRMH_SIM_DEFAULT_MOVE_TIMEOUT_S
                      + ((g_sim.force_rate_nps > 0.0f) ? (cmd.target_force_n - g_sim.force_now_n) / g_sim.force_rate_nps : 0.0f);
        Actuator_CmdForce(g_sim.force_now_n, LRMH_SIM_DEFAULT_RETRACT_RATE_MPS, timeout);
        g_sim.max_force_n  = 0.0f;
        g_sim.max_stroke_m = cmd.max_stroke_m;
        g_sim.cmd_flags    = 0;
        break;
    }

    case HAL_REQ_RETRACT:
        if (!g_sim.initialized) { resp->status = LRMH_HAL_NOT_INITIALIZED; break; }
        Actuator_CmdRetract(LRMH_SIM_DEFAULT_RETRACT_RATE_MPS,
                            LRMH_SIM_DEFAULT_RETRACT_TIMEOUT_S);
        g_sim.max_force_n = 0.0f; g_sim.max_stroke_m = 0.0f; g_sim.cmd_flags = 0;
        break;

    case HAL_REQ_HOLD:
        if (!g_sim.initialized) { resp->status = LRMH_HAL_NOT_INITIALIZED; break; }
        Actuator_CmdHold();
        g_sim.cmd_flags = 0;
        break;

    case HAL_REQ_ESTOP:
        if (!g_sim.initialized) { resp->status = LRMH_HAL_NOT_INITIALIZED; break; }
        Actuator_EStop();
        g_sim.cmd_flags = 0;
        break;

    case HAL_REQ_STEP_SIM: {
        double dt;
        memcpy(&dt, req->payload, sizeof(dt));
        physics_tick(dt);
        break;
    }

    case HAL_REQ_INJECT_FAULT: {
        HalInjectFaultPayload_t inj;
        memcpy(&inj, req->payload, sizeof(inj));
        g_sim.inject_kind  = inj.kind;
        g_sim.inject_value = inj.override_value;
        if (inj.duration_ms > 0) {
            g_sim.inject_until_us =
                g_sim.time_us + (uint64_t)inj.duration_ms * 1000ULL;
        } else {
            g_sim.inject_until_us = 0;   /* persistent until INJECT_NONE */
        }
        if (inj.kind != LRMH_INJECT_FORCE_RATE) {
            g_sim.inject_ramp_force_n = 0.0f;
        }
        fprintf(stderr,
                "lrmh_hal_sim: inject_fault kind=%u value=%.3f duration_ms=%u\n",
                inj.kind, (double)inj.override_value, inj.duration_ms);
        break;
    }

    case HAL_REQ_SET_SITE: {
        HalSetSitePayload_t site;
        memcpy(&site, req->payload, sizeof(site));
        if (site.preset >= REGOLITH_PRESET_COUNT ||
            Regolith_SetSite((RegolithPreset_t)site.preset, site.seed) != 0) {
            resp->status = LRMH_HAL_REJECTED_BAD_PARAM;
            break;
        }
        g_sim.site_set = true;
        const RegolithProfile_t *p = Regolith_GetCurrentProfile();
        fprintf(stderr, "lrmh_hal_sim: site %s seed=%u layers=%u clasts=%u g=%.2f\n",
                Regolith_PresetName((RegolithPreset_t)site.preset), site.seed,
                p->NumLayers, p->NumClasts, (double)p->Gravity_mps2);
        break;
    }

    case HAL_REQ_SIM_FAULT: {
        HalSimFaultPayload_t f;
        memcpy(&f, req->payload, sizeof(f));
        if (f.kind != LRMH_SIM_FAULT_NONE && f.kind > 3) { resp->status = LRMH_HAL_REJECTED_BAD_PARAM; break; }
        ActuatorState_t st;
        Actuator_GetState(&st);
        g_sim.sim_fault_kind = f.kind;
        g_sim.sim_fault_value = f.value;
        g_sim.sim_fault_until_us = f.duration_ms ? g_sim.time_us + (uint64_t)f.duration_ms * 1000ULL : 0;
        g_sim.frozen_force_n = 0.0f / 0.0f;                     /* NaN: captured at the next read */
        g_sim.hard_layer_from_m = st.Position_m;
        fprintf(stderr, "lrmh_hal_sim: sim_fault kind=%u value=%.3f duration_ms=%u from_depth=%.4f\n",
                f.kind, (double)f.value, f.duration_ms, (double)st.Position_m);
        break;
    }

    case HAL_REQ_ACTUATOR_FAULT: {
        HalActuatorFaultPayload_t f;
        memcpy(&f, req->payload, sizeof(f));
        if (Actuator_InjectFault((uint8)f.kind, (uint8)(f.active ? 1 : 0)) != 0) {
            resp->status = LRMH_HAL_REJECTED_BAD_PARAM;
            break;
        }
        fprintf(stderr, "lrmh_hal_sim: actuator_fault kind=%u active=%u\n", f.kind, f.active);
        break;
    }

    default:
        resp->status = LRMH_HAL_REJECTED_BAD_PARAM;
        break;
    }
}

/* ============================================================
 * UDP listen + dispatch loop
 * ============================================================ */
static int udp_open(uint16_t port)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return -1; }
    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    struct sockaddr_in addr = { 0 };
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(fd);
        return -1;
    }
    return fd;
}

static void on_signal(int sig) { (void)sig; g_sim.running = false; }

int main(int argc, char **argv)
{
    (void)argc; (void)argv;

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    g_sim.udp_fd = udp_open(LRMH_HAL_SIM_PORT);
    if (g_sim.udp_fd < 0) return 1;

    /* Auto-init so the sim is ready before the first cFS request */
    g_sim.sim_fault_kind = LRMH_SIM_FAULT_NONE;
    Actuator_Init();
    Regolith_Init();
    Sensor_Init();
    g_sim.initialized = true;
    g_sim.running = true;
    clock_gettime(CLOCK_MONOTONIC, &g_sim.last_tick);
    g_sim.time_us = monotonic_us();

    fprintf(stderr,
        "lrmh_hal_sim: listening on UDP :%d, physics tick @ 100 Hz\n",
        LRMH_HAL_SIM_PORT);

    HalRequest_t  req;
    HalResponse_t resp;
    struct sockaddr_in client;
    socklen_t client_len;

    while (g_sim.running) {
        struct pollfd pfd = { .fd = g_sim.udp_fd, .events = POLLIN };
        int pr = poll(&pfd, 1, 1);

        /* Drive physics on the wall clock */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long elapsed_ns = (now.tv_sec - g_sim.last_tick.tv_sec) * 1000000000L
                        + (now.tv_nsec - g_sim.last_tick.tv_nsec);
        if (elapsed_ns >= LRMH_HAL_SIM_TICK_NSEC) {
            physics_tick(LRMH_HAL_SIM_TICK_NSEC / 1e9);
            g_sim.last_tick = now;
        }

        if (pr > 0 && (pfd.revents & POLLIN)) {
            client_len = sizeof(client);
            ssize_t n = recvfrom(g_sim.udp_fd, &req, sizeof(req), 0,
                                 (struct sockaddr *)&client, &client_len);
            if (n < (ssize_t)sizeof(req) - (ssize_t)sizeof(req.payload)) continue;
            if (req.magic != LRMH_HAL_MAGIC) continue;

            handle_request(&req, &resp);
            sendto(g_sim.udp_fd, &resp, sizeof(resp), 0,
                   (struct sockaddr *)&client, client_len);
        }
    }

    fprintf(stderr, "lrmh_hal_sim: shutting down\n");
    close(g_sim.udp_fd);
    return 0;
}
