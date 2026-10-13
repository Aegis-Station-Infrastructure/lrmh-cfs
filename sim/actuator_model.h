/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file actuator_model.h
 * @brief Simulated linear actuator model for LRMH
 *
 * Simulates:
 *   - Stroke position, velocity, acceleration
 *   - Response lag / dynamics
 *   - Force-control and position-control modes
 *   - Hard stops at physical limits
 *   - Motor current draw
 *   - Stall / jam detection
 *   - E-stop behavior
 */

#ifndef ACTUATOR_MODEL_H
#define ACTUATOR_MODEL_H

#include "common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Actuator Parameters (configurable per mechanism)
 * ============================================================ */
typedef struct
{
    float   MaxStroke_m;            /* Full travel range             */
    float   MaxVelocity_mps;       /* Peak velocity                 */
    float   MaxForce_N;            /* Peak force capability         */
    float   MaxCurrent_A;          /* Motor current limit           */
    float   Backlash_m;            /* Mechanical backlash           */
    float   ResponseLag_s;         /* First-order lag time constant */
    float   StallCurrentThresh_A;  /* Current above this = stall   */
    float   HardStopStiffness;     /* Spring constant at hard stop  */
    float   HomingPosition_m;      /* Retract-to position           */
} ActuatorParams_t;

/* ============================================================
 * Actuator Internal State
 * ============================================================ */
typedef struct
{
    /* Current physical state */
    float   Position_m;
    float   Velocity_mps;
    float   AppliedForce_N;
    float   MotorCurrent_A;
    float   Temperature_C;

    /* Command targets */
    float   CmdTargetPosition_m;
    float   CmdTargetForce_N;
    float   CmdRateLimit_mps;
    float   CmdTimeout_s;
    float   CmdElapsed_s;

    /* Mode and status */
    uint8   ControlMode;            /* 0=position, 1=force, 2=retract, 3=hold, 4=estop */
    uint8   AtHardStopMin;
    uint8   AtHardStopMax;
    uint8   IsStalled;
    uint8   IsEStoped;
    uint8   CommandComplete;
    uint8   Spare[2];

    /* Fault injection */
    uint8   InjectStall;
    uint8   InjectOvercurrent;
    uint8   InjectEncoderLoss;
    uint8   Spare2;
} ActuatorState_t;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief Initialize actuator model with default parameters
 */
int32 Actuator_Init(void);

/**
 * @brief Set custom actuator parameters
 */
int32 Actuator_SetParams(const ActuatorParams_t *Params);

/**
 * @brief Get current actuator state
 */
int32 Actuator_GetState(ActuatorState_t *State);

/**
 * @brief Command: move to target position
 */
int32 Actuator_CmdPosition(float TargetPos_m, float RateLimit_mps, float Timeout_s);

/**
 * @brief Command: apply target force (ramped at RateLimit)
 */
int32 Actuator_CmdForce(float TargetForce_N, float RateLimit_mps, float Timeout_s);

/**
 * @brief Command: retract to home position
 */
int32 Actuator_CmdRetract(float RateLimit_mps, float Timeout_s);

/**
 * @brief Command: hold current position
 */
int32 Actuator_CmdHold(void);

/* Move the force target of the command in progress without restarting it:
 * the HAL sim ramps a force command at its ramp rate. The command is not
 * complete while the target is still being moved. */
int32 Actuator_SetForceTarget(float TargetForce_N);

/**
 * @brief Emergency stop
 */
int32 Actuator_EStop(void);

/**
 * @brief Clear e-stop condition
 */
int32 Actuator_ClearEStop(void);

/**
 * @brief Step the actuator physics by dt seconds
 * @param[in] dt                  Time step
 * @param[in] ExternalForce_N     Resistance from soil model
 */
int32 Actuator_Step(double dt, float ExternalForce_N);

/**
 * @brief Inject a fault condition for testing
 * @param[in] FaultType  0=stall, 1=overcurrent, 2=encoder_loss
 * @param[in] Active     1=inject, 0=clear
 */
int32 Actuator_InjectFault(uint8 FaultType, uint8 Active);

#ifdef __cplusplus
}
#endif

#endif /* ACTUATOR_MODEL_H */
