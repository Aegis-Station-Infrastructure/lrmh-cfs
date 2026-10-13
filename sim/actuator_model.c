/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file actuator_model.c
 * @brief Simulated linear actuator dynamics
 *
 * Physics:
 *   - First-order velocity response toward commanded rate
 *   - Position integrates velocity
 *   - Force mode: velocity proportional to (target - actual) force error
 *   - Hard stops at 0 and MaxStroke with spring-like reaction
 *   - Motor current ~ force / force_constant + velocity * back_emf
 *   - Stall: current exceeds threshold while velocity near zero
 *   - Temperature: simple thermal model (current^2 * resistance * dt)
 */

#include "actuator_model.h"
#include <math.h>
#include <string.h>

/* ============================================================
 * Module State
 * ============================================================ */
static ActuatorParams_t g_Params;
static ActuatorState_t  g_State;
static uint8            g_Initialized = 0;

/* Motor constants (representative small linear actuator) */
#define FORCE_CONSTANT     140.0f   /* N/A  — force per amp; 650 N continuous at 5 A (ADR-0012), so the
                                        600 N force watchpoint trips before the 4.5 A current one */
#define BACK_EMF_CONST      0.02f   /* A/(m/s) — current from motion  */
#define THERMAL_RESISTANCE  0.5f    /* degC per watt                   */
#define THERMAL_CAPACITANCE 200.0f  /* joules per degC                 */
#define AMBIENT_TEMP_C      20.0f   /* Desktop sim ambient             */

/* ============================================================
 * Initialization
 * ============================================================ */
int32 Actuator_Init(void)
{
    /* Sim simplification: Init returns the tool to home (position 0).
     * The HAL contract says Init does not move the tool; a bench or
     * flight backend must keep the physical position. Leaving the
     * simulated tip parked at the bottom of its hole makes the
     * regolith model's loading/unloading branch flip every sample,
     * which reads as a continuous force-rate excursion. */
    memset(&g_State, 0, sizeof(g_State));
    memset(&g_Params, 0, sizeof(g_Params));

    /* Default parameters: LRMH-class linear actuator */
    g_Params.MaxStroke_m           = 0.35f;
    g_Params.MaxVelocity_mps      = 0.01f;   /* 10 mm/s max         */
    g_Params.MaxForce_N           = 650.0f;
    g_Params.MaxCurrent_A         = 5.0f;
    g_Params.Backlash_m           = 0.0002f;  /* 0.2 mm              */
    g_Params.ResponseLag_s        = 0.05f;    /* 50 ms time constant */
    g_Params.StallCurrentThresh_A = 4.0f;
    g_Params.HardStopStiffness    = 50000.0f; /* N/m at limits       */
    g_Params.HomingPosition_m     = 0.0f;

    g_State.Position_m     = 0.0f;
    g_State.Temperature_C  = AMBIENT_TEMP_C;
    g_State.ControlMode    = 3; /* HOLD */
    g_State.CommandComplete = 1;

    g_Initialized = 1;
    return 0;
}

int32 Actuator_SetParams(const ActuatorParams_t *Params)
{
    if (Params == NULL) return -1;
    memcpy(&g_Params, Params, sizeof(ActuatorParams_t));
    return 0;
}

int32 Actuator_GetState(ActuatorState_t *State)
{
    if (!g_Initialized || State == NULL) return -1;
    memcpy(State, &g_State, sizeof(ActuatorState_t));
    return 0;
}

/* ============================================================
 * Commands
 * ============================================================ */
int32 Actuator_CmdPosition(float TargetPos_m, float RateLimit_mps, float Timeout_s)
{
    if (!g_Initialized) return -1;
    if (g_State.IsEStoped) return -5;

    /* Clamp target */
    if (TargetPos_m < 0.0f) TargetPos_m = 0.0f;
    if (TargetPos_m > g_Params.MaxStroke_m) TargetPos_m = g_Params.MaxStroke_m;
    if (RateLimit_mps > g_Params.MaxVelocity_mps) RateLimit_mps = g_Params.MaxVelocity_mps;

    g_State.CmdTargetPosition_m = TargetPos_m;
    g_State.CmdRateLimit_mps    = RateLimit_mps;
    g_State.CmdTimeout_s        = Timeout_s;
    g_State.CmdElapsed_s        = 0.0f;
    g_State.ControlMode         = 0; /* POSITION */
    g_State.CommandComplete     = 0;
    g_State.IsStalled           = 0;

    return 0;
}

int32 Actuator_CmdForce(float TargetForce_N, float RateLimit_mps, float Timeout_s)
{
    if (!g_Initialized) return -1;
    if (g_State.IsEStoped) return -5;

    if (TargetForce_N > g_Params.MaxForce_N) TargetForce_N = g_Params.MaxForce_N;
    if (RateLimit_mps > g_Params.MaxVelocity_mps) RateLimit_mps = g_Params.MaxVelocity_mps;

    g_State.CmdTargetForce_N    = TargetForce_N;
    g_State.CmdRateLimit_mps    = RateLimit_mps;
    g_State.CmdTimeout_s        = Timeout_s;
    g_State.CmdElapsed_s        = 0.0f;
    g_State.ControlMode         = 1; /* FORCE */
    g_State.CommandComplete     = 0;
    g_State.IsStalled           = 0;

    return 0;
}

int32 Actuator_CmdRetract(float RateLimit_mps, float Timeout_s)
{
    if (!g_Initialized) return -1;
    if (g_State.IsEStoped) return -5;

    g_State.CmdTargetPosition_m = g_Params.HomingPosition_m;
    g_State.CmdRateLimit_mps    = RateLimit_mps > 0 ? RateLimit_mps : g_Params.MaxVelocity_mps;
    g_State.CmdTimeout_s        = Timeout_s;
    g_State.CmdElapsed_s        = 0.0f;
    g_State.ControlMode         = 2; /* RETRACT */
    g_State.CommandComplete     = 0;

    return 0;
}

int32 Actuator_SetForceTarget(float TargetForce_N)
{
    if (!g_Initialized) return -1;
    if (g_State.ControlMode != 1) return -1;
    if (TargetForce_N > g_Params.MaxForce_N) TargetForce_N = g_Params.MaxForce_N;
    g_State.CmdTargetForce_N = TargetForce_N;
    g_State.CommandComplete  = 0;
    return 0;
}

int32 Actuator_CmdHold(void)
{
    if (!g_Initialized) return -1;
    g_State.ControlMode     = 3; /* HOLD */
    g_State.CommandComplete = 1;
    return 0;
}

int32 Actuator_EStop(void)
{
    if (!g_Initialized) return -1;
    g_State.ControlMode     = 4; /* ESTOP */
    g_State.IsEStoped       = 1;
    g_State.Velocity_mps    = 0.0f;
    g_State.CommandComplete = 1;
    return 0;
}

int32 Actuator_ClearEStop(void)
{
    if (!g_Initialized) return -1;
    g_State.IsEStoped   = 0;
    g_State.ControlMode = 3; /* HOLD after clearing */
    return 0;
}

/* ============================================================
 * Physics Step
 * ============================================================ */
int32 Actuator_Step(double dt, float ExternalForce_N)
{
    if (!g_Initialized) return -1;
    if (dt <= 0.0) return 0;

    float dtf = (float)dt;

    /* E-stop: everything stopped */
    if (g_State.IsEStoped)
    {
        g_State.Velocity_mps = 0.0f;
        g_State.MotorCurrent_A = 0.0f;
        return 0;
    }

    /* Injected stall */
    if (g_State.InjectStall)
    {
        g_State.IsStalled = 1;
        g_State.Velocity_mps = 0.0f;
        g_State.MotorCurrent_A = g_Params.StallCurrentThresh_A + 0.5f;
        g_State.CommandComplete = 0;
        goto thermal_update;
    }

    /* Compute desired velocity based on control mode */
    float desiredVel = 0.0f;

    switch (g_State.ControlMode)
    {
        case 0: /* POSITION */
        {
            float posError = g_State.CmdTargetPosition_m - g_State.Position_m;
            float sign = (posError >= 0.0f) ? 1.0f : -1.0f;
            float absError = fabsf(posError);

            if (absError < 0.0005f) /* 0.5 mm deadband */
            {
                desiredVel = 0.0f;
                g_State.CommandComplete = 1;
            }
            else
            {
                /* Proportional with rate limit */
                float propVel = posError * 2.0f; /* P-gain */
                if (fabsf(propVel) > g_State.CmdRateLimit_mps)
                    propVel = sign * g_State.CmdRateLimit_mps;
                desiredVel = propVel;
            }
            break;
        }

        case 1: /* FORCE */
        {
            float forceError = g_State.CmdTargetForce_N - ExternalForce_N;
            if (forceError > 1.0f)
            {
                /* Need to push harder: advance */
                desiredVel = g_State.CmdRateLimit_mps * 0.5f;
                if (forceError < 10.0f) /* Slow down near target */
                    desiredVel *= forceError / 10.0f;
            }
            else if (forceError < -1.0f)
            {
                /* Overshoot: back off slightly */
                desiredVel = -g_State.CmdRateLimit_mps * 0.2f;
            }
            else
            {
                desiredVel = 0.0f;
                g_State.CommandComplete = 1;
            }
            break;
        }

        case 2: /* RETRACT */
        {
            float posError = g_State.CmdTargetPosition_m - g_State.Position_m;
            if (fabsf(posError) < 0.001f)
            {
                desiredVel = 0.0f;
                g_State.CommandComplete = 1;
            }
            else
            {
                desiredVel = (posError >= 0.0f) ? g_State.CmdRateLimit_mps
                                                 : -g_State.CmdRateLimit_mps;
            }
            break;
        }

        case 3: /* HOLD */
            desiredVel = 0.0f;
            break;

        case 4: /* ESTOP */
            desiredVel = 0.0f;
            break;

        default:
            desiredVel = 0.0f;
            break;
    }

    /* First-order lag on velocity */
    float tau = g_Params.ResponseLag_s;
    if (tau < 0.001f) tau = 0.001f;
    float alpha = dtf / (tau + dtf);
    g_State.Velocity_mps += alpha * (desiredVel - g_State.Velocity_mps);

    /* Integrate position */
    g_State.Position_m += g_State.Velocity_mps * dtf;

    /* Hard stop enforcement */
    g_State.AtHardStopMin = 0;
    g_State.AtHardStopMax = 0;

    if (g_State.Position_m <= 0.0f)
    {
        g_State.Position_m = 0.0f;
        if (g_State.Velocity_mps < 0.0f) g_State.Velocity_mps = 0.0f;
        g_State.AtHardStopMin = 1;
    }
    if (g_State.Position_m >= g_Params.MaxStroke_m)
    {
        g_State.Position_m = g_Params.MaxStroke_m;
        if (g_State.Velocity_mps > 0.0f) g_State.Velocity_mps = 0.0f;
        g_State.AtHardStopMax = 1;
    }

    /* Motor current model */
    float forceCurrent = fabsf(ExternalForce_N) / FORCE_CONSTANT;
    float velCurrent   = fabsf(g_State.Velocity_mps) * BACK_EMF_CONST;
    g_State.MotorCurrent_A = forceCurrent + velCurrent + 0.1f; /* 0.1A quiescent */

    /* Overcurrent injection */
    if (g_State.InjectOvercurrent)
    {
        g_State.MotorCurrent_A = g_Params.MaxCurrent_A + 1.0f;
    }

    /* Clamp current */
    if (g_State.MotorCurrent_A > g_Params.MaxCurrent_A)
    {
        g_State.MotorCurrent_A = g_Params.MaxCurrent_A;
    }

    /* Stall detection */
    if (g_State.MotorCurrent_A > g_Params.StallCurrentThresh_A &&
        fabsf(g_State.Velocity_mps) < 0.0001f &&
        g_State.ControlMode != 3 && g_State.ControlMode != 4)
    {
        g_State.IsStalled = 1;
    }

    /* Applied force tracking */
    g_State.AppliedForce_N = ExternalForce_N;

    /* Timeout check */
    if (g_State.ControlMode < 3) /* Active command modes */
    {
        g_State.CmdElapsed_s += dtf;
        if (g_State.CmdTimeout_s > 0.0f && g_State.CmdElapsed_s >= g_State.CmdTimeout_s)
        {
            g_State.CommandComplete = 1;
            /* Don't change mode — let the app decide */
        }
    }

thermal_update:
    /* Simple thermal model */
    {
        float powerDissipated = g_State.MotorCurrent_A * g_State.MotorCurrent_A * 0.5f; /* I^2 * R */
        float heatIn  = powerDissipated * dtf;
        float heatOut = (g_State.Temperature_C - AMBIENT_TEMP_C) / THERMAL_RESISTANCE * dtf;
        float dTemp   = (heatIn - heatOut) / THERMAL_CAPACITANCE;
        g_State.Temperature_C += dTemp;
    }

    return 0;
}

/* ============================================================
 * Fault Injection
 * ============================================================ */
int32 Actuator_InjectFault(uint8 FaultType, uint8 Active)
{
    if (!g_Initialized) return -1;

    switch (FaultType)
    {
        case 0: g_State.InjectStall       = Active; break;
        case 1: g_State.InjectOvercurrent = Active; break;
        case 2: g_State.InjectEncoderLoss = Active; break;
        default: return -1;
    }
    return 0;
}
