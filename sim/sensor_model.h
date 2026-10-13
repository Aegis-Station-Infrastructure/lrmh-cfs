/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file sensor_model.h
 * @brief Sensor noise and imperfection model for LRMH simulation
 *
 * Takes ideal "truth" values from actuator and regolith models and
 * produces realistic sensor readings with:
 *   - Gaussian noise
 *   - Encoder quantization
 *   - Bias / drift
 *   - Dropout (intermittent loss)
 *   - Saturation / clipping
 *   - Timestamp jitter
 */

#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

#include "common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Sensor Configuration
 * ============================================================ */
typedef struct
{
    /* Force sensor (load cell) */
    float   ForceNoiseSigma_N;      /* Gaussian noise std dev        */
    float   ForceBias_N;            /* Constant offset               */
    float   ForceDrift_Nps;         /* Drift rate (N per second)     */
    float   ForceSaturation_N;      /* Clipping threshold            */
    float   ForceDropoutProb;       /* Probability of dropout [0,1]  */

    /* Depth / encoder */
    float   DepthQuantization_m;    /* Encoder resolution            */
    float   DepthNoiseSigma_m;      /* Gaussian noise                */
    float   DepthBias_m;            /* Constant offset               */
    float   DepthDropoutProb;       /* Probability of dropout        */

    /* Motor current */
    float   CurrentNoiseSigma_A;
    float   CurrentBias_A;

    /* Temperature */
    float   TempNoiseSigma_C;
    float   TempBias_C;

    /* Timestamp */
    float   TimestampJitter_s;      /* Max jitter on timestamps      */
} SensorConfig_t;

/* ============================================================
 * Sensor Readings (post-noise, what the app actually sees)
 * ============================================================ */
typedef struct
{
    double  Timestamp_s;
    float   Force_N;
    float   Depth_m;
    float   StrokePosition_m;
    float   MotorCurrent_A;
    float   Temperature_C;
    uint8   ForceValid;             /* 0 = dropout this cycle */
    uint8   DepthValid;
    uint8   CurrentValid;
    uint8   TempValid;
} SensorReadings_t;

/* ============================================================
 * API
 * ============================================================ */

int32 Sensor_Init(void);
int32 Sensor_SetConfig(const SensorConfig_t *Config);
int32 Sensor_GetConfig(SensorConfig_t *Config);

/**
 * @brief Apply sensor model to ideal truth values
 * @param[in]  TrueTime_s       Actual mission time
 * @param[in]  TrueForce_N      Ideal force from regolith model
 * @param[in]  TrueDepth_m      Ideal penetration depth
 * @param[in]  TrueStroke_m     Ideal actuator position
 * @param[in]  TrueCurrent_A    Ideal motor current
 * @param[in]  TrueTemp_C       Ideal temperature
 * @param[out] Readings          Noisy sensor readings
 */
int32 Sensor_ApplyModel(double TrueTime_s,
                         float TrueForce_N,
                         float TrueDepth_m,
                         float TrueStroke_m,
                         float TrueCurrent_A,
                         float TrueTemp_C,
                         SensorReadings_t *Readings);

/**
 * @brief Reset accumulated drift
 */
int32 Sensor_ResetDrift(void);

/**
 * @brief Set force sensor zero offset (tare)
 */
int32 Sensor_ZeroForce(float CurrentTrueForce_N);

/**
 * @brief Set depth encoder zero
 */
int32 Sensor_ZeroDepth(float CurrentTrueDepth_m);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MODEL_H */
