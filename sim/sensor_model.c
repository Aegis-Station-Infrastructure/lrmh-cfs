/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file sensor_model.c
 * @brief Sensor imperfection model implementation
 */

#include "sensor_model.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

static SensorConfig_t g_Config;
/* Last valid readings, held through a dropout (see Sim Model Spec). */
static float g_LastForce_N = 0.0f;
static float g_LastDepth_m = 0.0f;
static uint8          g_Initialized = 0;
static float          g_AccumulatedDrift_N = 0.0f;
static float          g_ForceZeroOffset_N  = 0.0f;
static float          g_DepthZeroOffset_m  = 0.0f;
static double         g_LastTime_s         = 0.0;
static uint8  g_HaveLastTime = 0;
static uint32         g_RngState           = 54321;

/* ============================================================
 * PRNG Utilities
 * ============================================================ */
static uint32 NextRng(void)
{
    g_RngState = g_RngState * 1103515245 + 12345;
    return (g_RngState >> 16) & 0x7FFF;
}

static float RandUniform(void)
{
    return (float)NextRng() / 32767.0f;
}

/* Box-Muller for Gaussian noise */
static float RandGaussian(float sigma)
{
    if (sigma <= 0.0f) return 0.0f;

    float u1 = RandUniform();
    float u2 = RandUniform();
    if (u1 < 1e-7f) u1 = 1e-7f;

    float z = sqrtf(-2.0f * logf(u1)) * cosf(6.28318f * u2);
    return z * sigma;
}

/* ============================================================
 * Initialization
 * ============================================================ */
int32 Sensor_Init(void)
{
    g_HaveLastTime = 0;
    memset(&g_Config, 0, sizeof(g_Config));

    /* Realistic defaults for an LRMH-class instrument */
    g_Config.ForceNoiseSigma_N   = 1.5f;       /* ±1.5 N noise        */
    g_Config.ForceBias_N         = 0.3f;        /* Small constant bias */
    g_Config.ForceDrift_Nps      = 0.001f;      /* Very slow drift     */
    g_Config.ForceSaturation_N   = 650.0f;      /* Load cell max       */
    g_Config.ForceDropoutProb    = 0.002f;       /* 0.2% dropout rate   */

    g_Config.DepthQuantization_m = 0.00005f;    /* 50 µm encoder res   */
    g_Config.DepthNoiseSigma_m   = 0.0001f;     /* 100 µm noise        */
    g_Config.DepthBias_m         = 0.0f;
    g_Config.DepthDropoutProb    = 0.001f;

    g_Config.CurrentNoiseSigma_A = 0.05f;
    g_Config.CurrentBias_A       = 0.02f;

    g_Config.TempNoiseSigma_C    = 0.2f;
    g_Config.TempBias_C          = 0.0f;

    g_Config.TimestampJitter_s   = 0.001f;      /* 1 ms jitter         */

    g_AccumulatedDrift_N = 0.0f;
    g_ForceZeroOffset_N  = 0.0f;
    g_DepthZeroOffset_m  = 0.0f;
    g_LastTime_s         = 0.0;

    g_Initialized = 1;
    return 0;
}

int32 Sensor_SetConfig(const SensorConfig_t *Config)
{
    if (Config == NULL) return -1;
    memcpy(&g_Config, Config, sizeof(SensorConfig_t));
    return 0;
}

int32 Sensor_GetConfig(SensorConfig_t *Config)
{
    if (Config == NULL) return -1;
    memcpy(Config, &g_Config, sizeof(SensorConfig_t));
    return 0;
}

/* ============================================================
 * Apply Sensor Model
 * ============================================================ */
int32 Sensor_ApplyModel(double TrueTime_s,
                         float TrueForce_N,
                         float TrueDepth_m,
                         float TrueStroke_m,
                         float TrueCurrent_A,
                         float TrueTemp_C,
                         SensorReadings_t *Readings)
{
    if (!g_Initialized || Readings == NULL) return -1;

    memset(Readings, 0, sizeof(SensorReadings_t));

    /* Drift accumulates from the first sample, not from time zero: the
     * sim clock is the host's monotonic time, which can be days. */
    float dt = g_HaveLastTime ? (float)(TrueTime_s - g_LastTime_s) : 0.0f;
    if (dt < 0.0f) dt = 0.0f;
    g_LastTime_s   = TrueTime_s;
    g_HaveLastTime = 1;

    /* ---- Timestamp with jitter ---- */
    Readings->Timestamp_s = TrueTime_s + (double)RandGaussian(g_Config.TimestampJitter_s);

    /* ---- Force sensor ---- */
    Readings->ForceValid = 1;
    if (g_Config.ForceDropoutProb > 0.0f && RandUniform() < g_Config.ForceDropoutProb)
    {
        /* Dropout: the channel is flagged invalid and holds its last value. */
        Readings->ForceValid = 0;
        Readings->Force_N    = g_LastForce_N;
    }
    else
    {
        /* Accumulate drift */
        g_AccumulatedDrift_N += g_Config.ForceDrift_Nps * dt;

        float measured = TrueForce_N
                       - g_ForceZeroOffset_N
                       + g_Config.ForceBias_N
                       + g_AccumulatedDrift_N
                       + RandGaussian(g_Config.ForceNoiseSigma_N);

        /* Saturation clipping */
        if (measured > g_Config.ForceSaturation_N)
            measured = g_Config.ForceSaturation_N;
        if (measured < -g_Config.ForceSaturation_N)
            measured = -g_Config.ForceSaturation_N;

        Readings->Force_N = measured;
        g_LastForce_N = measured;
    }

    /* ---- Depth / encoder ---- */
    Readings->DepthValid = 1;
    if (g_Config.DepthDropoutProb > 0.0f && RandUniform() < g_Config.DepthDropoutProb)
    {
        Readings->DepthValid = 0;
        Readings->Depth_m    = g_LastDepth_m;
    }
    else
    {
        float measured = TrueDepth_m
                       - g_DepthZeroOffset_m
                       + g_Config.DepthBias_m
                       + RandGaussian(g_Config.DepthNoiseSigma_m);

        /* Quantize to encoder resolution */
        if (g_Config.DepthQuantization_m > 0.0f)
        {
            measured = roundf(measured / g_Config.DepthQuantization_m) * g_Config.DepthQuantization_m;
        }

        Readings->Depth_m = measured;
        g_LastDepth_m = measured;
    }

    /* ---- Stroke position (same encoder model as depth) ---- */
    {
        float measured = TrueStroke_m
                       + RandGaussian(g_Config.DepthNoiseSigma_m);
        if (g_Config.DepthQuantization_m > 0.0f)
        {
            measured = roundf(measured / g_Config.DepthQuantization_m) * g_Config.DepthQuantization_m;
        }
        Readings->StrokePosition_m = measured;
    }

    /* ---- Motor current ---- */
    Readings->CurrentValid = 1;
    Readings->MotorCurrent_A = TrueCurrent_A
                              + g_Config.CurrentBias_A
                              + RandGaussian(g_Config.CurrentNoiseSigma_A);

    /* ---- Temperature ---- */
    Readings->TempValid = 1;
    Readings->Temperature_C = TrueTemp_C
                            + g_Config.TempBias_C
                            + RandGaussian(g_Config.TempNoiseSigma_C);

    return 0;
}

/* ============================================================
 * Calibration / Zero
 * ============================================================ */
int32 Sensor_ResetDrift(void)
{
    g_AccumulatedDrift_N = 0.0f;
    return 0;
}

int32 Sensor_ZeroForce(float CurrentTrueForce_N)
{
    g_ForceZeroOffset_N = CurrentTrueForce_N + g_Config.ForceBias_N + g_AccumulatedDrift_N;
    return 0;
}

int32 Sensor_ZeroDepth(float CurrentTrueDepth_m)
{
    g_DepthZeroOffset_m = CurrentTrueDepth_m + g_Config.DepthBias_m;
    return 0;
}
