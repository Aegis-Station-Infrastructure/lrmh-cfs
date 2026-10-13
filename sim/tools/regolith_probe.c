/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file regolith_probe.c
 * @brief Push a cone through a drawn site and print what it feels.
 *
 *   regolith_probe <preset> <seed> [area_cm2=1.29] [max_depth_cm=35] [step_cm=0.5] [gravity]
 *   regolith_probe --list
 *
 * One line per step: depth_cm  q_kPa  force_N  layer  clast  hole.
 * A first line names the site that was drawn. The default cone is the
 * Apollo self-recording penetrometer's 0.2 in^2 (1.29 cm^2), so a lunar
 * preset can be laid against the Apollo 16 records directly.
 */

#include "regolith_model.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int preset_from_arg(const char *a)
{
    for (int p = 0; p < REGOLITH_PRESET_COUNT; p++)
    {
        if (strcmp(a, Regolith_PresetName((RegolithPreset_t)p)) == 0) return p;
    }
    char *end = NULL;
    long v = strtol(a, &end, 10);
    return (end && *end == '\0' && v >= 0 && v < REGOLITH_PRESET_COUNT) ? (int)v : -1;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "--list") == 0)
    {
        for (int p = 0; p < REGOLITH_PRESET_COUNT; p++) printf("%d %s\n", p, Regolith_PresetName((RegolithPreset_t)p));
        return 0;
    }
    if (argc < 3)
    {
        fprintf(stderr, "usage: regolith_probe <preset> <seed> [area_cm2] [max_depth_cm] [step_cm] [gravity]\n");
        return 2;
    }
    int preset = preset_from_arg(argv[1]);
    if (preset < 0) { fprintf(stderr, "no such preset: %s\n", argv[1]); return 2; }
    uint32 seed     = (uint32)strtoul(argv[2], NULL, 10);
    float  area_m2  = (argc > 3 ? (float)atof(argv[3]) : 1.29f) * 1e-4f;
    float  maxDepth = (argc > 4 ? (float)atof(argv[4]) : 35.0f) * 0.01f;
    float  step     = (argc > 5 ? (float)atof(argv[5]) : 0.5f) * 0.01f;

    Regolith_Init();
    RegolithProfile_t p;
    Regolith_DrawSite((RegolithPreset_t)preset, seed, &p);
    if (argc > 6) p.Gravity_mps2 = (float)atof(argv[6]);
    Regolith_LoadProfile(&p);

    printf("# %s seed %u g %.2f rho %.0f(%.1f/%.1f) layers %u:", Regolith_PresetName((RegolithPreset_t)preset),
           seed, (double)p.Gravity_mps2, (double)p.RhoInf_kgm3, (double)p.RhoA_cm, (double)p.RhoB_cm, p.NumLayers);
    for (int i = 0; i < p.NumLayers; i++)
        printf(" [%.1fcm c=%.0fPa phi=%.0f x%.2f]", (double)p.Layers[i].ZTop_m * 100, (double)p.Layers[i].Cohesion_Pa,
               (double)p.Layers[i].FrictionAngle_deg, (double)p.Layers[i].DensityScale);
    printf(" crust %.1fcm x%.2f var %.2f/%.1fcm clasts %u:", (double)p.CrustThickness_m * 100, (double)p.CrustFactor,
           (double)p.VariabilitySigma, (double)p.VariabilityLength_m * 100, p.NumClasts);
    for (int i = 0; i < p.NumClasts; i++)
        printf(" [%.1fcm %.1fcm %.1fMPa%s]", (double)p.Clasts[i].Z_m * 100, (double)p.Clasts[i].Size_m * 100,
               (double)p.Clasts[i].Strength_Pa / 1e6, p.Clasts[i].Immovable ? " fixed" : "");
    printf("\n# depth_cm q_kPa force_N layer clast hole\n");

    for (float z = 0.0f; z <= maxDepth + 1e-6f; z += step)
    {
        RegolithResponse_t r;
        Regolith_GetPenetrationResponse(z, area_m2, 0.005f, &r);
        printf("%.2f %.1f %.2f %u %u %u\n", (double)z * 100, (double)r.PenetrationResistance_kPa,
               (double)r.ReactionForce_N, r.LayerIndex, r.InClast, r.InHole);
    }
    return 0;
}
