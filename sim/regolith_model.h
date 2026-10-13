/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file regolith_model.h
 * @brief Simulated regolith for the LRMH HAL sim: drawn sites, lunar and Earth
 *
 * Sim-only (ADR-0002: the regolith is opaque to the flight software; the
 * mission apps see force, depth, current and temperature and nothing here).
 *
 * What the model does:
 *   - A site is DRAWN from a preset, not typed in: a density profile, one to
 *     a few layers with their own cohesion and friction angle, a surface
 *     crust, a correlated variability field along depth, and buried clasts.
 *     Two sites of the same preset differ the way two Apollo 16 penetrations
 *     did (a factor of several in resistance at the same depth).
 *   - Penetration resistance from bearing-capacity theory (the method the
 *     Apollo 16 soil-mechanics report used in reverse): cohesion and
 *     friction angle of the layer, the overburden stress from the density
 *     profile under the site's gravity, the cone's size, and shape and
 *     depth factors. One calibration factor per preset puts the lunar
 *     presets inside the Apollo 16 self-recording penetrometer envelope.
 *   - The density profile is Carrier's law, rho(z) = RhoInf (z+A)/(z+B) with
 *     z in cm (1.92 (z+12.2)/(z+18) g/cm3 for the Moon): loose at the top,
 *     dense below, which is where the Moon's steep near-surface gradient
 *     comes from. A = B gives a uniform profile (an Earth bin).
 *   - Clasts: a buried fragment is a bump in resistance over its size. A
 *     small one is shoved aside; an immovable one stops the tool at the
 *     force clamp. The bench bin's floor is an immovable clast at its depth.
 *   - The hole: a second penetration at the same micro-site (POST_PEN) feels
 *     only shaft friction down to the depth already reached, and compacted
 *     soil below it from the load cycles.
 *   - Earth presets at 9.81 m/s2: dry sand, sandy loam, firm clay, and a
 *     bench bin of JSC-1A-like simulant. The bench backend will be pushed
 *     into one of these; the sim should already know what that feels like.
 *   - Load-sinkage (Bekker, p = (kc/b + kphi) z^n) with the classical
 *     parameter tables, kept for the plate-load API.
 *
 * Nothing here is fitted to the bench hardware. Every number is a model
 * choice, documented in docs/30-subsystems/regolith-model.md.
 */

#ifndef REGOLITH_MODEL_H
#define REGOLITH_MODEL_H

#include "common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define REGOLITH_MAX_LAYERS     6
#define REGOLITH_MAX_CLASTS     8
#define REGOLITH_MAX_SITES      32

/* ============================================================
 * Presets: the families a site is drawn from
 * ============================================================ */
typedef enum
{
    REGOLITH_PRESET_LUNAR_HIGHLAND = 0,   /* Apollo 16 Descartes-like: dense, cohesive, rock fragments */
    REGOLITH_PRESET_LUNAR_MARE     = 1,   /* Apollo 12/15 mare-like: a little looser, fewer fragments */
    REGOLITH_PRESET_LUNAR_LOOSE    = 2,   /* fresh ejecta, a crater rim: loose and weak */
    REGOLITH_PRESET_EARTH_DRY_SAND = 3,   /* 1 g, cohesionless, uniform density */
    REGOLITH_PRESET_EARTH_SANDY_LOAM = 4, /* 1 g, some cohesion, a crust */
    REGOLITH_PRESET_EARTH_FIRM_CLAY = 5,  /* 1 g, cohesion carries the load, no friction */
    REGOLITH_PRESET_BENCH_SIMULANT = 6,   /* 1 g, a bin of JSC-1A/LHS-1-like simulant with a floor */
    REGOLITH_PRESET_COUNT
} RegolithPreset_t;

/* ============================================================
 * A layer: from its top down to the next layer's top
 * ============================================================ */
typedef struct
{
    float   ZTop_m;                 /* Top of layer, depth from surface     */
    float   Cohesion_Pa;            /* Mohr-Coulomb c at full density       */
    float   FrictionAngle_deg;      /* Mohr-Coulomb phi at full density     */
    float   DensityScale;           /* Multiplies the density profile       */
} RegolithLayer_t;

/* ============================================================
 * A clast: a buried fragment, or the bench bin's floor
 * ============================================================ */
typedef struct
{
    float   Z_m;                    /* Depth of its centre                  */
    float   Size_m;                 /* Its extent along the hole            */
    float   Strength_Pa;            /* Resistance it adds at its centre     */
    uint8   Immovable;              /* 1 = stops the tool; 0 = shoved aside */
    uint8   Spare[3];
} RegolithClast_t;

/* ============================================================
 * Site profile
 * ============================================================ */
typedef struct
{
    uint16          SiteId;
    uint8           Preset;                 /* RegolithPreset_t             */
    uint8           NumLayers;
    float           Gravity_mps2;

    /* Density profile rho(z) = RhoInf (zcm + A)/(zcm + B), kg/m3, z in cm */
    float           RhoInf_kgm3;
    float           RhoA_cm;
    float           RhoB_cm;

    float           ConeFactor;             /* Calibration of cone resistance */
    float           SurfaceSoftening_deg;   /* phi lost at the loose surface  */

    RegolithLayer_t Layers[REGOLITH_MAX_LAYERS];

    float           CrustThickness_m;       /* 0 = no crust                   */
    float           CrustFactor;            /* Resistance multiplier in it    */

    float           VariabilitySigma;       /* Log-normal field along depth   */
    float           VariabilityLength_m;    /* Its correlation length         */

    uint8           NumClasts;
    uint8           Spare[3];
    RegolithClast_t Clasts[REGOLITH_MAX_CLASTS];

    float           ShaftFriction;          /* Fraction of q felt in the hole */
    float           Compressibility;        /* Density growth per load cycle  */

    /* Bekker load-sinkage, p = (kc/b + kphi) z^n, SI */
    float           Bekker_kc;              /* N/m^(n+1)                      */
    float           Bekker_kphi;            /* N/m^(n+2)                      */
    float           Bekker_n;

    uint32          RandomSeed;
} RegolithProfile_t;

/* ============================================================
 * Disturbance state: the hole and the compaction
 * ============================================================ */
typedef struct
{
    uint16  SiteId;
    uint8   LoadCyclesApplied;
    uint8   Spare;
    float   CumulativeCompaction_m;     /* Total sinkage from all cycles     */
    float   LastIncrementalSinkage_m;
    float   CurrentDensityMultiplier;   /* 1.0 undisturbed, >1.0 compacted   */
    float   MaxDepth_m;                 /* The hole: deepest the tip has been */
} RegolithCompactionState_t;

/* ============================================================
 * Model output: what the actuator feels
 * ============================================================ */
typedef struct
{
    float   ReactionForce_N;            /* Force resisting penetration       */
    float   PenetrationResistance_kPa;  /* Cone resistance at this depth     */
    float   Sinkage_m;                  /* Sinkage under current load        */
    float   StiffnessProxy;             /* Local dF/dz                       */
    uint8   InLayerTransition;          /* 1 = just below a layer top        */
    uint8   InClast;                    /* 1 = the tip is on a clast         */
    uint8   InHole;                     /* 1 = inside the earlier hole       */
    uint8   LayerIndex;
} RegolithResponse_t;

/* ============================================================
 * API
 * ============================================================ */

/** Initialize the model and draw the default site (lunar highland, site 0). */
int32 Regolith_Init(void);

/** Load a profile as given. */
int32 Regolith_LoadProfile(const RegolithProfile_t *Profile);

/** Draw a site of a preset from a seed, without loading it. */
int32 Regolith_DrawSite(RegolithPreset_t Preset, uint32 Seed, RegolithProfile_t *Out);

/** Draw a site of a preset from a seed and load it. */
int32 Regolith_SetSite(RegolithPreset_t Preset, uint32 Seed);

/** The default: a lunar highland site drawn from the site id. */
int32 Regolith_SetDefaultProfile(uint16 SiteId);

/** The preset's name, for logs. */
const char *Regolith_PresetName(RegolithPreset_t Preset);

/**
 * Soil response to the tip at a depth.
 * @param[in]  Depth_m       Current penetration depth of the tip
 * @param[in]  ToolArea_m2   Projected area of the tip
 * @param[in]  Rate_mps      Penetration rate
 * @param[out] Response      The reaction
 *
 * Deepens the hole when Depth_m exceeds the deepest point so far.
 */
int32 Regolith_GetPenetrationResponse(float Depth_m, float ToolArea_m2,
                                       float Rate_mps, RegolithResponse_t *Response);

/** Plate-load response at the surface (Bekker). */
int32 Regolith_GetLoadResponse(float AppliedForce_N, float PlateArea_m2,
                                RegolithResponse_t *Response);

/** Record a completed load cycle (compaction). */
int32 Regolith_RecordLoadCycle(float PeakForce_N, float PeakSinkage_m);

int32 Regolith_GetCompactionState(RegolithCompactionState_t *State);

/** Forget the hole and the compaction: an undisturbed micro-site. */
int32 Regolith_ResetCompaction(void);

const RegolithProfile_t* Regolith_GetCurrentProfile(void);

#ifdef __cplusplus
}
#endif

#endif /* REGOLITH_MODEL_H */
