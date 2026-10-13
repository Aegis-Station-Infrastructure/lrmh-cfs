/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file regolith_model.c
 * @brief Simulated regolith: drawn sites, bearing-capacity cone resistance
 *
 * See regolith_model.h for what the model is, and
 * docs/30-subsystems/regolith-model.md for where each number comes from.
 */

#include "regolith_model.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define PI_F 3.14159265f
#define REBOUND_M 0.008f     /* elastic rebound of the soil under the tip as it lifts */
#define CLAST_RAMP_M 0.005f  /* an immovable clast is met over the contact's elastic compression, not in one step */
#define CONE_HEIGHT_M 0.010f /* the tip engages over its own height: a 60-degree cone of 11 mm diameter */

/* ============================================================
 * Module state
 * ============================================================ */
static RegolithProfile_t         g_Profile;
static RegolithCompactionState_t g_Compaction;
static uint8                     g_Initialized = 0;

/* ============================================================
 * Seeded draws (an LCG; repeatable per site)
 * ============================================================ */
static uint32 g_RngState = 12345;

static uint32 NextU32(uint32 *s)
{
    *s = *s * 1103515245u + 12345u;
    return (*s >> 8) & 0xFFFFFFu;
}

static float RandUniform(uint32 *s, float lo, float hi)
{
    return lo + (float)NextU32(s) / 16777215.0f * (hi - lo);
}

/* ============================================================
 * Value noise along depth: a Gaussian-ish value at each lattice
 * node from a hash of (seed, node), cosine-interpolated between
 * nodes. A function of depth alone, so the same hole feels the
 * same soil on the way down and on the way back.
 * ============================================================ */
static uint32 Hash(uint32 seed, uint32 i)
{
    uint32 h = seed ^ (i * 0x9E3779B1u);
    h ^= h >> 15; h *= 0x85EBCA6Bu;
    h ^= h >> 13; h *= 0xC2B2AE35u;
    h ^= h >> 16;
    return h;
}

static float NodeGaussian(uint32 seed, uint32 i)
{
    /* Sum of four uniforms on [-1, 1]: variance 4/3, so scale to unit */
    float s = 0.0f;
    for (uint32 k = 0; k < 4; k++)
    {
        s += (float)(Hash(seed, i * 4u + k) & 0xFFFFu) / 32767.5f - 1.0f;
    }
    return s * 0.8660254f;
}

static float ValueNoise(uint32 seed, float z_m, float length_m)
{
    if (length_m < 1e-4f) length_m = 1e-4f;
    float x = z_m / length_m + 1000.0f;             /* keep the node index positive */
    uint32 i = (uint32)floorf(x);
    float  f = x - (float)i;
    float  w = 0.5f - 0.5f * cosf(PI_F * f);
    return NodeGaussian(seed, i) * (1.0f - w) + NodeGaussian(seed, i + 1u) * w;
}

/* ============================================================
 * Density profile and overburden
 *
 * rho(z) = RhoInf (zcm + A)/(zcm + B)
 * The integral of (z+A)/(z+B) dz is z + (A-B) ln(z+B).
 * ============================================================ */
static float DensityRatio(float z_m)
{
    float zc = z_m * 100.0f;
    float b  = g_Profile.RhoB_cm;
    if (zc + b < 1e-3f) return 1.0f;
    return (zc + g_Profile.RhoA_cm) / (zc + b);
}

static float DensityIntegral_cm(float zc)
{
    float a = g_Profile.RhoA_cm, b = g_Profile.RhoB_cm;
    if (b < 1e-3f) return zc;
    return zc + (a - b) * logf((zc + b) / b);
}

static uint8 LayerAt(float z_m)
{
    uint8 idx = 0;
    for (uint8 i = 0; i < g_Profile.NumLayers; i++)
    {
        if (z_m >= g_Profile.Layers[i].ZTop_m) idx = i;
    }
    return idx;
}

/* Overburden stress at depth z, Pa, integrated layer by layer. */
static float Overburden_Pa(float z_m)
{
    float sigma = 0.0f;
    for (uint8 i = 0; i < g_Profile.NumLayers; i++)
    {
        float top = g_Profile.Layers[i].ZTop_m;
        float bot = (i + 1 < g_Profile.NumLayers) ? g_Profile.Layers[i + 1].ZTop_m : 1e9f;
        if (z_m <= top) break;
        float z1 = top, z2 = (z_m < bot) ? z_m : bot;
        float dI = DensityIntegral_cm(z2 * 100.0f) - DensityIntegral_cm(z1 * 100.0f); /* cm */
        sigma += g_Profile.RhoInf_kgm3 * g_Profile.Layers[i].DensityScale * dI * 0.01f * g_Profile.Gravity_mps2;
    }
    return sigma;
}

/* ============================================================
 * Cone resistance by bearing capacity
 *
 * q = F (c Nc sc dc + sigma_v Nq sq dq + 0.5 gamma B Ngamma sgamma)
 * Nq, Nc, Ngamma after Vesic; shape factors for a circular tip;
 * Hansen's depth factors. The loose surface lowers phi and c with
 * the density ratio, which is what gives the near-surface gradient.
 * ============================================================ */
static float ConeResistance_Pa(float z_m, float area_m2, uint8 *layerOut)
{
    uint8 li = LayerAt(z_m);
    const RegolithLayer_t *L = &g_Profile.Layers[li];
    if (layerOut) *layerOut = li;

    float r    = DensityRatio(z_m);                 /* A/B at the surface, 1 at depth */
    float r0   = (g_Profile.RhoB_cm > 1e-3f) ? g_Profile.RhoA_cm / g_Profile.RhoB_cm : 1.0f;
    float loose = (r0 < 0.999f) ? (1.0f - r) / (1.0f - r0) : 0.0f;   /* 1 at the surface, 0 at depth */
    if (loose < 0.0f) loose = 0.0f;
    if (loose > 1.0f) loose = 1.0f;

    float phi_deg = L->FrictionAngle_deg - g_Profile.SurfaceSoftening_deg * loose;
    if (phi_deg < 0.0f) phi_deg = 0.0f;
    float c       = L->Cohesion_Pa * r * r;
    float phi     = phi_deg * PI_F / 180.0f;
    float t       = tanf(phi);

    float Nq, Nc, Ng;
    if (phi_deg > 0.5f)
    {
        float kp = tanf(PI_F / 4.0f + phi / 2.0f);
        Nq = expf(PI_F * t) * kp * kp;
        Nc = (Nq - 1.0f) / t;
        Ng = 2.0f * (Nq + 1.0f) * t;
    }
    else
    {
        Nq = 1.0f; Nc = 5.14f; Ng = 0.0f;
    }

    float B  = sqrtf(4.0f * area_m2 / PI_F);        /* tip diameter */
    float k  = (z_m <= B) ? z_m / B : atanf(z_m / B);
    float sq = 1.0f + t;
    float sc = 1.0f + Nq / Nc;
    float sg = 0.6f;
    float dq = 1.0f + 2.0f * t * (1.0f - sinf(phi)) * (1.0f - sinf(phi)) * k;
    float dc = 1.0f + 0.4f * k;

    float sigma = Overburden_Pa(z_m);
    float gamma = g_Profile.RhoInf_kgm3 * L->DensityScale * r * g_Profile.Gravity_mps2;

    float q = c * Nc * sc * dc + sigma * Nq * sq * dq + 0.5f * gamma * B * Ng * sg;
    return q * g_Profile.ConeFactor;
}

/* Everything that modulates the bare cone resistance at a depth. */
static float Resistance_Pa(float z_m, float area_m2, float rate_mps, RegolithResponse_t *R)
{
    uint8 li = 0;
    float q = ConeResistance_Pa(z_m, area_m2, &li);

    /* The tip engages the ground over its own height: no bearing area at
     * zero depth, the full area once the cone is buried. Without this a
     * cohesive surface bore more than the 5 N contact force at zero
     * penetration and the positioning move could never complete. */
    if (z_m < CONE_HEIGHT_M)
    {
        float e = z_m / CONE_HEIGHT_M;
        q *= e * e;
    }

    /* Surface crust (or the loose top) */
    if (g_Profile.CrustThickness_m > 0.0f && z_m < g_Profile.CrustThickness_m)
    {
        float u = z_m / g_Profile.CrustThickness_m;           /* full at the surface, gone at its bottom */
        float w = 1.0f - u * u;
        q *= 1.0f + (g_Profile.CrustFactor - 1.0f) * w;
    }

    /* Correlated variability along depth */
    if (g_Profile.VariabilitySigma > 0.0f)
    {
        q *= expf(g_Profile.VariabilitySigma * ValueNoise(g_Profile.RandomSeed, z_m, g_Profile.VariabilityLength_m));
    }

    /* Clasts */
    uint8 inClast = 0;
    for (uint8 i = 0; i < g_Profile.NumClasts; i++)
    {
        const RegolithClast_t *C = &g_Profile.Clasts[i];
        float half = C->Size_m * 0.5f;
        if (half <= 0.0f) continue;
        float u = (z_m - C->Z_m) / half;             /* -1 .. 1 across the clast */
        if (C->Immovable)
        {
            float into = z_m - (C->Z_m - half);
            if (into >= 0.0f)
            {
                float e = into / CLAST_RAMP_M;
                q += C->Strength_Pa * (e < 1.0f ? e : 1.0f);
                inClast = 1;
            }
        }
        else if (u > -1.0f && u < 1.0f)
        {
            float bump = C->Strength_Pa * (1.0f - u * u);
            if (bump > q) q = bump;
            inClast = 1;
        }
    }

    /* The hole from an earlier penetration: only shaft friction inside it.
     * The soil unloads over a few millimetres of rebound as the tip lifts,
     * not in one step, so the force rate on a retract stays physical. */
    uint8 inHole = 0;
    if (z_m < g_Compaction.MaxDepth_m)
    {
        float up = (g_Compaction.MaxDepth_m - z_m) / REBOUND_M;      /* 0 at the bottom, 1 at full rebound */
        if (up > 1.0f) up = 1.0f;
        q *= 1.0f - (1.0f - g_Profile.ShaftFriction) * up;
        inHole = (up >= 1.0f) ? 1 : 0;
    }

    /* Compaction from the load cycles, below the hole */
    if (!inHole) q *= g_Compaction.CurrentDensityMultiplier;

    /* Rate effect: a little stiffer when pushed fast */
    q *= 1.0f + 0.05f * fabsf(rate_mps) / 0.005f;

    if (R)
    {
        R->LayerIndex = li;
        R->InClast    = inClast;
        R->InHole     = inHole;
        R->InLayerTransition = (li > 0 && z_m - g_Profile.Layers[li].ZTop_m < 0.01f) ? 1 : 0;
    }
    return q;
}

/* ============================================================
 * Drawing a site
 * ============================================================ */
typedef struct
{
    float g;
    float rhoInf, rhoA, rhoB, rhoScaleLo, rhoScaleHi;
    float coneFactor, softening;
    int   layersLo, layersHi;
    float firstTopLo, firstTopHi;            /* where the second layer may begin */
    float cLo, cHi;                           /* cohesion, Pa, of the top layer */
    float cGrowth;                            /* each deeper layer: c times this */
    float phiLo, phiHi, phiGrowth_deg;
    float crustThickness, crustLo, crustHi;
    float varSigma, varLength;
    float clastsPerMetre, clastSizeLo, clastSizeHi, clastStrengthLo, clastStrengthHi, immovableProb;
    float shaft, compress;
    float kc, kphi, n;
    float floorDepth;                         /* > 0: a bin floor */
} PresetSpec_t;

static const PresetSpec_t k_Presets[REGOLITH_PRESET_COUNT] = {
    /* LUNAR_HIGHLAND: Carrier density law; dense, cohesive; rock fragments.
     * ConeFactor 4.5 fitted to the Apollo 16 SRP envelope (1.29 cm2 cone). */
    { 1.62f, 1920.0f, 12.2f, 18.0f, 0.95f, 1.05f, 4.5f, 8.0f, 1, 3, 0.04f, 0.20f,
      500.0f, 1200.0f, 1.4f, 40.0f, 45.0f, 1.5f, 0.04f, 0.2f, 0.4f, 0.25f, 0.015f,
      2.5f, 0.005f, 0.03f, 2.0e6f, 8.0e6f, 0.25f, 0.15f, 0.10f, 1400.0f, 820000.0f, 1.0f, 0.0f },
    /* LUNAR_MARE: a little lighter and weaker, fewer fragments */
    { 1.62f, 1800.0f, 12.2f, 18.0f, 0.95f, 1.05f, 4.5f, 8.0f, 1, 3, 0.04f, 0.20f,
      200.0f, 800.0f, 1.4f, 36.0f, 42.0f, 1.5f, 0.04f, 0.2f, 0.4f, 0.25f, 0.015f,
      1.0f, 0.005f, 0.02f, 2.0e6f, 6.0e6f, 0.25f, 0.15f, 0.12f, 1400.0f, 820000.0f, 1.0f, 0.0f },
    /* LUNAR_LOOSE: fresh ejecta, a rim: loose, weak, the gradient still there */
    { 1.62f, 1500.0f, 8.0f, 18.0f, 0.90f, 1.05f, 4.5f, 10.0f, 1, 2, 0.05f, 0.25f,
      100.0f, 300.0f, 1.3f, 30.0f, 36.0f, 1.0f, 0.04f, 0.2f, 0.4f, 0.30f, 0.02f,
      1.5f, 0.005f, 0.03f, 1.0e6f, 5.0e6f, 0.25f, 0.15f, 0.20f, 1400.0f, 820000.0f, 1.0f, 0.0f },
    /* EARTH_DRY_SAND: 1 g, cohesionless, uniform */
    { 9.81f, 1650.0f, 18.0f, 18.0f, 0.95f, 1.05f, 1.5f, 0.0f, 1, 2, 0.08f, 0.30f,
      0.0f, 0.0f, 1.0f, 31.0f, 36.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.20f, 0.02f,
      1.0f, 0.01f, 0.03f, 3.0e6f, 10.0e6f, 0.3f, 0.10f, 0.08f, 990.0f, 1528430.0f, 1.1f, 0.0f },
    /* EARTH_SANDY_LOAM: 1 g, some cohesion, a crust on top */
    { 9.81f, 1500.0f, 18.0f, 18.0f, 0.95f, 1.05f, 1.0f, 0.0f, 1, 3, 0.05f, 0.25f,
      3000.0f, 8000.0f, 1.3f, 26.0f, 32.0f, 1.0f, 0.02f, 1.2f, 1.8f, 0.30f, 0.02f,
      3.0f, 0.01f, 0.04f, 3.0e6f, 10.0e6f, 0.3f, 0.20f, 0.10f, 5270.0f, 1515040.0f, 0.7f, 0.0f },
    /* EARTH_FIRM_CLAY: 1 g, cohesion carries it, no friction */
    { 9.81f, 1800.0f, 18.0f, 18.0f, 0.95f, 1.05f, 1.0f, 0.0f, 1, 2, 0.08f, 0.30f,
      25000.0f, 60000.0f, 1.2f, 0.0f, 5.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.20f, 0.03f,
      0.5f, 0.01f, 0.04f, 5.0e6f, 20.0e6f, 0.5f, 0.30f, 0.05f, 13190.0f, 692150.0f, 0.5f, 0.0f },
    /* BENCH_SIMULANT: 1 g, a bin of JSC-1A/LHS-1-like simulant, one layer, a floor at 30 cm */
    { 9.81f, 1750.0f, 18.0f, 18.0f, 0.90f, 1.08f, 1.0f, 0.0f, 1, 1, 0.10f, 0.20f,
      500.0f, 1500.0f, 1.0f, 40.0f, 46.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.15f, 0.015f,
      0.0f, 0.01f, 0.02f, 3.0e6f, 6.0e6f, 0.0f, 0.15f, 0.10f, 1400.0f, 820000.0f, 1.0f, 0.30f },
};

static const char *k_PresetNames[REGOLITH_PRESET_COUNT] = {
    "lunar_highland", "lunar_mare", "lunar_loose",
    "earth_dry_sand", "earth_sandy_loam", "earth_firm_clay", "bench_simulant",
};

const char *Regolith_PresetName(RegolithPreset_t Preset)
{
    return (Preset < REGOLITH_PRESET_COUNT) ? k_PresetNames[Preset] : "?";
}

int32 Regolith_DrawSite(RegolithPreset_t Preset, uint32 Seed, RegolithProfile_t *Out)
{
    if (Out == NULL || Preset >= REGOLITH_PRESET_COUNT) return -1;
    const PresetSpec_t *P = &k_Presets[Preset];
    uint32 s = Seed * 2654435761u + 97u;

    memset(Out, 0, sizeof(*Out));
    Out->SiteId       = (uint16)(Seed & 0xFFFFu);
    Out->Preset       = (uint8)Preset;
    Out->Gravity_mps2 = P->g;
    Out->RhoInf_kgm3  = P->rhoInf;
    Out->RhoA_cm      = P->rhoA;
    Out->RhoB_cm      = P->rhoB;
    Out->ConeFactor   = P->coneFactor;
    Out->SurfaceSoftening_deg = P->softening;
    Out->RandomSeed   = Seed;

    /* Layers: the first at the surface; each next one deeper, denser and stronger */
    int n = P->layersLo + (int)(RandUniform(&s, 0.0f, 0.999f) * (float)(P->layersHi - P->layersLo + 1));
    if (n > REGOLITH_MAX_LAYERS) n = REGOLITH_MAX_LAYERS;
    Out->NumLayers = (uint8)n;
    float top = 0.0f;
    float c   = RandUniform(&s, P->cLo, P->cHi);
    float phi = RandUniform(&s, P->phiLo, P->phiHi);
    float scale = RandUniform(&s, P->rhoScaleLo, P->rhoScaleHi);
    for (int i = 0; i < n; i++)
    {
        if (i > 0)
        {
            top  += RandUniform(&s, P->firstTopLo, P->firstTopHi);
            c    *= P->cGrowth * RandUniform(&s, 0.8f, 1.2f);
            phi  += P->phiGrowth_deg * RandUniform(&s, 0.5f, 1.5f);
            scale *= RandUniform(&s, 1.0f, 1.06f);
        }
        Out->Layers[i].ZTop_m           = top;
        Out->Layers[i].Cohesion_Pa      = c;
        Out->Layers[i].FrictionAngle_deg = phi;
        Out->Layers[i].DensityScale     = scale;
    }

    Out->CrustThickness_m = P->crustThickness;
    Out->CrustFactor      = RandUniform(&s, P->crustLo, P->crustHi);
    Out->VariabilitySigma = P->varSigma * RandUniform(&s, 0.7f, 1.3f);
    Out->VariabilityLength_m = P->varLength;

    /* Clasts: Poisson-ish in the top 0.35 m */
    float expected = P->clastsPerMetre * 0.35f;
    int   nc = 0;
    for (int i = 0; i < REGOLITH_MAX_CLASTS - 1; i++)
    {
        if (RandUniform(&s, 0.0f, 1.0f) >= expected / (float)(REGOLITH_MAX_CLASTS - 1)) continue;
        RegolithClast_t *C = &Out->Clasts[nc++];
        C->Z_m         = RandUniform(&s, 0.02f, 0.35f);
        C->Size_m      = RandUniform(&s, P->clastSizeLo, P->clastSizeHi);
        C->Strength_Pa = RandUniform(&s, P->clastStrengthLo, P->clastStrengthHi);
        C->Immovable   = (C->Size_m > 0.02f && RandUniform(&s, 0.0f, 1.0f) < P->immovableProb) ? 1 : 0;
    }
    if (P->floorDepth > 0.0f)
    {
        RegolithClast_t *C = &Out->Clasts[nc++];
        C->Z_m = P->floorDepth; C->Size_m = 0.01f; C->Strength_Pa = 50.0e6f; C->Immovable = 1;
    }
    Out->NumClasts = (uint8)nc;

    Out->ShaftFriction   = P->shaft;
    Out->Compressibility = P->compress;
    Out->Bekker_kc   = P->kc;
    Out->Bekker_kphi = P->kphi;
    Out->Bekker_n    = P->n;
    return 0;
}

/* ============================================================
 * Init, load, default
 * ============================================================ */
int32 Regolith_Init(void)
{
    memset(&g_Profile, 0, sizeof(g_Profile));
    memset(&g_Compaction, 0, sizeof(g_Compaction));
    g_Initialized = 1;
    Regolith_SetDefaultProfile(0);
    return 0;
}

int32 Regolith_LoadProfile(const RegolithProfile_t *Profile)
{
    if (Profile == NULL) return -1;
    memcpy(&g_Profile, Profile, sizeof(RegolithProfile_t));
    Regolith_ResetCompaction();
    g_Compaction.SiteId = Profile->SiteId;
    g_RngState = Profile->RandomSeed;
    return 0;
}

int32 Regolith_SetSite(RegolithPreset_t Preset, uint32 Seed)
{
    RegolithProfile_t p;
    if (Regolith_DrawSite(Preset, Seed, &p) != 0) return -1;
    return Regolith_LoadProfile(&p);
}

int32 Regolith_SetDefaultProfile(uint16 SiteId)
{
    int32 rc = Regolith_SetSite(REGOLITH_PRESET_LUNAR_HIGHLAND, (uint32)SiteId * 7919u + 42u);
    g_Profile.SiteId = SiteId;
    g_Compaction.SiteId = SiteId;
    return rc;
}

/* ============================================================
 * Penetration response
 * ============================================================ */
int32 Regolith_GetPenetrationResponse(float Depth_m, float ToolArea_m2,
                                       float Rate_mps, RegolithResponse_t *Response)
{
    if (!g_Initialized || Response == NULL) return -1;
    if (Depth_m < 0.0f) Depth_m = 0.0f;
    if (ToolArea_m2 < 1e-7f) ToolArea_m2 = 1e-7f;

    memset(Response, 0, sizeof(RegolithResponse_t));

    float q = Resistance_Pa(Depth_m, ToolArea_m2, Rate_mps, Response);

    /* Local stiffness: dq/dz over a millimetre, before the hole deepens */
    float dz = 0.001f;
    float q2 = Resistance_Pa(Depth_m + dz, ToolArea_m2, Rate_mps, NULL);

    if (Depth_m > g_Compaction.MaxDepth_m) g_Compaction.MaxDepth_m = Depth_m;

    Response->ReactionForce_N           = q * ToolArea_m2;
    Response->PenetrationResistance_kPa = q / 1000.0f;
    Response->Sinkage_m                 = Depth_m;
    Response->StiffnessProxy            = (q2 - q) * ToolArea_m2 / dz;
    return 0;
}

/* ============================================================
 * Plate-load response (Bekker)
 * ============================================================ */
int32 Regolith_GetLoadResponse(float AppliedForce_N, float PlateArea_m2,
                                RegolithResponse_t *Response)
{
    if (!g_Initialized || Response == NULL) return -1;
    if (AppliedForce_N < 0.0f) AppliedForce_N = 0.0f;
    if (PlateArea_m2 < 1e-6f) PlateArea_m2 = 1e-6f;

    memset(Response, 0, sizeof(RegolithResponse_t));

    float b = sqrtf(PlateArea_m2 / PI_F);          /* equivalent radius */
    float k = (g_Profile.Bekker_kc / b + g_Profile.Bekker_kphi) * g_Compaction.CurrentDensityMultiplier;
    float n = g_Profile.Bekker_n;
    if (g_Profile.VariabilitySigma > 0.0f)
    {
        k *= expf(0.5f * g_Profile.VariabilitySigma * ValueNoise(g_Profile.RandomSeed ^ 0xA5A5u, 0.0f, 1.0f));
    }

    float pressure = AppliedForce_N / PlateArea_m2;
    float sinkage  = (k > 0.0f && pressure > 0.0f) ? powf(pressure / k, 1.0f / n) : 0.0f;
    float stiffness = (sinkage > 1e-6f) ? k * PlateArea_m2 * n * powf(sinkage, n - 1.0f) : 0.0f;

    Response->ReactionForce_N           = AppliedForce_N;
    Response->PenetrationResistance_kPa = pressure / 1000.0f;
    Response->Sinkage_m                 = sinkage;
    Response->StiffnessProxy            = stiffness;
    return 0;
}

/* ============================================================
 * Disturbance tracking
 * ============================================================ */
int32 Regolith_RecordLoadCycle(float PeakForce_N, float PeakSinkage_m)
{
    if (!g_Initialized) return -1;

    g_Compaction.LoadCyclesApplied++;
    g_Compaction.LastIncrementalSinkage_m = PeakSinkage_m - g_Compaction.CumulativeCompaction_m;
    if (g_Compaction.LastIncrementalSinkage_m < 0.0f)
        g_Compaction.LastIncrementalSinkage_m = 0.0f;
    g_Compaction.CumulativeCompaction_m = PeakSinkage_m;

    g_Compaction.CurrentDensityMultiplier += g_Profile.Compressibility * (PeakForce_N / 500.0f);
    if (g_Compaction.CurrentDensityMultiplier > 2.0f)
        g_Compaction.CurrentDensityMultiplier = 2.0f;
    return 0;
}

int32 Regolith_GetCompactionState(RegolithCompactionState_t *State)
{
    if (!g_Initialized || State == NULL) return -1;
    memcpy(State, &g_Compaction, sizeof(RegolithCompactionState_t));
    return 0;
}

int32 Regolith_ResetCompaction(void)
{
    uint16 site = g_Compaction.SiteId;
    memset(&g_Compaction, 0, sizeof(g_Compaction));
    g_Compaction.SiteId = site;
    g_Compaction.CurrentDensityMultiplier = 1.0f;
    g_Compaction.MaxDepth_m = 0.0f;
    return 0;
}

const RegolithProfile_t* Regolith_GetCurrentProfile(void)
{
    return &g_Profile;
}
