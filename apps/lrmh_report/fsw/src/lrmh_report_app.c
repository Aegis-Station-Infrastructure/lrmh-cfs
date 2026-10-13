/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_report_app.c
 * @brief LRMH lrmh_report application — implementation
 *
 * Site result assembly and publication. Per ADR-0016: NO direct
 * file I/O, NO in-FSW JSON. Site results publish on two SB MIDs:
 *
 *   LRMH_REPORT_SITE_RECORD_MID  binary record (DS captures)
 *   LRMH_REPORT_SITE_HUMAN_MID   human text (DS captures)
 *
 * The binary `LRMH_SiteResultRecord_t` is the source of truth.
 * Ground tools render JSON from binary.
 */

#include "lrmh_report_app.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

LRMH_REPORT_AppData_t g_ReportApp;

/* ============================================================
 * Forward declarations
 * ============================================================ */
static int32 LRMH_REPORT_Init(void);
static void  LRMH_REPORT_ProcessMsg(const CFE_SB_Buffer_t *BufPtr);
static void  LRMH_REPORT_AssembleRecord(uint16 site_id, uint8 quality,
                                        LRMH_SiteResultRecord_t *out);
static void  LRMH_REPORT_PublishRecord(const LRMH_SiteResultRecord_t *rec);
static void  LRMH_REPORT_PublishHumanText(uint16 site_id, const char *text,
                                          size_t text_len);

/* ============================================================
 * App main
 * ============================================================ */
void LRMH_REPORT_AppMain(void)
{
    int32 Status;
    CFE_SB_Buffer_t *BufPtr = NULL;
    uint32 RunStatus = CFE_ES_RunStatus_APP_RUN;

    Status = LRMH_REPORT_Init();
    if (Status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog("LRMH_REPORT: Init failed\n");
        CFE_ES_ExitApp(CFE_ES_RunStatus_APP_ERROR);
        return;
    }

    CFE_EVS_SendEvent(LRMH_REPORT_INIT_INF_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_REPORT: initialized (DS-routed I/O)");

    while (CFE_ES_RunLoop(&RunStatus) == true) {
        /* 1 s timeout so HS AppMon sees the exec counter tick even
         * when no messages arrive (PEND_FOREVER would leave the
         * counter static and trip HS's 30 s threshold). */
        Status = CFE_SB_ReceiveBuffer(&BufPtr, g_ReportApp.CmdPipe, 1000);
        if (Status == CFE_SUCCESS && BufPtr != NULL) {
            LRMH_REPORT_ProcessMsg(BufPtr);
        }
        else if (Status != CFE_SB_TIME_OUT) {
            /* A software-bus receive error is not something the app can
             * recover from in place: event it and let ES restart the app
             * (HS policy) rather than spin on a broken pipe. */
            g_ReportApp.ErrCount++;
            CFE_EVS_SendEvent(LRMH_REPORT_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "LRMH_REPORT: SB receive error 0x%08lX, exiting",
                              (unsigned long)Status);
            RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }
    }

    CFE_ES_ExitApp(RunStatus);
}

/* ============================================================
 * Initialization
 * ============================================================ */
static int32 LRMH_REPORT_Init(void)
{
    int32 Status;
    memset(&g_ReportApp, 0, sizeof(g_ReportApp));

    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) return Status;

    Status = CFE_SB_CreatePipe(&g_ReportApp.CmdPipe,
                               LRMH_REPORT_PIPE_DEPTH, "LRMH_REPORT_PIPE");
    if (Status != CFE_SUCCESS) return Status;

    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_REPORT_CMD_MID),
                     g_ReportApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEQ_TO_REPORT_MID),
                     g_ReportApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_PROC_TLM_MID),
                     g_ReportApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_SEQ_SAFE_MID),
                     g_ReportApp.CmdPipe);
    CFE_SB_Subscribe(CFE_SB_ValueToMsgId(LRMH_TOKEN_RESET_MID),
                     g_ReportApp.CmdPipe);

    return CFE_SUCCESS;
}

/* ============================================================
 * Phase result ingestion
 * ============================================================ */
void LRMH_REPORT_IngestPhaseResult(const LRMH_ProcResultTlm_t *result)
{
    if (g_ReportApp.PhaseCount >= LRMH_MAX_PHASES_PER_TEST) {
        g_ReportApp.ErrCount++;
        return;
    }
    g_ReportApp.Phases[g_ReportApp.PhaseCount] = result->Result;
    g_ReportApp.PhaseCount++;
    g_ReportApp.CurrentSiteId = result->SiteId;

    CFE_EVS_SendEvent(LRMH_REPORT_PHASE_INGEST_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_REPORT: ingested phase %u (%u so far)",
                      (unsigned)result->Result.phase,
                      (unsigned)g_ReportApp.PhaseCount);
}

/* ============================================================
 * Site result record assembly
 *
 * Aggregates per-phase data into the binary record. The
 * derived science quantities are computed from the per-phase
 * results: penetration resistance is the max across the
 * penetration phases, stiffness is the highest stiffness proxy,
 * etc.
 * ============================================================ */
static void LRMH_REPORT_AssembleRecord(uint16 site_id, uint8 quality,
                                       LRMH_SiteResultRecord_t *out)
{
    memset(out, 0, sizeof(*out));

    out->version          = 1;
    out->session_id       = g_ReportApp.CurrentSessionId;
    out->sequence_number  = g_ReportApp.CurrentSequenceNumber;
    out->site_id          = site_id;
    out->paired_site_id   = g_ReportApp.PairedSiteId;
    out->test_profile_id  = g_ReportApp.TestProfileId;
    out->tool_id          = g_ReportApp.ToolId;
    out->outcome          = quality;
    /* Site-level storage_mode is the OR of per-phase modes per
     * ADR-0015: if any phase ran in STREAMING, the site is
     * tagged STREAMING for ground tooling. Computed below from
     * the per-phase results. */
    out->storage_mode     = 0;
    out->start_time_us    = g_ReportApp.SiteStartTime_us;
    out->end_time_us      = g_ReportApp.SiteEndTime_us;

    /* Copy per-phase summaries */
    out->phase_count = g_ReportApp.PhaseCount;
    for (uint8 i = 0; i < g_ReportApp.PhaseCount &&
                      i < LRMH_MAX_PHASES_PER_TEST; i++) {
        const LRMH_ProcPhaseResult_t *p = &g_ReportApp.Phases[i];
        LRMH_SitePhaseSummary_t *s = &out->phases[i];
        s->phase                = p->phase;
        s->quality              = p->quality;
        s->sample_count         = p->sample_count;
        s->max_force_n          = p->max_force_n;
        s->max_sinkage_m        = p->max_sinkage_m;
        s->stiffness_proxy_npm  = p->stiffness_proxy_npm;
        s->bearing_proxy        = p->bearing_proxy;
        s->layer_break_depth_m  = p->layer_break_depth_m;
        s->duration_s           = p->duration_s;
        if (p->storage_mode != 0) {
            out->storage_mode = 1;  /* any STREAMING phase tags site */
        }
    }

    /* Aggregated science quantities — max across penetration phases */
    float max_pr_kpa = 0.0f;
    float mean_pr_kpa = 0.0f;
    int   pr_count = 0;
    float max_stiff = 0.0f;
    float max_bearing = 0.0f;
    float layer_break = 0.0f;
    float compaction = 0.0f;
    float variability = 0.0f;

    for (uint8 i = 0; i < g_ReportApp.PhaseCount; i++) {
        const LRMH_ProcPhaseResult_t *p = &g_ReportApp.Phases[i];
        if (p->phase == LRMH_PHASE_BASELINE_PEN ||
            p->phase == LRMH_PHASE_POST_PEN) {
            /* Penetration resistance = peak force / tip area, in kPa.
             * The nominal 1 cm^2 cone area is used; a per-tool table
             * would replace it. */
            float pr_kpa = p->max_force_n / LRMH_TOOL_TIP_AREA_M2 / 1000.0f;
            if (pr_kpa > max_pr_kpa) max_pr_kpa = pr_kpa;
            mean_pr_kpa += pr_kpa;
            pr_count++;
        }
        if (p->stiffness_proxy_npm > max_stiff) {
            max_stiff = p->stiffness_proxy_npm;
        }
        if (p->bearing_proxy > max_bearing) {
            max_bearing = p->bearing_proxy;
        }
        if (p->layer_break_depth_m > 0.0f && layer_break == 0.0f) {
            layer_break = p->layer_break_depth_m;
        }
        if (fabsf(p->compaction_delta) > fabsf(compaction)) {
            compaction = p->compaction_delta;
        }
        if (p->variability_index > variability) {
            variability = p->variability_index;
        }
    }
    if (pr_count > 0) mean_pr_kpa /= (float)pr_count;

    out->penetration_resistance_mean_kpa = mean_pr_kpa;
    out->penetration_resistance_peak_kpa = max_pr_kpa;
    out->load_sinkage_stiffness_kn_per_m = max_stiff / 1000.0f;
    /* lrmh_proc reports bearing as force-at-1-cm / LRMH_PROC_BEARING_REF_N;
     * convert back to a force and express it over the tip area, in kPa. */
    out->bearing_proxy_kpa               = max_bearing * LRMH_PROC_BEARING_REF_N
                                           / LRMH_TOOL_TIP_AREA_M2 / 1000.0f;
    out->layer_break_depth_m             = layer_break;
    out->compaction_delta                = compaction;
    out->variability                     = variability;
    out->fault_flags                     = 0;  /* not populated in this release */
    out->recovery_tier                   = 0;  /* not populated in this release */
}

/* ============================================================
 * Human text formatter (per ADR-0016 template)
 *
 * Returns the number of characters written (excluding the
 * terminating NUL). Truncates at LRMH_MAX_REPORT_TEXT_LEN.
 *
 * Exposed for unit testing — the test verifies the output
 * against expected substrings.
 * ============================================================ */
int LRMH_REPORT_FormatHumanText(const LRMH_SiteResultRecord_t *rec,
                                 char *out, size_t out_max)
{
    if (!rec || !out || out_max == 0) return 0;

    /* Match the canonical LRMH_Quality_t enum names from
     * msg/lrmh_msg.h so ground tooling can lookup the same way. */
    const char *outcome_str =
        (rec->outcome == LRMH_QUALITY_VALID)   ? "VALID"   :
        (rec->outcome == LRMH_QUALITY_PARTIAL) ? "PARTIAL" :
                                                  "FAULT";

    int n = snprintf(out, out_max,
        "=== LRMH Site Result ===\n"
        "Session: 0x%04X  Site: %u (paired with site %u)\n"
        "Profile: %u  Tool: %u\n"
        "Outcome: %s\n"
        "Storage: %s\n"
        "\n"
        "Start: %llu us\n"
        "End:   %llu us\n"
        "\n"
        "Host pose at start: X=%.2f Y=%.2f Z=%.2f\n"
        "                    R=%.3f P=%.3f Y=%.3f\n"
        "\n"
        "Per-phase (%u):\n",
        (unsigned)rec->session_id,
        (unsigned)rec->site_id,
        (unsigned)rec->paired_site_id,
        (unsigned)rec->test_profile_id,
        (unsigned)rec->tool_id,
        outcome_str,
        rec->storage_mode == 0 ? "RAW" : "STREAMING",
        (unsigned long long)rec->start_time_us,
        (unsigned long long)rec->end_time_us,
        (double)rec->host_x_m, (double)rec->host_y_m, (double)rec->host_z_m,
        (double)rec->host_roll_rad, (double)rec->host_pitch_rad,
        (double)rec->host_yaw_rad,
        (unsigned)rec->phase_count);
    if (n < 0 || (size_t)n >= out_max) return n;

    for (uint8 i = 0; i < rec->phase_count && (size_t)n < out_max; i++) {
        const LRMH_SitePhaseSummary_t *p = &rec->phases[i];
        const char *q = (p->quality == LRMH_QUALITY_VALID) ? "ok" :
                        (p->quality == LRMH_QUALITY_PARTIAL) ? "partial" :
                                                                "fault";
        int wrote = snprintf(out + n, out_max - (size_t)n,
            "  phase=%u %-7s n=%-5u maxF=%.1f N depth=%.4f m stiff=%.1f N/m\n",
            (unsigned)p->phase, q, (unsigned)p->sample_count,
            (double)p->max_force_n, (double)p->max_sinkage_m,
            (double)p->stiffness_proxy_npm);
        if (wrote < 0) break;
        n += wrote;
    }

    if ((size_t)n < out_max) {
        n += snprintf(out + n, out_max - (size_t)n,
            "\n"
            "Derived:\n"
            "  Penetration resistance (mean): %.2f\n"
            "  Penetration resistance (peak): %.2f\n"
            "  Load-sinkage stiffness (kN/m): %.2f\n"
            "  Bearing proxy (kPa):           %.2f\n"
            "  Layer break depth (m):         %.4f\n"
            "  Compaction delta:              %.3f\n"
            "  Variability:                   %.3f\n"
            "\n"
            "Faults: 0x%04X  Recovery tier: %u\n"
            "========================\n",
            (double)rec->penetration_resistance_mean_kpa,
            (double)rec->penetration_resistance_peak_kpa,
            (double)rec->load_sinkage_stiffness_kn_per_m,
            (double)rec->bearing_proxy_kpa,
            (double)rec->layer_break_depth_m,
            (double)rec->compaction_delta,
            (double)rec->variability,
            (unsigned)rec->fault_flags,
            (unsigned)rec->recovery_tier);
    }

    return n;
}

/* ============================================================
 * Site result generation — assemble + publish both MIDs
 * ============================================================ */
void LRMH_REPORT_GenerateSiteResult(uint16 site_id, uint8 quality)
{
    /* Assemble the binary record */
    LRMH_SiteResultRecord_t rec;
    LRMH_REPORT_AssembleRecord(site_id, quality, &rec);

    /* Publish binary record on LRMH_REPORT_SITE_RECORD_MID */
    LRMH_REPORT_PublishRecord(&rec);

    /* Format and publish human text on LRMH_REPORT_SITE_HUMAN_MID */
    char text[LRMH_MAX_REPORT_TEXT_LEN];
    int text_len = LRMH_REPORT_FormatHumanText(&rec, text, sizeof(text));
    if (text_len < 0 || (size_t)text_len >= sizeof(text)) {
        CFE_EVS_SendEvent(LRMH_REPORT_TEXT_OVERFLOW_EID,
                          CFE_EVS_EventType_ERROR,
                          "LRMH_REPORT: human text overflow");
        text_len = (int)sizeof(text) - 1;
    }
    LRMH_REPORT_PublishHumanText(site_id, text, (size_t)text_len);

    g_ReportApp.CmdCount++;
    CFE_EVS_SendEvent(LRMH_REPORT_GENERATE_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_REPORT: site %u published (quality=%u, %u phases)",
                      (unsigned)site_id, (unsigned)quality,
                      (unsigned)g_ReportApp.PhaseCount);

    /* Reset accumulator for the next site */
    g_ReportApp.PhaseCount = 0;
}

/* ============================================================
 * Tag PARTIAL (ground command): publish the site so far as PARTIAL
 * ============================================================ */
void LRMH_REPORT_TagPartial(uint16 site_id)
{
    LRMH_REPORT_GenerateSiteResult(site_id, LRMH_QUALITY_PARTIAL);
    g_ReportApp.PartialTaggedCount++;
    CFE_EVS_SendEvent(LRMH_REPORT_PARTIAL_TAGGED_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_REPORT: site %u tagged PARTIAL",
                      (unsigned)site_id);
}

/* ============================================================
 * Token reset
 * ============================================================ */
void LRMH_REPORT_OnTokenReset(uint8 reason)
{
    memset(&g_ReportApp.AcceptedToken, 0, sizeof(g_ReportApp.AcceptedToken));
    g_ReportApp.ResetObserved = true;
    /* A reset for a new test or an INIT starts a fresh site: drop any
     * phases left over from a site that never generated (a T3 that
     * safed the instrument). SAFE entry keeps them so the FAULT-quality
     * record that follows still carries what was collected. */
    if (reason != (uint8)LRMH_TOKEN_RESET_SAFE_ENTRY) {
        g_ReportApp.PhaseCount = 0;
    }
    CFE_EVS_SendEvent(LRMH_REPORT_TOKEN_RESET_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_REPORT: token reset observed");
}

/* ============================================================
 * Publication helpers
 * ============================================================ */
static void LRMH_REPORT_PublishRecord(const LRMH_SiteResultRecord_t *rec)
{
    /* CFE_MSG_Init zeroes the whole buffer before writing the CCSDS
     * header: init the destination first, then copy the payload. */
    LRMH_SiteResultRecord_t out;
    CFE_MSG_Init(CFE_MSG_PTR(out.TelemetryHeader),
                 CFE_SB_ValueToMsgId(LRMH_REPORT_SITE_RECORD_MID),
                 sizeof(out));
    const size_t hdr = sizeof(out.TelemetryHeader);
    memcpy((uint8 *)&out + hdr,
           (const uint8 *)rec + hdr,
           sizeof(out) - hdr);
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(out.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(out.TelemetryHeader), true);

    /* Echo the populated aggregated science fields and per-phase
     * summaries via EVS so they can be cross-checked against
     * lrmh_proc events without needing the binary downlink. */
    CFE_EVS_SendEvent(LRMH_REPORT_RECORD_PUB_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_REPORT[rec]: site=%u sess=0x%04x seq=%u "
                      "outcome=%u phases=%u storage=%u",
                      (unsigned)rec->site_id, (unsigned)rec->session_id,
                      (unsigned)rec->sequence_number,
                      (unsigned)rec->outcome,
                      (unsigned)rec->phase_count,
                      (unsigned)rec->storage_mode);
    CFE_EVS_SendEvent(LRMH_REPORT_HUMAN_PUB_EID,
                      CFE_EVS_EventType_INFORMATION,
                      "LRMH_REPORT[agg]: pen_mean=%.2f peak=%.2f "
                      "stiff=%.2f bear=%.2f layer=%.4f",
                      (double)rec->penetration_resistance_mean_kpa,
                      (double)rec->penetration_resistance_peak_kpa,
                      (double)rec->load_sinkage_stiffness_kn_per_m,
                      (double)rec->bearing_proxy_kpa,
                      (double)rec->layer_break_depth_m);
    for (uint8 p = 0; p < LRMH_MAX_PHASES_PER_TEST; p++) {
        const LRMH_SitePhaseSummary_t *ph = &rec->phases[p];
        if (ph->sample_count == 0) continue;
        CFE_EVS_SendEvent(LRMH_REPORT_PUB_ERR_EID,
                          CFE_EVS_EventType_INFORMATION,
                          "LRMH_REPORT[ph%u]: n=%u MaxF=%.2f Sink=%.4f "
                          "Stiff=%.2f Bear=%.2f LayerBrk=%.4f dur=%.2f",
                          (unsigned)p, (unsigned)ph->sample_count,
                          (double)ph->max_force_n,
                          (double)ph->max_sinkage_m,
                          (double)ph->stiffness_proxy_npm,
                          (double)ph->bearing_proxy,
                          (double)ph->layer_break_depth_m,
                          (double)ph->duration_s);
    }
}

static void LRMH_REPORT_PublishHumanText(uint16 site_id, const char *text,
                                         size_t text_len)
{
    LRMH_SiteResultHumanTlm_t tlm;
    memset(&tlm, 0, sizeof(tlm));
    CFE_MSG_Init(CFE_MSG_PTR(tlm.TelemetryHeader),
                 CFE_SB_ValueToMsgId(LRMH_REPORT_SITE_HUMAN_MID),
                 sizeof(tlm));
    tlm.site_id  = site_id;
    tlm.text_len = (uint16)text_len;
    if (text_len >= sizeof(tlm.text)) text_len = sizeof(tlm.text) - 1;
    memcpy(tlm.text, text, text_len);
    tlm.text[text_len] = '\0';
    CFE_SB_TimeStampMsg(CFE_MSG_PTR(tlm.TelemetryHeader));
    CFE_SB_TransmitMsg(CFE_MSG_PTR(tlm.TelemetryHeader), true);
}

/* ============================================================
 * Message router
 * ============================================================ */
static void LRMH_REPORT_ProcessMsg(const CFE_SB_Buffer_t *BufPtr)
{
    CFE_SB_MsgId_t MsgId;
    CFE_MSG_GetMsgId(&BufPtr->Msg, &MsgId);

    switch (CFE_SB_MsgIdToValue(MsgId)) {

    case LRMH_PROC_TLM_MID:
        LRMH_REPORT_IngestPhaseResult((const LRMH_ProcResultTlm_t *)BufPtr);
        break;

    case LRMH_SEQ_TO_REPORT_MID: {
        const LRMH_ReportTriggerCmd_t *trig =
            (const LRMH_ReportTriggerCmd_t *)BufPtr;
        LRMH_SeqToken_t incoming;
        incoming.Epoch    = LRMH_SEQ_TOKEN_UNPACK_EPOCH(trig->SeqTokenPacked);
        incoming.SeqState = LRMH_SEQ_TOKEN_UNPACK_STATE(trig->SeqTokenPacked);
        incoming.SeqCount = LRMH_SEQ_TOKEN_UNPACK_COUNT(trig->SeqTokenPacked);

        /* Latch identification carried by the trigger so the
         * assemble step can stamp the binary record correctly. */
        g_ReportApp.CurrentSessionId      = trig->SessionId;
        g_ReportApp.CurrentSequenceNumber = trig->SequenceNumber;
        g_ReportApp.PairedSiteId          = trig->PairedSiteId;
        g_ReportApp.TestProfileId         = trig->TestProfileId;
        g_ReportApp.ToolId                = trig->ToolId;
        g_ReportApp.SiteStartTime_us      = trig->TestStartTime_us;
        {
            /* End time is now (the moment lrmh_report received the
             * trigger). lrmh_seq stamps start-of-test in HandleStartTest;
             * end-of-test = trigger arrival → wall duration = end - start. */
            CFE_TIME_SysTime_t now = CFE_TIME_GetTime();
            g_ReportApp.SiteEndTime_us =
                (uint64)now.Seconds * 1000000ULL +
                (((uint64)now.Subseconds * 1000000ULL) >> 32);
        }

        if (g_ReportApp.ResetObserved) {
            g_ReportApp.AcceptedToken = incoming;
            g_ReportApp.ResetObserved = false;
            LRMH_REPORT_GenerateSiteResult(trig->SiteId, trig->QualityFlag);
            break;
        }

        LRMH_TokenValidationResult_t r = LRMH_SeqToken_Validate(
            &incoming, &g_ReportApp.AcceptedToken, false);
        if (r == LRMH_TOKEN_ACCEPT || r == LRMH_TOKEN_UNCONDITIONAL_ACCEPT) {
            LRMH_REPORT_GenerateSiteResult(trig->SiteId, trig->QualityFlag);
        } else {
            CFE_EVS_SendEvent(LRMH_REPORT_STALE_CMD_EID,
                              CFE_EVS_EventType_INFORMATION,
                              "LRMH_REPORT: stale trigger dropped (rule=%d)",
                              (int)r);
        }
        break;
    }

    case LRMH_REPORT_CMD_MID: {
        /* Ground command — bypass token validation per ADR-0007 */
        CFE_MSG_FcnCode_t fcn;
        CFE_MSG_GetFcnCode(&BufPtr->Msg, &fcn);
        if (fcn == LRMH_REPORT_NOOP_CC) {
            g_ReportApp.CmdCount++;
            CFE_EVS_SendEvent(LRMH_REPORT_NOOP_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "LRMH_REPORT: NOOP Cmd=%u Err=%u phases=%u",
                              (unsigned)g_ReportApp.CmdCount, (unsigned)g_ReportApp.ErrCount,
                              (unsigned)g_ReportApp.PhaseCount);
        } else if (fcn == LRMH_REPORT_RESET_CC) {
            g_ReportApp.CmdCount = 0;
            g_ReportApp.ErrCount = 0;
            CFE_EVS_SendEvent(LRMH_REPORT_RESET_INF_EID, CFE_EVS_EventType_INFORMATION,
                              "LRMH_REPORT: counters reset");
        } else if (fcn == LRMH_REPORT_TAG_PARTIAL_CC) {
            const LRMH_ReportTriggerCmd_t *trig =
                (const LRMH_ReportTriggerCmd_t *)BufPtr;
            LRMH_REPORT_TagPartial(trig->SiteId);
        } else if (fcn == LRMH_REPORT_GENERATE_CC) {
            const LRMH_ReportTriggerCmd_t *trig =
                (const LRMH_ReportTriggerCmd_t *)BufPtr;
            LRMH_REPORT_GenerateSiteResult(trig->SiteId, trig->QualityFlag);
        }
        break;
    }

    case LRMH_TOKEN_RESET_MID:
        LRMH_REPORT_OnTokenReset(((const LRMH_TokenResetTlm_t *)BufPtr)->ResetReason);
        break;

    case LRMH_SEQ_SAFE_MID:
        /* SAFE does not publish; lrmh_seq's T3 path sends the report
         * trigger with quality FAULT. */
        break;

    default:
        break;
    }
}
