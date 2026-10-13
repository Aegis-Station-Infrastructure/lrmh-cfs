/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file ds_filter_table.c
 * @brief LRMH DS (Data Storage) Packet Filter Table
 *
 * Per ADR-0016 (report I/O via DS).
 *
 * Two MIDs are captured into rotated file sets:
 *
 *   LRMH_REPORT_SITE_RECORD_MID  (0x08AA, binary records)
 *     → file set 0 (LRMH_DS_FILE_RECORDS_IDX), 1 MB / 24 h rotation
 *
 *   LRMH_REPORT_SITE_HUMAN_MID   (0x08AB, human-readable text)
 *     → file set 1 (LRMH_DS_FILE_HUMAN_IDX), 512 KB / 24 h rotation
 *
 * Both filters use DS_BY_COUNT with N=1, X=1 (capture every
 * matching message — there is no down-sampling at the DS layer).
 * Site result publication is rate-limited at the source by
 * lrmh_report (one per site, not high frequency).
 */

#include "cfe_sb_api_typedefs.h"
#include "cfe_tbl_filedef.h"
#include "ds_platform_cfg.h"
#include "ds_tbl.h"
#include "ds_msg.h"

#include "lrmh_msgids.h"
#include "lrmh_platform_cfg.h"

DS_FilterTable_t DS_FilterTable = {
    /* Descriptor capped at DS_DESCRIPTOR_BUFSIZE = 32 bytes */
    .Descriptor = "LRMH site result capture",
    .Packet = {

        /* ── Site result binary records (ADR-0016) ──────────── */
        {
            .MessageID = CFE_SB_MSGID_WRAP_VALUE(LRMH_REPORT_SITE_RECORD_MID),
            .Filter = {
                {
                    .FileTableIndex = LRMH_DS_FILE_RECORDS_IDX,
                    .FilterType     = DS_BY_COUNT,
                    .Algorithm_N    = 1,    /* every message */
                    .Algorithm_X    = 1,
                    .Algorithm_O    = 0
                },
                /* Other filter slots unused */
                { DS_UNUSED, 0, 0, 0, 0 },
                { DS_UNUSED, 0, 0, 0, 0 },
                { DS_UNUSED, 0, 0, 0, 0 }
            }
        },

        /* ── Site result human-readable text (ADR-0016) ─────── */
        {
            .MessageID = CFE_SB_MSGID_WRAP_VALUE(LRMH_REPORT_SITE_HUMAN_MID),
            .Filter = {
                {
                    .FileTableIndex = LRMH_DS_FILE_HUMAN_IDX,
                    .FilterType     = DS_BY_COUNT,
                    .Algorithm_N    = 1,
                    .Algorithm_X    = 1,
                    .Algorithm_O    = 0
                },
                { DS_UNUSED, 0, 0, 0, 0 },
                { DS_UNUSED, 0, 0, 0, 0 },
                { DS_UNUSED, 0, 0, 0, 0 }
            }
        }

        /* Remaining DS_PACKETS_IN_FILTER_TABLE slots unused.
         * If future ADRs add new DS-captured MIDs, append here. */
    }
};

/* The DS app registers its filter table as "FILTER_TBL"; the
 * cFE TBL service prepends the app name → "DS.FILTER_TBL".
 * The output filename matches what ds_internal_cfg.h points
 * DEFAULT_DS_INTERNAL_DEF_FILTER_FILENAME at (/cf/ds_filter_tbl.tbl). */
CFE_TBL_FILEDEF(DS_FilterTable, DS.FILTER_TBL, LRMH DS filter, ds_filter_tbl.tbl)
