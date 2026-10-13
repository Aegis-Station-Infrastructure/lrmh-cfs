/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file ds_file_table.c
 * @brief LRMH DS (Data Storage) Destination File Table
 *
 * Per ADR-0016 (report I/O via DS).
 *
 * Two file sets:
 *
 *   Index 0 (LRMH_DS_FILE_RECORDS_IDX) — binary site records
 *     Pathname: /cf/lrmh/data/
 *     Basename: lrmh_site_
 *     Ext:      .bin
 *     Rotation: 1 MB max size, 24 h max age, time-based filenames
 *
 *   Index 1 (LRMH_DS_FILE_HUMAN_IDX) — human-readable text
 *     Pathname: /cf/lrmh/reports/
 *     Basename: lrmh_human_
 *     Ext:      .txt
 *     Rotation: 512 KB max size, 24 h max age
 *
 * Both file sets are FM-managed for downlink. The standard cFS
 * FM app picks them up via the cFE filesystem and ground can
 * request them by name through normal FM commands.
 */

#include "cfe_tbl_filedef.h"
#include "ds_mission_cfg.h"
#include "ds_platform_cfg.h"
#include "ds_tbl.h"
#include "ds_msg.h"

#include "lrmh_platform_cfg.h"

DS_DestFileTable_t DS_DestFileTable = {
    .Descriptor = "LRMH DS file destinations",
    .File = {

        /* ── File set 0: binary site records ──────────────── */
        [LRMH_DS_FILE_RECORDS_IDX] = {
            .Movename       = "",
            .Pathname       = LRMH_DATA_OUTPUT_DIR,    /* "/cf/lrmh/data/" */
            .Basename       = "lrmh_site_",
            .Extension      = ".bin",
            .FileNameType   = DS_BY_TIME,
            .EnableState    = DS_ENABLED,
            .MaxFileSize    = LRMH_DS_MAX_FILE_SIZE_BYTES,    /* 1 MB */
            .MaxFileAge     = LRMH_DS_MAX_FILE_AGE_SEC,       /* 24 h */
            .SequenceCount  = 0
        },

        /* ── File set 1: human-readable text ─────────────── */
        [LRMH_DS_FILE_HUMAN_IDX] = {
            .Movename       = "",
            .Pathname       = LRMH_REPORT_OUTPUT_DIR,  /* "/cf/lrmh/reports/" */
            .Basename       = "lrmh_human_",
            .Extension      = ".txt",
            .FileNameType   = DS_BY_TIME,
            .EnableState    = DS_ENABLED,
            .MaxFileSize    = (LRMH_DS_MAX_FILE_SIZE_BYTES / 2),  /* 512 KB */
            .MaxFileAge     = LRMH_DS_MAX_FILE_AGE_SEC,
            .SequenceCount  = 0
        }

        /* Remaining DS_DEST_FILE_CNT slots default to disabled.
         * If future ADRs add file sets (e.g., a streaming sample
         * archive per ADR-0015 phase 2), define them here. */
    }
};

/* DS registers its destination table as "FILE_TBL"; cFE TBL
 * service prepends the app name → "DS.FILE_TBL". Output
 * filename matches /cf/ds_file_tbl.tbl from ds_internal_cfg.h. */
CFE_TBL_FILEDEF(DS_DestFileTable, DS.FILE_TBL, LRMH DS files, ds_file_tbl.tbl)
