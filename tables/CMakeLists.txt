# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## tables/ — LRMH LC, SC, HS and DS table sources
##
## These sources are copied into the NASA support apps' table
## directories by install.sh and compiled into cFE-loadable .tbl files
## by the cFS build. They encode the FDIR policy (ADR-0008, ADR-0013,
## ADR-0017) and the DS storage layout (ADR-0016):
##
##   lc_watchpoints.c    -> LC.LC_WDT      (apps/lc/fsw/tables/lc_def_wdt.c)
##   lc_actionpoints.c   -> LC.LC_ADT      (apps/lc/fsw/tables/lc_def_adt.c)
##   sc/sc_rts001..009.c -> SC RTS tables
##   sc/sc_ats1.c        -> SC ATS table
##   hs_table.c          -> HS.AMT_DEFAULT (apps/hs/fsw/tables/hs_amt.c)
##   ds_filter_table.c   -> DS.FilterTable
##   ds_file_table.c     -> DS.FileTable
##
## There is nothing to build here in the standalone host tree.
##
message(STATUS "tables/: sources are installed into the cFS tree by install.sh")
