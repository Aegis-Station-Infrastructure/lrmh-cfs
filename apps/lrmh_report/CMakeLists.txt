# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## apps/lrmh_report/ — LRMH site result publication app
##
## Per ADR-0016 (report I/O via DS): no in-FSW file writes,
## no in-FSW JSON. Site results publish on two SB MIDs for
## the standard cFS DS app to capture into rotated files.
##

project(CFE_LRMH_REPORT C)

if(COMMAND add_cfe_app)
    add_cfe_app(lrmh_report fsw/src/lrmh_report_app.c)

    target_sources(lrmh_report PRIVATE
        ${MISSION_SOURCE_DIR}/lrmh_defs/lrmh_seq_token.c
    )

    target_include_directories(lrmh_report PUBLIC fsw/inc)
    target_include_directories(lrmh_report PRIVATE
        fsw/src
        ${MISSION_SOURCE_DIR}/lrmh_defs
        ${MISSION_SOURCE_DIR}/apps/lrmh_proc/fsw/inc
    )

    target_link_libraries(lrmh_report m)
endif()

if(BUILD_TESTING)
    add_executable(test_lrmh_report
        fsw/test/test_lrmh_report.c
    )
    target_include_directories(test_lrmh_report PRIVATE
        ${CMAKE_SOURCE_DIR}/cfe_stubs/test_inc
        ${CMAKE_SOURCE_DIR}/cfe_stubs/inc
        fsw/inc
        fsw/src
        ${CMAKE_SOURCE_DIR}/apps/lrmh_proc/fsw/inc
        ${CMAKE_SOURCE_DIR}/msg
        ${CMAKE_SOURCE_DIR}/cfs_defs
        ${CMAKE_SOURCE_DIR}/hal/include
    )
    target_link_libraries(test_lrmh_report PRIVATE m)
    target_compile_options(test_lrmh_report PRIVATE -Wall -Wextra -g)
    add_test(NAME lrmh_report_unit_tests COMMAND test_lrmh_report)
endif()
