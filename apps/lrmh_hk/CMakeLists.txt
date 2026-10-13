# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## apps/lrmh_hk/ — LRMH housekeeping aggregator
##
## Per ADR-0006: aggregates per-app status TLMs into the
## instrument-level HK packet. Does NOT call HAL.
##

project(CFE_LRMH_HK C)

if(COMMAND add_cfe_app)
    add_cfe_app(lrmh_hk fsw/src/lrmh_hk_app.c)

    target_include_directories(lrmh_hk PUBLIC fsw/inc)
    target_include_directories(lrmh_hk PRIVATE
        fsw/src
        ${MISSION_SOURCE_DIR}/lrmh_defs
        ${MISSION_SOURCE_DIR}/apps/lrmh_cmd/fsw/inc
        ${MISSION_SOURCE_DIR}/apps/lrmh_seq/fsw/inc
        ${MISSION_SOURCE_DIR}/apps/lrmh_ctrl/fsw/inc
        ${MISSION_SOURCE_DIR}/apps/lrmh_sense/fsw/inc
        ${MISSION_SOURCE_DIR}/apps/lrmh_fault/fsw/inc
    )
endif()

if(BUILD_TESTING)
    add_executable(test_lrmh_hk
        fsw/test/test_lrmh_hk.c
    )
    target_include_directories(test_lrmh_hk PRIVATE
        ${CMAKE_SOURCE_DIR}/cfe_stubs/test_inc
        ${CMAKE_SOURCE_DIR}/cfe_stubs/inc
        fsw/inc
        fsw/src
        ${CMAKE_SOURCE_DIR}/apps/lrmh_seq/fsw/inc
        ${CMAKE_SOURCE_DIR}/apps/lrmh_ctrl/fsw/inc
        ${CMAKE_SOURCE_DIR}/apps/lrmh_sense/fsw/inc
        ${CMAKE_SOURCE_DIR}/apps/lrmh_cmd/fsw/inc
        ${CMAKE_SOURCE_DIR}/apps/lrmh_fault/fsw/inc
        ${CMAKE_SOURCE_DIR}/msg
        ${CMAKE_SOURCE_DIR}/cfs_defs
        ${CMAKE_SOURCE_DIR}/hal/include
    )
    target_compile_options(test_lrmh_hk PRIVATE -Wall -Wextra -g)
    add_test(NAME lrmh_hk_unit_tests COMMAND test_lrmh_hk)
endif()
