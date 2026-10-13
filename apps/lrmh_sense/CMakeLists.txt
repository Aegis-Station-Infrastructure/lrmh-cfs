# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## apps/lrmh_sense/ — LRMH sensor acquisition application
##
## Per ADR-0006: 100 Hz sensor reader. First user of ADR-0009
## (sensor freshness floor) and ADR-0011 (host telemetry
## subscription).
##

project(CFE_LRMH_SENSE C)

if(COMMAND add_cfe_app)
    add_cfe_app(lrmh_sense fsw/src/lrmh_sense_app.c)

    target_sources(lrmh_sense PRIVATE
        ${MISSION_SOURCE_DIR}/lrmh_defs/lrmh_seq_token.c
        ${MISSION_SOURCE_DIR}/lrmh_defs/lrmh_hal_client.c
    )

    target_include_directories(lrmh_sense PUBLIC fsw/inc)
    target_include_directories(lrmh_sense PRIVATE
        fsw/src
        ${MISSION_SOURCE_DIR}/lrmh_defs
    )
endif()

if(BUILD_TESTING)
    add_executable(test_lrmh_sense
        fsw/test/test_lrmh_sense.c
    )
    target_include_directories(test_lrmh_sense PRIVATE
        ${CMAKE_SOURCE_DIR}/cfe_stubs/test_inc
        ${CMAKE_SOURCE_DIR}/cfe_stubs/inc
        fsw/inc
        fsw/src
        ${CMAKE_SOURCE_DIR}/msg
        ${CMAKE_SOURCE_DIR}/cfs_defs
        ${CMAKE_SOURCE_DIR}/hal/include
    )
    target_compile_options(test_lrmh_sense PRIVATE -Wall -Wextra -g)
    add_test(NAME lrmh_sense_unit_tests COMMAND test_lrmh_sense)
endif()
