# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## apps/lrmh_seq/ — LRMH sequencer application
##
## Per ADR-0006: owns the site-test phase machine, the
## sequence-token stamping, and the recovery-tier dispatch.
##

project(CFE_LRMH_SEQ C)

if(COMMAND add_cfe_app)
    add_cfe_app(lrmh_seq fsw/src/lrmh_seq_app.c)

    target_sources(lrmh_seq PRIVATE
        ${MISSION_SOURCE_DIR}/lrmh_defs/lrmh_seq_token.c
    )

    target_include_directories(lrmh_seq PUBLIC fsw/inc)
    target_include_directories(lrmh_seq PRIVATE
        fsw/src
        ${MISSION_SOURCE_DIR}/lrmh_defs
        ${MISSION_SOURCE_DIR}/apps/lrmh_cmd/fsw/inc
        ${MISSION_SOURCE_DIR}/apps/lrmh_ctrl/fsw/inc
        ${MISSION_SOURCE_DIR}/apps/lrmh_sense/fsw/inc
        ${MISSION_SOURCE_DIR}/apps/lrmh_proc/fsw/inc
        ${MISSION_SOURCE_DIR}/apps/lrmh_report/fsw/inc
        ${MISSION_SOURCE_DIR}/apps/lrmh_fault/fsw/inc
    )
endif()

if(BUILD_TESTING)
    add_executable(test_lrmh_seq
        fsw/test/test_lrmh_seq.c
    )
    target_include_directories(test_lrmh_seq PRIVATE
        ${CMAKE_SOURCE_DIR}/cfe_stubs/test_inc
        ${CMAKE_SOURCE_DIR}/cfe_stubs/inc
        fsw/inc
        fsw/src
        ${CMAKE_SOURCE_DIR}/apps/lrmh_cmd/fsw/inc
        ${CMAKE_SOURCE_DIR}/apps/lrmh_ctrl/fsw/inc
        ${CMAKE_SOURCE_DIR}/apps/lrmh_sense/fsw/inc
        ${CMAKE_SOURCE_DIR}/apps/lrmh_proc/fsw/inc
        ${CMAKE_SOURCE_DIR}/apps/lrmh_report/fsw/inc
        ${CMAKE_SOURCE_DIR}/apps/lrmh_fault/fsw/inc
        ${CMAKE_SOURCE_DIR}/msg
        ${CMAKE_SOURCE_DIR}/cfs_defs
        ${CMAKE_SOURCE_DIR}/hal/include
    )
    target_compile_options(test_lrmh_seq PRIVATE -Wall -Wextra -g)
    add_test(NAME lrmh_seq_unit_tests COMMAND test_lrmh_seq)
endif()
