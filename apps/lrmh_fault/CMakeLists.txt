# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## apps/lrmh_fault/ — LRMH fault orchestrator
##
## Per ADR-0017: thin orchestrator. Detection moves to LC tables; this app
## dispatches recovery tiers per ADR-0008.
##

project(CFE_LRMH_FAULT C)

if(COMMAND add_cfe_app)
    add_cfe_app(lrmh_fault fsw/src/lrmh_fault_app.c)

    target_include_directories(lrmh_fault PUBLIC fsw/inc)
    target_include_directories(lrmh_fault PRIVATE
        fsw/src
        ${MISSION_SOURCE_DIR}/lrmh_defs
        ${MISSION_SOURCE_DIR}/apps/lrmh_sense/fsw/inc
        ${MISSION_SOURCE_DIR}/apps/lrmh_cmd/fsw/inc
    )
endif()

if(BUILD_TESTING)
    add_executable(test_lrmh_fault
        fsw/test/test_lrmh_fault.c
    )
    target_include_directories(test_lrmh_fault PRIVATE
        ${CMAKE_SOURCE_DIR}/cfe_stubs/test_inc
        ${CMAKE_SOURCE_DIR}/cfe_stubs/inc
        fsw/inc
        fsw/src
        ${CMAKE_SOURCE_DIR}/apps/lrmh_sense/fsw/inc
        ${CMAKE_SOURCE_DIR}/apps/lrmh_cmd/fsw/inc
        ${CMAKE_SOURCE_DIR}/msg
        ${CMAKE_SOURCE_DIR}/cfs_defs
        ${CMAKE_SOURCE_DIR}/hal/include
    )
    target_compile_options(test_lrmh_fault PRIVATE -Wall -Wextra -g)
    add_test(NAME lrmh_fault_unit_tests COMMAND test_lrmh_fault)
endif()
