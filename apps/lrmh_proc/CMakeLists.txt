# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## apps/lrmh_proc/ — LRMH signal processing application
##
## Per ADR-0002 (regolith opaque): signal-derived engineering
## quantities only, no soil model.
##

project(CFE_LRMH_PROC C)

if(COMMAND add_cfe_app)
    add_cfe_app(lrmh_proc fsw/src/lrmh_proc_app.c)

    target_sources(lrmh_proc PRIVATE
        ${MISSION_SOURCE_DIR}/lrmh_defs/lrmh_seq_token.c
    )

    target_include_directories(lrmh_proc PUBLIC fsw/inc)
    target_include_directories(lrmh_proc PRIVATE
        fsw/src
        ${MISSION_SOURCE_DIR}/lrmh_defs
        ${MISSION_SOURCE_DIR}/apps/lrmh_sense/fsw/inc
    )

    target_link_libraries(lrmh_proc m)
endif()

if(BUILD_TESTING)
    add_executable(test_lrmh_proc
        fsw/test/test_lrmh_proc.c
    )
    target_include_directories(test_lrmh_proc PRIVATE
        ${CMAKE_SOURCE_DIR}/cfe_stubs/test_inc
        ${CMAKE_SOURCE_DIR}/cfe_stubs/inc
        fsw/inc
        fsw/src
        ${CMAKE_SOURCE_DIR}/apps/lrmh_sense/fsw/inc
        ${CMAKE_SOURCE_DIR}/msg
        ${CMAKE_SOURCE_DIR}/cfs_defs
        ${CMAKE_SOURCE_DIR}/hal/include
    )
    target_link_libraries(test_lrmh_proc PRIVATE m)
    target_compile_options(test_lrmh_proc PRIVATE -Wall -Wextra -g)
    add_test(NAME lrmh_proc_unit_tests COMMAND test_lrmh_proc)
endif()
