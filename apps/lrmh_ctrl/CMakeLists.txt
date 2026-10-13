# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## apps/lrmh_ctrl/ — LRMH actuator control application
##
## Per ADR-0006: actuator command translator. First 2.0 receiver
## of token-validated commands (ADR-0007).
##

project(CFE_LRMH_CTRL C)

if(COMMAND add_cfe_app)
    add_cfe_app(lrmh_ctrl fsw/src/lrmh_ctrl_app.c)

    target_sources(lrmh_ctrl PRIVATE
        ${MISSION_SOURCE_DIR}/lrmh_defs/lrmh_seq_token.c
        ${MISSION_SOURCE_DIR}/lrmh_defs/lrmh_hal_client.c
    )

    target_include_directories(lrmh_ctrl PUBLIC fsw/inc)
    target_include_directories(lrmh_ctrl PRIVATE
        fsw/src
        ${MISSION_SOURCE_DIR}/lrmh_defs
    )
endif()

if(BUILD_TESTING)
    add_executable(test_lrmh_ctrl
        fsw/test/test_lrmh_ctrl.c
    )
    target_include_directories(test_lrmh_ctrl PRIVATE
        ${CMAKE_SOURCE_DIR}/cfe_stubs/test_inc
        ${CMAKE_SOURCE_DIR}/cfe_stubs/inc
        fsw/inc
        fsw/src
        ${CMAKE_SOURCE_DIR}/msg
        ${CMAKE_SOURCE_DIR}/cfs_defs
        ${CMAKE_SOURCE_DIR}/hal/include
    )
    target_compile_options(test_lrmh_ctrl PRIVATE -Wall -Wextra -g)
    add_test(NAME lrmh_ctrl_unit_tests COMMAND test_lrmh_ctrl)
endif()
