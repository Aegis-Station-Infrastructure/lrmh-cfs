# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## apps/lrmh_cmd/ — LRMH command surface application
##
## Per ADR-0006 (app split): owns the mode FSM, the dispatch cache,
## and the token epoch bumps for INIT/SAFE/restart paths.
##

project(CFE_LRMH_CMD C)

##
## When built inside the cFS workspace, use add_cfe_app(). When
## built standalone (host unit tests), use
## add_executable() for the test binary only.
##
if(COMMAND add_cfe_app)
    ##
    ## cFS workspace build. Mission apps are MODULE_LIBRARY shared
    ## objects loaded by ES; they cannot link to other cFS modules.
    ## Dependencies (seq_token, hal client) are compiled into each
    ## app's source list via target_sources. See
    ## install.sh for the file layout in the
    ## workspace.
    ##
    add_cfe_app(lrmh_cmd fsw/src/lrmh_cmd_app.c)

    target_sources(lrmh_cmd PRIVATE
        ${MISSION_SOURCE_DIR}/lrmh_defs/lrmh_seq_token.c
        ${MISSION_SOURCE_DIR}/lrmh_defs/lrmh_hal_client.c
    )

    target_include_directories(lrmh_cmd PUBLIC fsw/inc)
    target_include_directories(lrmh_cmd PRIVATE
        fsw/src
        ${MISSION_SOURCE_DIR}/lrmh_defs
    )
endif()

##
## Standalone unit test target — builds and runs without cFS bundle.
## The cfe_stubs/inc/ path comes BEFORE the real headers so the
## stubs intercept HAL and seq_token calls.
##
if(BUILD_TESTING)
    add_executable(test_lrmh_cmd
        fsw/test/test_lrmh_cmd.c
    )
    target_include_directories(test_lrmh_cmd PRIVATE
        ${CMAKE_SOURCE_DIR}/cfe_stubs/test_inc
        ${CMAKE_SOURCE_DIR}/cfe_stubs/inc   # ← FIRST: stubs win
        fsw/inc
        fsw/src
        ${CMAKE_SOURCE_DIR}/msg
        ${CMAKE_SOURCE_DIR}/cfs_defs
        ${CMAKE_SOURCE_DIR}/hal/include
    )
    target_compile_options(test_lrmh_cmd PRIVATE -Wall -Wextra -g)
    add_test(NAME lrmh_cmd_unit_tests COMMAND test_lrmh_cmd)
endif()
