# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## hal/sim/ — standalone HAL sim executable
##
## Per ADR-0001 / ADR-0003, this runs as a separate process from
## the cFS image. cFS apps that need actuator/sensor I/O link the
## hal/client/ UDP backend, which sends requests here.
##

add_executable(lrmh_hal_sim
    lrmh_hal_sim.c
)

target_link_libraries(lrmh_hal_sim
    PRIVATE
        lrmh_hal_iface
        lrmh_sim_models
        m
)

target_include_directories(lrmh_hal_sim PRIVATE
    ${CMAKE_SOURCE_DIR}/cfe_stubs/inc
)

target_compile_definitions(lrmh_hal_sim PRIVATE
    LRMH_SIM_STANDALONE=1
)
