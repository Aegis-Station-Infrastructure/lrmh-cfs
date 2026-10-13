# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## sim/ — physics models for the LRMH HAL sim backend
##
## Regolith, actuator and sensor models. Per ADR-0002
## (regolith opaque) these are sim-only and never imported by mission
## apps; they are linked only into hal/sim/lrmh_hal_sim and the
## regolith_probe tool, both host-native executables. The models use the
## OSAL fixed-width type aliases, provided for the standalone build by
## cfe_stubs/inc/common_types.h.
##

add_library(lrmh_sim_models STATIC
    actuator_model.c
    regolith_model.c
    sensor_model.c
)

target_include_directories(lrmh_sim_models PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(lrmh_sim_models PUBLIC
    ${CMAKE_SOURCE_DIR}/cfe_stubs/inc    # common_types.h stand-in
)

target_compile_definitions(lrmh_sim_models PRIVATE
    LRMH_SIM_STANDALONE=1
)

## regolith_probe — push a cone through a drawn site and print what it
## feels; lays a lunar preset against the Apollo 16 penetrometer records.
add_executable(regolith_probe tools/regolith_probe.c)
target_link_libraries(regolith_probe PRIVATE lrmh_sim_models m)
