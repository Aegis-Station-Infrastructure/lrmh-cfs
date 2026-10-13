# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## hal/bench/ — bench HAL backend (bench actuator + sensor stack)
##
## Standalone UDP server, structurally identical to hal/sim/. The
## cFS image's hal/client/ talks UDP to this process when running
## against bench hardware. Per ADR-0001, mission apps unchanged
## across sim/bench/flight; only this backend swaps.
##
## Built when the parent hal/CMakeLists.txt is configured with
## TARGET=bench. Currently a scaffold — see lrmh_hal_bench.c
## header for the partner-integration plan.
##

add_executable(lrmh_hal_bench
    lrmh_hal_bench.c
)

target_link_libraries(lrmh_hal_bench
    PRIVATE
        lrmh_hal_iface
        m
)

# The standalone bench executable doesn't need cFS app machinery,
# but the HAL header includes cfe.h for fixed-width types. Use the
# same stub shim as hal/sim/.
target_include_directories(lrmh_hal_bench PRIVATE
    ${CMAKE_SOURCE_DIR}/cfe_stubs/inc
)

target_compile_definitions(lrmh_hal_bench PRIVATE
    LRMH_BENCH_STANDALONE=1
)
