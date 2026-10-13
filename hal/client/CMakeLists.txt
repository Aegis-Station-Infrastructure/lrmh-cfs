# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## hal/client/ — UDP client backend, linked into the cFS image
##
## Implements the LRMH_HAL_* functions by sending requests to a
## HAL server (sim today, bench/flight later). The mission apps
## link against this and never know about the UDP transport.
##

add_library(lrmh_hal_client STATIC
    lrmh_hal_client.c
)

target_link_libraries(lrmh_hal_client
    PUBLIC
        lrmh_hal_iface
)

# When building standalone (no cFS workspace), the cfe_stubs
# header provides cfe.h. The in-workspace cFS build replaces
# this with the real cFE headers via the cFS app build pipeline.
target_include_directories(lrmh_hal_client PRIVATE
    ${CMAKE_SOURCE_DIR}/cfe_stubs/inc
    ${CMAKE_SOURCE_DIR}/cfs_defs
)

# When building inside the cFS workspace, this library is added to
# each LRMH mission app's link list via the per-app CMakeLists.
# In the standalone host build this library is compiled but not
# linked into anything (the unit tests stub the HAL).
