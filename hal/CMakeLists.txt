# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## hal/ — LRMH HAL header + selected backend
##
## The header in hal/include/lrmh_hal.h is the contract per ADR-0001
## (apps unchanged). The backend selected via the TARGET cmake var
## (sim/bench/flight) is what mission apps actually link against.
##

##
## Public header
##
add_library(lrmh_hal_iface INTERFACE)
target_include_directories(lrmh_hal_iface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../msg
    ${CMAKE_CURRENT_SOURCE_DIR}/../cfs_defs
)

##
## Backend selection
##
if("${TARGET}" STREQUAL "sim")
    add_subdirectory(sim)
    add_subdirectory(client)
elseif("${TARGET}" STREQUAL "bench")
    # Scaffold backend — UDP envelope + dispatch in place; per-call
    # controller integration is left as TODO(bench). The cFS-side
    # hal/client/ is unchanged across backends per ADR-0001.
    add_subdirectory(bench)
    add_subdirectory(client)
elseif("${TARGET}" STREQUAL "flight")
    message(WARNING "hal/flight/ not yet implemented")
else()
    message(FATAL_ERROR "Unknown TARGET: ${TARGET}")
endif()
