# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## LRMH cFS Flight Software — root CMakeLists.txt (standalone host build)
##
## This tree builds the host-native pieces: the sim physics models, the
## standalone HAL simulator (hal/sim/lrmh_hal_sim), the regolith_probe
## tool, and the per-app unit tests. It needs no NASA cFS checkout.
##
##   cmake -B build -S . -DTARGET=sim
##   cmake --build build
##   (cd build && ctest --output-on-failure)
##
## The flight image itself is built by the NASA cFS build system after
## install.sh has copied the apps into a cFS v7.0.0 tree:
##
##   ./install.sh ~/lrmh-cfs-build
##   cd ~/lrmh-cfs-build && make SIMULATION=native prep && make && make install
##
## Then scripts/lrmh_launch.sh brings up HAL sim + image + bridge.
##

cmake_minimum_required(VERSION 3.13)
project(aegis_lrmh_fsw C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

##
## Target selection (sim / bench / flight)
##
if(NOT DEFINED TARGET)
    set(TARGET "sim" CACHE STRING "HAL backend target: sim, bench, flight")
endif()
message(STATUS "LRMH target: ${TARGET}")

##
## Tests — must be enabled BEFORE add_subdirectory() so the
## per-component add_test() calls register with ctest.
##
enable_testing()
set(BUILD_TESTING ON CACHE BOOL "Build standalone unit tests")

##
## Component subdirectories
##
add_subdirectory(sim)             # physics models (regolith, actuator, sensor)
add_subdirectory(hal)             # HAL include + selected backend + sim executable
add_subdirectory(msg)             # message definitions
add_subdirectory(tables)          # LC, SC, HS, DS table sources

# Mission apps. In this host tree each one builds only its standalone
# unit-test target; the flight image is built by the cFS build system.
add_subdirectory(apps/lrmh_cmd)
add_subdirectory(apps/lrmh_seq)
add_subdirectory(apps/lrmh_ctrl)
add_subdirectory(apps/lrmh_sense)
add_subdirectory(apps/lrmh_proc)
add_subdirectory(apps/lrmh_report)
add_subdirectory(apps/lrmh_hk)
add_subdirectory(apps/lrmh_fault)
