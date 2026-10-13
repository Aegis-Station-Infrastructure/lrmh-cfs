# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
##
## msg/ — LRMH message ID and shared message header library
##
## Header-only. Mission apps and the HAL client both include from
## here. The message struct definitions live in per-app *_msg.h
## files under apps/ — this directory only contains the shared
## headers (lrmh_msgids.h, lrmh_seq_token.h).
##

add_library(lrmh_msg INTERFACE)

target_include_directories(lrmh_msg INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

##
## lrmh_seq_token library — real implementation per ADR-0007.
## Linked into the cFS image; standalone tests stub it locally.
##
add_library(lrmh_seq_token STATIC
    lrmh_seq_token.c
)
target_include_directories(lrmh_seq_token PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/cfe_stubs/inc
)

