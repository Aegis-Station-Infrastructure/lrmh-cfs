/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file common_types.h
 * @brief Standalone-build stand-in for the OSAL common_types.h header
 *
 * The sim physics models and the HAL simulator are built host-native
 * without a NASA cFS tree present. They use the OSAL fixed-width
 * aliases (int32, uint8, ...). This header provides those aliases so
 * the standalone cmake build and the per-app unit tests compile with
 * no cFS checkout. Inside a real cFS workspace the OSAL header is
 * used instead; nothing from this directory is installed there.
 */

#ifndef LRMH_STUB_COMMON_TYPES_H
#define LRMH_STUB_COMMON_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef int8_t   int8;
typedef int16_t  int16;
typedef int32_t  int32;
typedef int64_t  int64;
typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

typedef intptr_t  cpuaddr;
typedef size_t    cpusize;
typedef ptrdiff_t cpudiff;

#endif /* LRMH_STUB_COMMON_TYPES_H */
