/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file cfe.h
 * @brief Minimal cFE stub header for standalone build/test.
 *
 * Provides the subset of cFE API types and functions used by Aegis
 * mission apps. Replace with real cFE headers when integrating into
 * a full cFS workspace.
 */

#ifndef CFE_H
#define CFE_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* ── Basic types ───────────────────────────────────────────────────── */

/*
 * cFE common_types.h aliases (int32, uint8, ...). Real cFE gets these
 * from the OSAL common_types.h header; the standalone build provides a
 * stand-in in this directory so sources written against the cFE
 * convention compile with no cFS tree present.
 */
#include "common_types.h"

typedef int32_t  CFE_Status_t;

/*
 * Real cFE wraps message IDs in a struct for type safety. The stub
 * uses the same shape so the same source compiles in both contexts.
 */
typedef uint32_t CFE_SB_MsgId_Atom_t;
typedef struct {
    CFE_SB_MsgId_Atom_t Value;
} CFE_SB_MsgId_t;

static inline CFE_SB_MsgId_t CFE_SB_ValueToMsgId(CFE_SB_MsgId_Atom_t v)
{
    CFE_SB_MsgId_t mid;
    mid.Value = v;
    return mid;
}

static inline CFE_SB_MsgId_Atom_t CFE_SB_MsgIdToValue(CFE_SB_MsgId_t mid)
{
    return mid.Value;
}

/* CFE_SB_Buffer_t is a union in real cFE — first field is the base
   message type so callers can access &buf->Msg as a CFE_MSG_Message_t*. */

typedef uint32_t CFE_ES_AppId_t;
typedef uint32_t CFE_ES_TaskId_t;

/*
 * cFE message headers.
 *
 * In real cFE, CFE_MSG_CommandHeader_t and CFE_MSG_TelemetryHeader_t are
 * opaque structs whose layout is determined by the PSP. For the standalone
 * stub build, we make both header types embed the base CFE_MSG_Message_t
 * as their first field so they are layout-compatible (16 bytes) and a
 * pointer to either can be cast to CFE_MSG_Message_t *.
 *
 * Mission app _msg.h files should use CFE_MSG_CommandHeader_t for command
 * structs and CFE_MSG_TelemetryHeader_t for telemetry structs. This makes
 * the same source compile against both the stub and the real cFE.
 */
typedef struct {
    uint8_t  data[16];
} CFE_MSG_Message_t;

typedef struct {
    CFE_MSG_Message_t Msg;
} CFE_MSG_CommandHeader_t;

typedef struct {
    CFE_MSG_Message_t Msg;
} CFE_MSG_TelemetryHeader_t;

/*
 * CFE_MSG_PTR — converts a command or telemetry header struct (passed
 * by reference, NOT by pointer) to a CFE_MSG_Message_t pointer. Matches
 * the macro defined in cfe/modules/msg/option_inc/default_cfe_msg_hdr_pri.h
 * so the same source compiles against both stub and real cFE.
 */
#define CFE_MSG_PTR(shdr) (&((shdr).Msg))

/* Software Bus generic buffer — union with the base message at field 0. */
typedef union {
    CFE_MSG_Message_t Msg;
    long long int     LongInt;     /* alignment for long ints     */
    long double       LongDouble;  /* alignment for long doubles  */
} CFE_SB_Buffer_t;

typedef struct {
    CFE_SB_MsgId_t id;
} CFE_SB_PipeId_t;

typedef uint16_t CFE_MSG_Size_t;
typedef uint16_t CFE_MSG_FcnCode_t;

/* ── Return codes ──────────────────────────────────────────────────── */

#define CFE_SUCCESS           ((CFE_Status_t)0)
#define CFE_STATUS_WRONG_MSG_LENGTH  ((CFE_Status_t)-1)
#define CFE_SB_TIME_OUT       ((CFE_Status_t)0x0A00)
#define CFE_SB_PIPE_RD_ERR    ((CFE_Status_t)0x0A01)
#define CFE_SB_NO_MESSAGE     ((CFE_Status_t)0x0A02)

/* ── cFE ES (Executive Services) stubs ─────────────────────────────── */

#define CFE_ES_PERF_ENTRY_MID  0
#define CFE_ES_PERF_EXIT_MID   0

static inline CFE_Status_t CFE_ES_RegisterApp(void) { return CFE_SUCCESS; }
static inline void CFE_ES_PerfLogEntry(uint32_t id) { (void)id; }
static inline void CFE_ES_PerfLogExit(uint32_t id) { (void)id; }
static inline CFE_Status_t CFE_ES_RunLoop(const uint32_t *counter)
{
    (void)counter;
    return CFE_SUCCESS;
}
static inline void CFE_ES_WaitForStartupSync(uint32_t ms) { (void)ms; }
static inline void CFE_ES_ExitApp(uint32_t reason) { (void)reason; }
static inline CFE_Status_t CFE_ES_GetAppID(CFE_ES_AppId_t *id)
{
    *id = 1;
    return CFE_SUCCESS;
}

#define CFE_ES_RunStatus_APP_RUN   1
#define CFE_ES_RunStatus_APP_ERROR 2
#define CFE_ES_RunStatus_APP_EXIT  3

/* ── cFE EVS (Event Services) stubs ────────────────────────────────── */

typedef enum {
    CFE_EVS_EventType_INFORMATION = 1,
    CFE_EVS_EventType_DEBUG       = 2,
    CFE_EVS_EventType_ERROR       = 3,
    CFE_EVS_EventType_CRITICAL    = 4,
} CFE_EVS_EventType_Enum_t;

typedef struct {
    uint16_t EventID;
    uint16_t EventType;
} CFE_EVS_BinFilter_t;

#define CFE_EVS_EventFilter_BINARY 0

static inline CFE_Status_t CFE_EVS_Register(const CFE_EVS_BinFilter_t *filters,
                                             uint16_t count, uint16_t scheme)
{
    (void)filters; (void)count; (void)scheme;
    return CFE_SUCCESS;
}

static inline void CFE_ES_WriteToSysLog(const char *fmt, ...) { (void)fmt; }

static inline CFE_Status_t CFE_EVS_SendEvent(uint16_t id, uint16_t type,
                                              const char *fmt, ...)
{
    (void)id; (void)type; (void)fmt;
    return CFE_SUCCESS;
}

/* ── cFE SB (Software Bus) stubs ───────────────────────────────────── */

#define CFE_SB_DEFAULT_QOS    0
#define CFE_SB_PEND_FOREVER   (-1)

static inline CFE_Status_t CFE_SB_CreatePipe(CFE_SB_PipeId_t *pipe,
                                              uint16_t depth,
                                              const char *name)
{
    (void)pipe; (void)depth; (void)name;
    return CFE_SUCCESS;
}

static inline CFE_Status_t CFE_SB_Subscribe(CFE_SB_MsgId_t mid,
                                             CFE_SB_PipeId_t pipe)
{
    (void)mid; (void)pipe;
    return CFE_SUCCESS;
}

static inline CFE_Status_t CFE_SB_ReceiveBuffer(CFE_SB_Buffer_t **buf,
                                                  CFE_SB_PipeId_t pipe,
                                                  int32_t timeout)
{
    (void)pipe; (void)timeout;
    if (buf) *buf = NULL;
    return CFE_SB_TIME_OUT;
}

static inline CFE_Status_t CFE_SB_TransmitMsg(const void *msg,
                                                uint32_t inc_seq)
{
    (void)msg; (void)inc_seq;
    return CFE_SUCCESS;
}

static inline CFE_Status_t CFE_SB_TimeStampMsg(void *msg)
{
    (void)msg;
    return CFE_SUCCESS;
}

/* ── cFE MSG (Message) stubs ───────────────────────────────────────── */
/*
 * Real cFE accepts a CFE_MSG_Message_t * here. Our stubs use void * so
 * callers can pass &foo.CommandHeader or &foo.TelemetryHeader without an
 * explicit cast — those are layout-compatible with CFE_MSG_Message_t in
 * both the stub and real cFE.
 */

static inline void CFE_MSG_Init(void *msg,
                                 CFE_SB_MsgId_t mid,
                                 CFE_MSG_Size_t size)
{
    memset(msg, 0, size);
    (void)mid;
}

static inline void CFE_MSG_SetMsgId(void *msg,
                                     CFE_SB_MsgId_t mid)
{
    (void)msg; (void)mid;
}

static inline void CFE_MSG_GetMsgId(const void *msg,
                                     CFE_SB_MsgId_t *mid)
{
    (void)msg;
    mid->Value = 0;
}

static inline void CFE_MSG_GetSize(const void *msg,
                                    CFE_MSG_Size_t *size)
{
    (void)msg;
    *size = 0;
}

static inline void CFE_MSG_GetFcnCode(const void *msg,
                                       CFE_MSG_FcnCode_t *fc)
{
    (void)msg;
    *fc = 0;
}

static inline void CFE_MSG_SetFcnCode(void *msg, CFE_MSG_FcnCode_t fc)
{
    (void)msg; (void)fc;
}

static inline CFE_Status_t CFE_MSG_GenerateChecksum(void *msg)
{
    (void)msg;
    return 0;
}

/* ── cFE TIME stubs ────────────────────────────────────────────────── */

typedef struct {
    uint32_t Seconds;
    uint32_t Subseconds;
} CFE_TIME_SysTime_t;

static inline CFE_TIME_SysTime_t CFE_TIME_GetTime(void)
{
    CFE_TIME_SysTime_t t = {0, 0};
    return t;
}

static inline void CFE_MSG_SetMsgTime(void *msg,
                                       CFE_TIME_SysTime_t time)
{
    (void)msg; (void)time;
}

/* ── cFE TBL (Table) stubs ─────────────────────────────────────────── */

typedef uint32_t CFE_TBL_Handle_t;

static inline CFE_Status_t CFE_TBL_Register(CFE_TBL_Handle_t *handle,
                                             const char *name,
                                             size_t size,
                                             uint16_t opts,
                                             void *callback)
{
    (void)handle; (void)name; (void)size; (void)opts; (void)callback;
    return CFE_SUCCESS;
}

static inline CFE_Status_t CFE_TBL_Load(CFE_TBL_Handle_t handle,
                                          uint16_t src,
                                          const void *data)
{
    (void)handle; (void)src; (void)data;
    return CFE_SUCCESS;
}

static inline CFE_Status_t CFE_TBL_GetAddress(void **addr,
                                                CFE_TBL_Handle_t handle)
{
    (void)addr; (void)handle;
    return CFE_SUCCESS;
}

static inline CFE_Status_t CFE_TBL_ReleaseAddress(CFE_TBL_Handle_t handle)
{
    (void)handle;
    return CFE_SUCCESS;
}

#define CFE_TBL_OPT_DEFAULT     0
#define CFE_TBL_SRC_ADDRESS     0
#define CFE_TBL_SRC_FILE        1
#define CFE_TBL_INFO_UPDATED    ((CFE_Status_t)2)

#endif /* CFE_H */
