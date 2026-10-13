/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_ctrl_app.h
 * @brief LRMH lrmh_ctrl application — public interface
 *
 * Per ADR-0006: actuator command translator. Receives high-level
 * motion commands from lrmh_seq (token-validated per ADR-0007),
 * translates them into LRMH_HAL_* calls per the ADR-0010
 * contract, and publishes status TLM with completion flag.
 *
 * **First 2.0 receiver of token-validated commands.** This is
 * where ADR-0007's six validation rules first run end-to-end
 * against real cross-app commands.
 */

#ifndef LRMH_CTRL_APP_H
#define LRMH_CTRL_APP_H

#include "cfe.h"
#include "lrmh_msgids.h"
#include "lrmh_msg.h"
#include "lrmh_seq_token.h"
#include "lrmh_hal.h"
#include "lrmh_platform_cfg.h"

#include "lrmh_ctrl_msg.h"
#include "lrmh_ctrl_events.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * App data
 * ============================================================ */
typedef struct {
    CFE_SB_PipeId_t  CmdPipe;

    /* Active state */
    uint8   ActiveMode;
    uint8   CommandPending;
    float   LastCommandedPos_m;
    float   LastCommandedForce_N;

    /* Counters */
    uint32  CmdCount;
    uint32  ErrCount;
    uint32  StaleCount;        /* tokens rejected by validation */
    uint32  HalErrCount;       /* HAL calls that returned error */

    /* Sequence-token receiver state (ADR-0007) */
    LRMH_SeqToken_t AcceptedToken;
    bool            ResetObserved;  /* set true on LRMH_TOKEN_RESET_MID */
} LRMH_CTRL_AppData_t;

/* ============================================================
 * Public entry point
 * ============================================================ */
void LRMH_CTRL_AppMain(void);

/* ============================================================
 * Test-exposed internal API
 * ============================================================ */
extern LRMH_CTRL_AppData_t g_CtrlApp;

void LRMH_CTRL_HandleCommand(const LRMH_CtrlCmd_t *cmd, bool token_validated);
LRMH_TokenValidationResult_t LRMH_CTRL_ValidateAndDispatch(
    const LRMH_CtrlCmd_t *cmd);
void LRMH_CTRL_OnTokenReset(void);
void LRMH_CTRL_PublishStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* LRMH_CTRL_APP_H */
