/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_hal_client.c
 * @brief LRMH HAL UDP client — links into the cFS image
 *
 * This file implements the LRMH_HAL_* functions declared in
 * hal/include/lrmh_hal.h by sending UDP requests to a HAL server
 * (sim, bench, or flight) and decoding the responses.
 *
 * Per ADR-0001, this is the only thing that changes between
 * environments. The mission apps see only LRMH_HAL_* and never
 * know there is a UDP transport underneath. The UDP server they
 * talk to is:
 *
 *   sim     hal/sim/lrmh_hal_sim       (standalone executable)
 *   bench   hal/bench/lrmh_hal_bench   (future)
 *   flight  hal/flight/lrmh_hal_flight (future)
 *
 * Four UDP-HAL pitfalls are addressed:
 *
 *   1. Bounded recv with deadline — caller never blocks
 *      indefinitely; LRMH_HAL_BACKEND_TIMEOUT on slow server.
 *   2. Stale response detection via per-request sequence numbers
 *      — stale responses are DRAINED (recv loops until matching
 *      seq_no or deadline expires), not bailed on.
 *   3. Per-task socket isolation via the GCC __thread storage
 *      class (equivalent to C11 _Thread_local; chosen for cFS
 *      build compatibility — the cFS workspace builds C99 with
 *      -Wpedantic -Werror). Each cFS app's task (a pthread under
 *      OSAL-Linux) gets its own socket fd and seq_no array, so
 *      concurrent callers don't contend or cross-talk.
 *      lrmh_sense at 100 Hz and lrmh_ctrl at 10 Hz both run
 *      cleanly.
 *   4. Sim dispatcher does not block physics tick — the sim
 *      server runs its physics on its own clock, not on request
 *      arrival rate, so a slow client does not slow down the sim.
 */

#include "lrmh_hal.h"
#include "cfe.h"
#include "lrmh_platform_cfg.h"   /* LRMH_MAX_STROKE_M */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>

/* ============================================================
 * Configuration — must match hal/sim/lrmh_hal_sim.c
 * ============================================================ */
#define LRMH_HAL_SERVER_HOST   "127.0.0.1"
#define LRMH_HAL_SERVER_PORT   17011
#define LRMH_HAL_REQ_TIMEOUT_MS  100   /* per-request timeout */
#define LRMH_HAL_MAGIC         0x4C524D48u

/* ============================================================
 * UDP envelope — must match hal/sim/lrmh_hal_sim.c
 * ============================================================ */
typedef enum {
    HAL_REQ_INIT             = 0,
    HAL_REQ_READ_SENSORS     = 1,
    HAL_REQ_GET_HEALTH       = 2,
    HAL_REQ_CMD_POSITION     = 3,
    HAL_REQ_CMD_FORCE        = 4,
    HAL_REQ_RETRACT          = 5,
    HAL_REQ_HOLD             = 6,
    HAL_REQ_ESTOP            = 7,
    HAL_REQ_STEP_SIM         = 8
} HalReqType_t;

typedef struct {
    uint32_t  magic;
    uint32_t  seq_no;
    uint32_t  req_type;
    uint32_t  payload_len;
    uint8_t   payload[256];
} HalRequest_t;

typedef struct {
    uint32_t  magic;
    uint32_t  seq_no;
    uint32_t  req_type;
    int32_t   status;
    uint32_t  payload_len;
    uint8_t   payload[256];
} HalResponse_t;

/* ============================================================
 * Per-task socket state (pitfall 3)
 *
 * Each cFS app's task is a separate pthread under OSAL-Linux, so
 * GCC __thread gives us per-task isolation: each task's first
 * call to client_connect creates ITS socket, and the seq_no
 * array is private to that task. No cross-task contention on
 * send/recv pairing.
 *
 * Default zero-init is safe for everything except `fd` — we
 * gate use of `fd` on the explicit `initialized` flag rather
 * than a sentinel value.
 * ============================================================ */
static __thread bool                g_tls_initialized = false;
static __thread int                 g_tls_fd          = -1;
static __thread struct sockaddr_in  g_tls_server;
static __thread uint32_t            g_tls_seq_no_per_type[16];

/* ============================================================
 * Internal: open the per-task socket on first call
 * ============================================================ */
static int32 client_connect(void) {
    if (g_tls_initialized) return LRMH_HAL_OK;

    g_tls_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_tls_fd < 0) return LRMH_HAL_ERROR;

    g_tls_server.sin_family = AF_INET;
    g_tls_server.sin_port   = htons(LRMH_HAL_SERVER_PORT);
    if (inet_pton(AF_INET, LRMH_HAL_SERVER_HOST,
                  &g_tls_server.sin_addr) != 1) {
        close(g_tls_fd);
        g_tls_fd = -1;
        return LRMH_HAL_ERROR;
    }

    g_tls_initialized = true;
    return LRMH_HAL_OK;
}

/* ============================================================
 * Internal: monotonic-millisecond reading for the recv deadline
 * ============================================================ */
static int64_t monotonic_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000L;
}

/* ============================================================
 * Internal: send a request and drain until we get the matching
 * response or hit the deadline.
 *
 * `expected_resp_payload_len`:
 *   0           — no response payload expected
 *   nonzero     — response.payload_len must equal this exactly,
 *                 and resp_payload (size = expected_resp_payload_len)
 *                 receives the bytes
 *
 * Pitfalls 1, 2, 4 are addressed here:
 *   - bounded recv via deadline-clamped poll loop
 *   - stale responses (wrong seq_no / req_type / magic) drained
 *     by continuing the recv loop instead of returning early
 *   - response payload_len validated against caller's expected
 *     size — short or long payloads are rejected, not silently
 *     under-filled
 * ============================================================ */
static int32 client_request(HalReqType_t type,
                            const void *req_payload, size_t req_payload_len,
                            void *resp_payload,
                            size_t expected_resp_payload_len) {
    int32 r = client_connect();
    if (r != LRMH_HAL_OK) return r;

    HalRequest_t req;
    memset(&req, 0, sizeof(req));
    req.magic       = LRMH_HAL_MAGIC;
    req.req_type    = (uint32_t)type;
    req.seq_no      = ++g_tls_seq_no_per_type[type & 0xF];
    req.payload_len = (uint32_t)req_payload_len;
    if (req_payload_len > 0 && req_payload_len <= sizeof(req.payload)) {
        memcpy(req.payload, req_payload, req_payload_len);
    }

    if (sendto(g_tls_fd, &req, sizeof(req), 0,
               (struct sockaddr *)&g_tls_server,
               sizeof(g_tls_server)) < 0) {
        /* Transport-level send failure — not a server-slow timeout */
        return LRMH_HAL_ERROR;
    }

    /* Bounded recv loop. On every iteration: recompute remaining
     * time against the deadline, poll for that long, and drop
     * any response that doesn't match. */
    int64_t deadline_ms = monotonic_ms() + LRMH_HAL_REQ_TIMEOUT_MS;
    HalResponse_t resp;

    for (;;) {
        int64_t remaining = deadline_ms - monotonic_ms();
        if (remaining <= 0) return LRMH_HAL_BACKEND_TIMEOUT;

        struct pollfd pfd = { .fd = g_tls_fd, .events = POLLIN };
        int pr = poll(&pfd, 1, (int)remaining);
        if (pr <= 0) return LRMH_HAL_BACKEND_TIMEOUT;

        ssize_t n = recv(g_tls_fd, &resp, sizeof(resp), 0);
        if (n < (ssize_t)(sizeof(resp) - sizeof(resp.payload))) continue; /* runt */
        if (resp.magic    != LRMH_HAL_MAGIC) continue;     /* foreign */
        if (resp.req_type != req.req_type)   continue;     /* wrong type */
        if (resp.seq_no   != req.seq_no)     continue;     /* stale seq */
        break;  /* matched — exit loop */
    }

    /* Validate payload length matches expectation */
    if (expected_resp_payload_len != resp.payload_len) {
        return LRMH_HAL_ERROR;
    }
    if (expected_resp_payload_len > 0 && resp_payload != NULL) {
        memcpy(resp_payload, resp.payload, expected_resp_payload_len);
    }

    return resp.status;
}

/* ============================================================
 * Public API — implementing hal/include/lrmh_hal.h
 * ============================================================ */

int32 LRMH_HAL_Init(void) {
    return client_request(HAL_REQ_INIT, NULL, 0, NULL, 0);
}

int32 LRMH_HAL_ReadSensors(LRMH_HAL_SensorData_t *out) {
    if (!out) return LRMH_HAL_REJECTED_BAD_PARAM;
    return client_request(HAL_REQ_READ_SENSORS, NULL, 0,
                          out, sizeof(*out));
}

int32 LRMH_HAL_GetHealth(LRMH_HAL_Health_t *out) {
    if (!out) return LRMH_HAL_REJECTED_BAD_PARAM;
    return client_request(HAL_REQ_GET_HEALTH, NULL, 0,
                          out, sizeof(*out));
}

int32 LRMH_HAL_CommandPosition(const LRMH_HAL_PositionCmd_t *cmd) {
    if (!cmd) return LRMH_HAL_REJECTED_BAD_PARAM;
    if (cmd->target_stroke_m < 0.0f ||
        cmd->target_stroke_m > LRMH_MAX_STROKE_M) {
        return LRMH_HAL_REJECTED_BAD_PARAM;
    }
    if (cmd->max_force_n   <= 0.0f) return LRMH_HAL_REJECTED_BAD_PARAM;
    if (cmd->max_speed_mps <= 0.0f) return LRMH_HAL_REJECTED_BAD_PARAM;
    return client_request(HAL_REQ_CMD_POSITION, cmd, sizeof(*cmd),
                          NULL, 0);
}

int32 LRMH_HAL_CommandForce(const LRMH_HAL_ForceCmd_t *cmd) {
    if (!cmd) return LRMH_HAL_REJECTED_BAD_PARAM;
    if (cmd->target_force_n < 0.0f) return LRMH_HAL_REJECTED_BAD_PARAM;
    if (cmd->max_stroke_m   <= 0.0f) return LRMH_HAL_REJECTED_BAD_PARAM;
    if (cmd->ramp_rate_nps  <  0.0f) return LRMH_HAL_REJECTED_BAD_PARAM;
    return client_request(HAL_REQ_CMD_FORCE, cmd, sizeof(*cmd),
                          NULL, 0);
}

int32 LRMH_HAL_Retract(void) {
    return client_request(HAL_REQ_RETRACT, NULL, 0, NULL, 0);
}

int32 LRMH_HAL_Hold(void) {
    return client_request(HAL_REQ_HOLD, NULL, 0, NULL, 0);
}

int32 LRMH_HAL_EStop(void) {
    return client_request(HAL_REQ_ESTOP, NULL, 0, NULL, 0);
}

int32 LRMH_HAL_StepSim(double DeltaTime_s) {
    return client_request(HAL_REQ_STEP_SIM, &DeltaTime_s, sizeof(DeltaTime_s),
                          NULL, 0);
}
