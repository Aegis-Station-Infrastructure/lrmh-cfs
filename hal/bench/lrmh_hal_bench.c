/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file lrmh_hal_bench.c
 * @brief LRMH HAL bench backend — bench-hardware integration template
 *
 * Standalone UDP server that the cFS image's hal/client/ talks to
 * when running against bench hardware (the actuator + sensor
 * stack on the engineering test fixture).
 *
 * **Status:** scaffold. The UDP envelope, dispatch loop, and
 * request-type case structure are complete and identical to
 * hal/sim/lrmh_hal_sim.c so the cFS image side does not change
 * across sim/bench. Each request handler is a stub that returns
 * either zeroed-but-valid data (sensors) or LRMH_HAL_OK (commands),
 * with a `TODO(bench):` marker on every line that needs the real
 * controller call wired in.
 *
 * Drop-in plan for a bench integration:
 *   1. Replace each TODO(bench) with the matching call into the
 *      bench controller / sensor library.
 *   2. The UDP envelope (HalRequest_t / HalResponse_t / magic) is
 *      part of the contract; do not change it without coordinated
 *      changes in hal/client/lrmh_hal_client.c.
 *
 * Per ADR-0001, this file is the ONLY thing that changes between
 * sim and bench builds. Mission apps under apps/lrmh_<name>/ link
 * against hal/client/ and never know there is a UDP transport.
 *
 * Per ADR-0003, the backend exposes position/force setpoints, not
 * raw motor duty cycles. The bench controller is expected to close
 * its own inner loop on PWM / current.
 *
 * Per ADR-0009, every read returns a freshness mask
 * (`channel_valid`) and a monotonic timestamp.
 *
 * Per ADR-0010, the actuator HAL surface enforces a per-command
 * state machine — ESTOP rejects subsequent setpoints until cleared
 * by an Init or Retract; Retract and Hold are unconditional.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "lrmh_hal.h"

/* ============================================================
 * Configuration
 * ============================================================ */
#define LRMH_HAL_BENCH_PORT     17011    /* must match hal/client/ */

/* ============================================================
 * UDP envelope — IDENTICAL to hal/sim/lrmh_hal_sim.c
 *
 * The cFS-side client does not branch on backend type. Sim and
 * bench differ only in what populates the response payload.
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
    HAL_REQ_STEP_SIM         = 8,  /* sim-only; bench acks OK as no-op */
    HAL_REQ_INJECT_FAULT     = 9   /* sim-only; bench acks OK as no-op.
                                    * Bench-side fault stimulus needs a
                                    * controller-specific hook (see
                                    * ADR-0017); the HAL Porting Guide
                                    * covers that. */
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

#define LRMH_HAL_MAGIC  0x4C524D48u

/* ============================================================
 * Bench backend state
 *
 * Extend this struct with whatever long-lived handles
 * your controller library needs (CAN socket, SPI fd, calibration
 * vectors, etc.).
 * ============================================================ */
static struct {
    int       udp_fd;
    bool      running;
    bool      initialized;
    bool      estop_latched;
    /* TODO(bench): controller_handle_t ctrl;
     * TODO(bench): sensor_bus_handle_t sensors;
     */
} g_bench;

/* ============================================================
 * Time helper
 * ============================================================ */
static uint64_t monotonic_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}

/* ============================================================
 * Per-request handlers — each is one TODO(bench) away from real
 * ============================================================ */

/* HAL_REQ_INIT — bench bring-up. Mission apps call this once. */
static int32 hal_init(void)
{
    g_bench.estop_latched = false;
    g_bench.initialized   = true;
    /* TODO(bench): power-on the actuator controller.
     * TODO(bench): zero the load cell + depth encoder.
     * TODO(bench): run a 1 s self-test sweep; return
     *                 LRMH_HAL_ERROR if any channel fails. */
    fprintf(stderr, "[bench] INIT — controller bring-up stub\n");
    return LRMH_HAL_OK;
}

/* HAL_REQ_READ_SENSORS — called at 100 Hz from lrmh_sense.
 *
 * Per ADR-0009: must populate `channel_valid` (bit per channel,
 * set when fresh) and `time_us` (monotonic at acquisition).
 *
 * Channel order in LRMH_HAL_SensorData_t:
 *   force_n, depth_m, stroke_m, current_a, bus_v, temp_c
 * with one channel_valid bit each per LRMH_HAL_CH_* indexes.
 */
static int32 hal_read_sensors(LRMH_HAL_SensorData_t *out)
{
    if (!out) return LRMH_HAL_REJECTED_BAD_PARAM;
    if (!g_bench.initialized) return LRMH_HAL_NOT_INITIALIZED;

    memset(out, 0, sizeof(*out));
    out->valid         = true;
    out->time_us       = monotonic_us();
    /* Stub: leave numeric channels at 0 with all freshness bits set
     * so lrmh_fault's freshness check doesn't immediately trigger
     * SENSOR_TIMEOUT during dry-run. */
    out->channel_valid = LRMH_HAL_CH_ALL;
    /* TODO(bench): read each channel from your sensor bus, apply
     * calibration, store in the corresponding field. Clear the
     * matching bit in channel_valid on a per-channel read failure.
     *
     *   out->force_n   = calibrated_force_n();    // bit LRMH_HAL_CH_FORCE
     *   out->depth_m   = calibrated_depth_m();    // bit LRMH_HAL_CH_DEPTH
     *   out->stroke_m  = encoder_stroke_m();      // bit LRMH_HAL_CH_STROKE
     *   out->current_a = motor_current_a();       // bit LRMH_HAL_CH_CURRENT
     *   out->bus_v     = bus_voltage_v();         // bit LRMH_HAL_CH_BUS_V
     *   out->temp_c    = actuator_temp_c();       // bit LRMH_HAL_CH_TEMP
     */
    return LRMH_HAL_OK;
}

/* HAL_REQ_GET_HEALTH — controller state + flag word + last setpoint. */
static int32 hal_get_health(LRMH_HAL_Health_t *out)
{
    if (!out) return LRMH_HAL_REJECTED_BAD_PARAM;
    if (!g_bench.initialized) return LRMH_HAL_NOT_INITIALIZED;

    memset(out, 0, sizeof(*out));
    out->valid   = true;
    out->time_us = monotonic_us();
    out->state   = g_bench.estop_latched
                   ? LRMH_HAL_STATE_ESTOP
                   : LRMH_HAL_STATE_IDLE;
    /* TODO(bench): poll controller status flags (overcurrent,
     * encoder loss, thermal trip, watchdog) and OR into out->flags.
     * TODO(bench): out->stroke_m      = current stroke encoder reading
     * TODO(bench): out->last_force_n  = last commanded force */
    return LRMH_HAL_OK;
}

/* HAL_REQ_CMD_POSITION — drive to target stroke at the speed limit.
 *
 * Returns immediately; mission apps poll READ_SENSORS to see
 * progress. The controller is expected to honor max_speed_mps and
 * stop on the first of {target reached, max_force exceeded}.
 */
static int32 hal_cmd_position(const LRMH_HAL_PositionCmd_t *cmd)
{
    if (!cmd) return LRMH_HAL_REJECTED_BAD_PARAM;
    if (!g_bench.initialized) return LRMH_HAL_NOT_INITIALIZED;
    if (g_bench.estop_latched) return LRMH_HAL_REJECTED_ESTOP;
    /* TODO(bench): controller_position_setpoint(
     *     cmd->target_stroke_m, cmd->max_speed_mps, cmd->max_force_n); */
    fprintf(stderr,
            "[bench] CMD_POSITION target=%.4f m  speed_max=%.4f m/s  "
            "force_max=%.2f N\n",
            (double)cmd->target_stroke_m,
            (double)cmd->max_speed_mps,
            (double)cmd->max_force_n);
    return LRMH_HAL_OK;
}

/* HAL_REQ_CMD_FORCE — drive at controlled force ramp. */
static int32 hal_cmd_force(const LRMH_HAL_ForceCmd_t *cmd)
{
    if (!cmd) return LRMH_HAL_REJECTED_BAD_PARAM;
    if (!g_bench.initialized) return LRMH_HAL_NOT_INITIALIZED;
    if (g_bench.estop_latched) return LRMH_HAL_REJECTED_ESTOP;
    /* TODO(bench): controller_force_setpoint(
     *     cmd->target_force_n, cmd->ramp_rate_nps, cmd->max_stroke_m); */
    fprintf(stderr,
            "[bench] CMD_FORCE target=%.2f N  ramp=%.2f N/s  "
            "stroke_max=%.4f m\n",
            (double)cmd->target_force_n,
            (double)cmd->ramp_rate_nps,
            (double)cmd->max_stroke_m);
    return LRMH_HAL_OK;
}

/* HAL_REQ_RETRACT — unconditional retract to home. Allowed under
 * ESTOP per ADR-0010 (it's the safing primitive); clears the latch. */
static int32 hal_retract(void)
{
    if (!g_bench.initialized) return LRMH_HAL_NOT_INITIALIZED;
    /* TODO(bench): controller_retract_to_home(); */
    fprintf(stderr, "[bench] RETRACT\n");
    g_bench.estop_latched = false;
    return LRMH_HAL_OK;
}

/* HAL_REQ_HOLD — stop motion at current position, hold position. */
static int32 hal_hold(void)
{
    if (!g_bench.initialized) return LRMH_HAL_NOT_INITIALIZED;
    /* TODO(bench): controller_hold_position(); */
    fprintf(stderr, "[bench] HOLD\n");
    return LRMH_HAL_OK;
}

/* HAL_REQ_ESTOP — immediately drop motor enable; latch until
 * Retract or Init clears (ADR-0010). */
static int32 hal_estop(void)
{
    if (!g_bench.initialized) return LRMH_HAL_NOT_INITIALIZED;
    /* TODO(bench): controller_estop_now();   // drops motor enable */
    g_bench.estop_latched = true;
    fprintf(stderr, "[bench] ESTOP — latched\n");
    return LRMH_HAL_OK;
}

/* HAL_REQ_STEP_SIM — sim-only. Bench acks OK as a no-op so the
 * cFS-side client doesn't need to branch on backend type. */
static int32 hal_step_sim(void)
{
    return LRMH_HAL_OK;
}

/* HAL_REQ_INJECT_FAULT — sim-only. Bench acks OK as a no-op.
 * Real bench-side fault stimulus is hardware-dependent (apply a
 * load above the LC threshold to the load cell, drive overcurrent
 * via the controller's test hook, etc.) and lives outside the
 * UDP envelope; see ADR-0017 §Test and the bench-integration
 * HAL Porting Guide for wiring a controller-side
 * inject_fault() into their integration loop. */
static int32 hal_inject_fault(void)
{
    fprintf(stderr, "[bench] INJECT_FAULT (no-op on bench — "
                    "use the controller-side hook; see HAL Porting Guide)\n");
    return LRMH_HAL_OK;
}

/* ============================================================
 * UDP dispatch loop
 *
 * Same shape as hal/sim/lrmh_hal_sim.c. Handles one request at a
 * time; out-of-order responses can't happen because the cFS-side
 * client blocks on recvfrom (see hal/client/lrmh_hal_client.c).
 * ============================================================ */
static void handle_request(const HalRequest_t *req,
                           HalResponse_t *resp)
{
    memset(resp, 0, sizeof(*resp));
    resp->magic    = LRMH_HAL_MAGIC;
    resp->seq_no   = req->seq_no;
    resp->req_type = req->req_type;
    resp->status   = LRMH_HAL_ERROR;

    switch ((HalReqType_t)req->req_type) {
    case HAL_REQ_INIT:
        resp->status = hal_init();
        break;
    case HAL_REQ_READ_SENSORS: {
        LRMH_HAL_SensorData_t s;
        resp->status = hal_read_sensors(&s);
        if (resp->status == LRMH_HAL_OK) {
            memcpy(resp->payload, &s, sizeof(s));
            resp->payload_len = sizeof(s);
        }
        break;
    }
    case HAL_REQ_GET_HEALTH: {
        LRMH_HAL_Health_t h;
        resp->status = hal_get_health(&h);
        if (resp->status == LRMH_HAL_OK) {
            memcpy(resp->payload, &h, sizeof(h));
            resp->payload_len = sizeof(h);
        }
        break;
    }
    case HAL_REQ_CMD_POSITION: {
        LRMH_HAL_PositionCmd_t c;
        if (req->payload_len < sizeof(c)) {
            resp->status = LRMH_HAL_REJECTED_BAD_PARAM;
            break;
        }
        memcpy(&c, req->payload, sizeof(c));
        resp->status = hal_cmd_position(&c);
        break;
    }
    case HAL_REQ_CMD_FORCE: {
        LRMH_HAL_ForceCmd_t c;
        if (req->payload_len < sizeof(c)) {
            resp->status = LRMH_HAL_REJECTED_BAD_PARAM;
            break;
        }
        memcpy(&c, req->payload, sizeof(c));
        resp->status = hal_cmd_force(&c);
        break;
    }
    case HAL_REQ_RETRACT:
        resp->status = hal_retract();
        break;
    case HAL_REQ_HOLD:
        resp->status = hal_hold();
        break;
    case HAL_REQ_ESTOP:
        resp->status = hal_estop();
        break;
    case HAL_REQ_STEP_SIM:
        resp->status = hal_step_sim();
        break;
    case HAL_REQ_INJECT_FAULT:
        resp->status = hal_inject_fault();
        break;
    default:
        fprintf(stderr, "[bench] unknown req_type=%u seq=%u\n",
                req->req_type, req->seq_no);
        resp->status = LRMH_HAL_REJECTED_BAD_PARAM;
        break;
    }
}

static void on_signal(int sig)
{
    (void)sig;
    g_bench.running = false;
}

int main(int argc, char **argv)
{
    (void)argc; (void)argv;

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    g_bench.udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_bench.udp_fd < 0) {
        perror("socket");
        return 1;
    }
    int one = 1;
    setsockopt(g_bench.udp_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(LRMH_HAL_BENCH_PORT);
    if (bind(g_bench.udp_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return 1;
    }

    fprintf(stderr,
            "lrmh_hal_bench: scaffold backend on UDP :%d\n"
            "  TODO(bench): replace per-request stubs with real\n"
            "  bench controller / sensor calls. See file header.\n",
            LRMH_HAL_BENCH_PORT);

    g_bench.running = true;
    while (g_bench.running) {
        HalRequest_t  req;
        HalResponse_t resp;
        struct sockaddr_in peer;
        socklen_t plen = sizeof(peer);

        ssize_t n = recvfrom(g_bench.udp_fd, &req, sizeof(req), 0,
                             (struct sockaddr *)&peer, &plen);
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("recvfrom");
            break;
        }
        if ((size_t)n < offsetof(HalRequest_t, payload) ||
            req.magic != LRMH_HAL_MAGIC) {
            continue;  /* drop malformed */
        }
        handle_request(&req, &resp);
        sendto(g_bench.udp_fd, &resp,
               offsetof(HalResponse_t, payload) + resp.payload_len,
               0, (struct sockaddr *)&peer, plen);
    }

    close(g_bench.udp_fd);
    fprintf(stderr, "lrmh_hal_bench: shutdown\n");
    return 0;
}
