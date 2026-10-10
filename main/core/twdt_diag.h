/*
 * twdt_diag.h - decisive TWDT / IDLE-starvation instrumentation.
 *
 * Two hooks, both cheap and rate limited:
 *   1. a 500ms esp_timer that samples uxTaskGetSystemState() and prints per-task
 *      run-time deltas (needs CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y),
 *   2. a vprintf wrapper that reports log holds > 100ms and log floods > 500/s,
 *      which is what a log-lock wedge would look like from the inside.
 *
 * All output is tagged "TWDIAG" so it can be grepped out of a capture.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Idempotent. Call early in app_main (after settings/log init, before the
 * AP manager) so boot and softAP bring-up are covered. */
void twdt_diag_init(void);

#ifdef __cplusplus
}
#endif
