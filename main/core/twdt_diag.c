/*
 * twdt_diag.c - decisive TWDT / IDLE-starvation instrumentation.
 *
 * Discriminator output (tag "TWDIAG"):
 *   - run-time-stats line every 500ms: wall time vs sum of per-task CPU deltas.
 *       sum ~ 2*wall with IDLE~0            -> a task (or both cores) busy
 *       sum ~ wall   with idle~0, isr2~ big -> ISR context eating a core
 *       idle ~ wall                          -> IDLE actually ran (TWDT hook?)
 *   - "log hold N ms"  -> the logger itself held its lock that long
 *   - "log rate N lines/s" -> a log flood (each line is UART-blocking time)
 *
 * No malloc anywhere in the callbacks; snapshots live in static buffers.
 */
#include "core/twdt_diag.h"

#include "sdkconfig.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TD_TAG "TWDIAG"

/* ------------------------------------------------------------------ */
/* Patch 3: log-hold / log-rate tracker (vprintf wrapper).            */
/* ------------------------------------------------------------------ */

static vprintf_like_t s_orig_vprintf;
static bool s_wrap_installed;
static volatile bool s_in_wrapper;
static int64_t s_hold_last_warn_us;
static int64_t s_rate_window_us;
static uint32_t s_rate_window_count;

static int td_vprintf(const char *fmt, va_list args)
{
    /* Re-entrant call (our own reporting below): measure nothing, just pass
     * through so a hold report can never recurse forever. */
    if (s_in_wrapper || s_orig_vprintf == NULL) {
        return s_orig_vprintf ? s_orig_vprintf(fmt, args) : vprintf(fmt, args);
    }

    s_in_wrapper = true;
    const int64_t t0 = esp_timer_get_time();
    const int ret = s_orig_vprintf(fmt, args);
    const int64_t held_us = esp_timer_get_time() - t0;

    /* Rate accounting: one fixed 1s window, no allocation. */
    uint32_t rate_exceeded = 0;
    if (s_rate_window_us == 0) {
        s_rate_window_us = t0;
    }
    s_rate_window_count++;
    if ((t0 - s_rate_window_us) >= 1000000) {
        if (s_rate_window_count > 500) {
            rate_exceeded = s_rate_window_count;
        }
        s_rate_window_count = 0;
        s_rate_window_us = t0;
    }

    bool warn_hold = false;
    if (held_us > 100000 && (t0 - s_hold_last_warn_us) >= 1000000) {
        s_hold_last_warn_us = t0;
        warn_hold = true;
    }

    if (warn_hold) {
        ESP_LOGW(TD_TAG, "log hold %lld ms", (long long)(held_us / 1000));
    }
    if (rate_exceeded) {
        ESP_LOGW(TD_TAG, "log rate %u lines/s", (unsigned)rate_exceeded);
    }

    s_in_wrapper = false;
    return ret;
}

static void td_install_log_tracker(void)
{
    if (s_wrap_installed) {
        return;
    }
    /* esp_log_set_vprintf() returns the previous handler, so we chain instead
     * of clobbering whatever was already installed. */
    s_orig_vprintf = esp_log_set_vprintf(td_vprintf);
    s_wrap_installed = true;
}

/* ------------------------------------------------------------------ */
/* Patch 2: 500ms run-time-stats sampler.                             */
/* ------------------------------------------------------------------ */

#if defined(CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS)

#define TD_MAX_TASKS   24
#define TD_PERIOD_US   500000
#define TD_MIN_WALL_US 100000

static esp_timer_handle_t s_timer;
static TaskStatus_t s_prev[TD_MAX_TASKS];
static TaskStatus_t s_cur[TD_MAX_TASKS];   /* static: esp_timer task stack is 2KB */
static UBaseType_t s_prev_n;
static int64_t s_prev_us;
static bool s_have_prev;
static uint32_t s_seq;

/* Delta of one current task against the previous snapshot. Modular
 * subtraction keeps this correct even if the counter type is 32-bit and
 * wrapped in between. Tasks not in the previous snapshot (newly created)
 * contribute their full counter value so they are never lost from the sum. */
static uint64_t td_delta(const TaskStatus_t *cur)
{
    for (UBaseType_t i = 0; i < s_prev_n; i++) {
        if (s_prev[i].xHandle == cur->xHandle) {
            configRUN_TIME_COUNTER_TYPE d =
                (configRUN_TIME_COUNTER_TYPE)(cur->ulRunTimeCounter - s_prev[i].ulRunTimeCounter);
            return (uint64_t)d;
        }
    }
    return (uint64_t)cur->ulRunTimeCounter;
}

static uint64_t td_delta_by_name(const TaskStatus_t *cur, UBaseType_t n, const char *name)
{
    uint64_t acc = 0;
    for (UBaseType_t i = 0; i < n; i++) {
        if (cur[i].pcTaskName != NULL && strcmp(cur[i].pcTaskName, name) == 0) {
            acc += td_delta(&cur[i]);
        }
    }
    return acc;
}

static void td_snapshot(const TaskStatus_t *cur, UBaseType_t n, int64_t now)
{
    if (n > TD_MAX_TASKS) {
        n = TD_MAX_TASKS;
    }
    memcpy(s_prev, cur, (size_t)n * sizeof(TaskStatus_t));
    s_prev_n = n;
    s_prev_us = now;
    s_have_prev = true;
}

static void td_timer_cb(void *arg)
{
    (void)arg;

    configRUN_TIME_COUNTER_TYPE total = 0;
    UBaseType_t n = uxTaskGetSystemState(s_cur, TD_MAX_TASKS, &total);
    if (n > TD_MAX_TASKS) {
        n = TD_MAX_TASKS;
    }
    const int64_t now = esp_timer_get_time();

    if (!s_have_prev) {
        /* Seed only - a first line would compare "everything since boot"
         * against one wall interval and be pure noise. */
        td_snapshot(s_cur, n, now);
        return;
    }

    const int64_t wall_us = now - s_prev_us;

    uint64_t sum = 0;
    for (UBaseType_t i = 0; i < n; i++) {
        sum += td_delta(&s_cur[i]);
    }

    /* Stash the interesting deltas before the snapshot is overwritten. */
    const uint64_t d_idle    = td_delta_by_name(s_cur, n, "IDLE");
    const uint64_t d_idle1   = td_delta_by_name(s_cur, n, "IDLE1");
    const uint64_t d_wifi    = td_delta_by_name(s_cur, n, "wifi");
    const uint64_t d_tcpip   = td_delta_by_name(s_cur, n, "tcpip");
    const uint64_t d_main    = td_delta_by_name(s_cur, n, "main");
    const uint64_t d_mdns    = td_delta_by_name(s_cur, n, "mdns");
    const uint64_t d_esp_tim = td_delta_by_name(s_cur, n, "esp_timer");

    td_snapshot(s_cur, n, now);

    if (wall_us < TD_MIN_WALL_US) {
        return;   /* callbacks fired back to back - not a usable interval */
    }

    const uint64_t wall = (uint64_t)wall_us;
    /* Run-time counters accumulate across BOTH cores, so a single-core wall
     * under-estimates the budget; isr2~ uses 2*wall (dual core). */
    const uint64_t wall2 = wall * 2u;
    const uint64_t isr  = (wall > sum) ? (wall - sum) : 0u;
    const uint64_t isr2 = (wall2 > sum) ? (wall2 - sum) : 0u;

    s_seq++;
    ESP_LOGW(TD_TAG,
             "wall=%llu sum=%llu idle=%llu wifi=%llu tcpip=%llu main=%llu mdns=%llu "
             "esp_timer=%llu isr~=%llu seq=%u idle1=%llu isr2~=%llu",
             (unsigned long long)wall, (unsigned long long)sum,
             (unsigned long long)d_idle, (unsigned long long)d_wifi,
             (unsigned long long)d_tcpip, (unsigned long long)d_main,
             (unsigned long long)d_mdns, (unsigned long long)d_esp_tim,
             (unsigned long long)isr, (unsigned)s_seq,
             (unsigned long long)d_idle1, (unsigned long long)isr2);
}

static void td_start_sampler(void)
{
    if (s_timer) {
        return;
    }
    const esp_timer_create_args_t args = {
        .callback = td_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "twdt_diag",
    };
    if (esp_timer_create(&args, &s_timer) != ESP_OK) {
        ESP_LOGW(TD_TAG, "failed to create sampler timer");
        return;
    }
    esp_timer_start_periodic(s_timer, TD_PERIOD_US);
}

#else /* !CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS */

static void td_start_sampler(void)
{
    /* Other boards ship without run-time stats; stay quiet there instead of
     * adding a boot-time warning to every unrelated build. */
    ESP_LOGD(TD_TAG, "run-time stats off: sampler disabled (enable "
                     "CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS)");
}

#endif /* CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS */

void twdt_diag_init(void)
{
    static bool done;
    if (done) {
        return;
    }
    done = true;

    td_install_log_tracker();
    /* Pinned after the app's wildcard level so a user setting of ERROR cannot
     * silence the instrumentation. */
    esp_log_level_set(TD_TAG, ESP_LOG_WARN);
    td_start_sampler();
}
