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
#include "esp_rom_sys.h"  /* esp_rom_printf: lock-free console write, safe in abort context */
#include "esp_system.h"   /* esp_register_shutdown_handler */
#include "esp_intr_alloc.h" /* esp_intr_dump (abort-context last resort) */
#if CONFIG_IDF_TARGET_ESP32S3
#include "soc/soc.h"           /* REG_READ, DR_REG_*_BASE */
#include "soc/interrupt_reg.h" /* -> soc/interrupt_core0_reg.h: INTR_STATUS_0..3 */
#include "soc/bb_reg.h"        /* BBPD_CTRL  (WiFi BB power ctrl)  */
#include "soc/nrx_reg.h"       /* NRXPD_CTRL (WiFi RX power ctrl)  */
#include "soc/fe_reg.h"        /* FE_GEN_CTRL (RF front-end ctrl)  */
#endif
#if CONFIG_ESP_TASK_WDT_EN
#include "esp_task_wdt.h" /* esp_task_wdt_isr_user_handler (weak; we override it) */
#endif
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
#define TD_INIT_TASKS  32
#define TD_PERIOD_US   500000

static esp_timer_handle_t s_timer;
static TaskStatus_t s_prev[TD_MAX_TASKS];
static TaskStatus_t s_cur[TD_MAX_TASKS];   /* static: esp_timer task stack is 2KB */
static UBaseType_t s_prev_n;
static int64_t s_prev_us;
static bool s_have_prev;
static uint32_t s_seq;

/* Panic-dump baseline: snapshot taken in twdt_diag_init before the sampler
 * starts. Static, never on a stack - the dump runs from abort context. */
static TaskStatus_t s_init_prev[TD_INIT_TASKS];
static UBaseType_t s_init_prev_n;
static int64_t s_init_wall0;               /* 0 = baseline never taken */
static volatile uint32_t s_cb_count;
static volatile uint32_t s_last_cb_rel_ms;

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

/* Panic-dump baseline, taken in twdt_diag_init BEFORE the sampler starts. */
static void td_take_baseline(void)
{
    UBaseType_t n = uxTaskGetSystemState(s_init_prev, TD_INIT_TASKS, NULL);
    if (n > TD_INIT_TASKS) {
        n = TD_INIT_TASKS;
    }
    s_init_prev_n = n;
    s_init_wall0 = esp_timer_get_time();
    s_cb_count = 0;
    s_last_cb_rel_ms = 0;
    /* Seed the rolling sampler from the same snapshot. */
    td_snapshot(s_init_prev, n, s_init_wall0);
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

    s_cb_count++;
    if (s_init_wall0 != 0) {
        s_last_cb_rel_ms = (uint32_t)((now - s_init_wall0) / 1000);
    }

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

    const uint64_t wall = (uint64_t)wall_us;
    /* Run-time counters accumulate across BOTH cores, so a single-core wall
     * under-estimates the budget; isr2~ uses 2*wall (dual core). */
    const uint64_t wall2 = wall * 2u;
    const uint64_t isr  = (wall > sum) ? (wall - sum) : 0u;
    const uint64_t isr2 = (wall2 > sum) ? (wall2 - sum) : 0u;

    s_seq++;
    ESP_LOGE(TD_TAG,
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

/* ------------------------------------------------------------------ */
/* Patch 4: TWDUMP - one-shot per-task dump against the init baseline. */
/*                                                                      */
/* Runs in abort context (TWDT ISR user handler / shutdown handler): no  */
/* malloc, no stack buffers, and the primary output path is the ROM      */
/* console (esp_rom_printf) because the ESP log mutex may be held by a   */
/* preempted task during the abort - esp_log_write would wait 10ms       */
/* (MAX_MUTEX_WAIT_MS, components/log/src/os/log_lock.c) and silently    */
/* drop the line.                                                        */
/* ------------------------------------------------------------------ */

static bool s_dumped;   /* fire at most once (ISR handler + shutdown handler) */
static TaskStatus_t s_dump[TD_INIT_TASKS];   /* fresh snapshot; static, abort context */

static uint64_t td_dump_delta(const TaskStatus_t *cur)
{
    for (UBaseType_t i = 0; i < s_init_prev_n; i++) {
        if (s_init_prev[i].xHandle == cur->xHandle) {
            return (uint64_t)(cur->ulRunTimeCounter - s_init_prev[i].ulRunTimeCounter);
        }
    }
    /* Task created after the baseline: its whole counter is delta. */
    return (uint64_t)cur->ulRunTimeCounter;
}

static uint64_t td_dump_delta_by_name(const TaskStatus_t *cur, UBaseType_t n, const char *name)
{
    uint64_t acc = 0;
    for (UBaseType_t i = 0; i < n; i++) {
        if (cur[i].pcTaskName != NULL && strcmp(cur[i].pcTaskName, name) == 0) {
            acc += td_dump_delta(&cur[i]);
        }
    }
    return acc;
}

#if CONFIG_IDF_TARGET_ESP32S3
/* ------------------------------------------------------------------ */
/* Interrupt-controller + WiFi-domain attribution snapshot (round 6).  */
/*                                                                      */
/* All accesses read-only, safe from the TWDT abort ISR. Register names */
/* come from the v6.1 SoC headers:                                     */
/*   soc/interrupt_core0_reg.h : INTERRUPT_CORE0_INTR_STATUS_0..3 (RO, */
/*                               base+0x18C..0x198, pending status of  */
/*                               the mapped peripheral sources; IST    */
/*                               bit N == periph_interrupt_t source N) */
/*   soc/bb_reg.h              : BBPD_CTRL  (WiFi BB power control)    */
/*   soc/nrx_reg.h             : NRXPD_CTRL (WiFi RX power control)    */
/*   soc/fe_reg.h              : FE_GEN_CTRL (RF front-end control)    */
/* NOTE: the ESP32-S3 interrupt core has NO                       */
/* INTERRUPT_CORE0_CPU_INT_ENABLE_REG / CPU_INT_THRESH_REG (those      */
/* exist on the RISC-V interrupt core, e.g. esp32c3); on Xtensa the    */
/* CPU-side enable mask is the INTENABLE special register and the      */
/* pending mask is INTERRUPT, read here via RSR. #ifdef below keeps    */
/* the MMIO variant if a future header provides it.                    */
/* ------------------------------------------------------------------ */
static void td_dump_intc(void)
{
#ifdef INTERRUPT_CORE0_CPU_INT_ENABLE_REG
    const uint32_t ien = (uint32_t)REG_READ(INTERRUPT_CORE0_CPU_INT_ENABLE_REG);
#else
    uint32_t ien;
    __asm__ __volatile__("rsr %0, INTENABLE" : "=r"(ien));
#endif
    uint32_t ipend;
    __asm__ __volatile__("rsr %0, INTERRUPT" : "=r"(ipend));

    const uint32_t ist0 = (uint32_t)REG_READ(INTERRUPT_CORE0_INTR_STATUS_0_REG);
    const uint32_t ist1 = (uint32_t)REG_READ(INTERRUPT_CORE0_INTR_STATUS_1_REG);
    const uint32_t ist2 = (uint32_t)REG_READ(INTERRUPT_CORE0_INTR_STATUS_2_REG);
    const uint32_t ist3 = (uint32_t)REG_READ(INTERRUPT_CORE0_INTR_STATUS_3_REG);

    esp_rom_printf("IEN=0x%08x IPEND=0x%08x\r\n", ien, ipend);
    esp_rom_printf("IST=0x%08x/0x%08x/0x%08x/0x%08x\r\n", ist0, ist1, ist2, ist3);
    /* Static legend so a stuck bit is nameable without the headers:
     * IST bit positions = periph_interrupt_t source numbers
     * (soc/interrupts.h); CPU-int reservations from
     * esp_hw_support/port/esp32s3/esp_cpu_intr.c. */
    esp_rom_printf("IENLEG: IST b0=WMAC b1=WMAC_NMI b2=WPWR b3=WBB b8=RWBLE "
                   "b12=SLC0 b13=SLC1 b49=TG0_WDT b53=SYSTIMER; "
                   "IPEND b0=WMAC b4=WBB b14=NMI b24=T1WDT b28=IPC\r\n");
}
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

static void td_panic_dump(void)
{
    if (s_init_wall0 == 0 || s_dumped) {
        return;   /* baseline never taken, or already dumped */
    }
    s_dumped = true;

    UBaseType_t n = uxTaskGetSystemState(s_dump, TD_INIT_TASKS, NULL);
    if (n > TD_INIT_TASKS) {
        n = TD_INIT_TASKS;
    }

    const int64_t now = esp_timer_get_time();
    const uint64_t wall_us =
        (now > s_init_wall0) ? (uint64_t)(now - s_init_wall0) : 0u;

    uint64_t sum = 0;
    /* Top-3 CPU-time consumers by delta (attribution when a task is not one
     * of the named fields below). Updated in the same single pass. */
    const char *top_name[3] = { "-", "-", "-" };
    uint64_t    top_ms[3]   = { 0, 0, 0 };
    for (UBaseType_t i = 0; i < n; i++) {
        const uint64_t d = td_dump_delta(&s_dump[i]);
        sum += d;
        const uint64_t dm = d / 1000u;
        for (int t = 0; t < 3; t++) {
            if (dm > top_ms[t]) {
                for (int u = 2; u > t; u--) {
                    top_ms[u]   = top_ms[u - 1];
                    top_name[u] = top_name[u - 1];
                }
                top_ms[t]   = dm;
                top_name[t] = (s_dump[i].pcTaskName != NULL) ? s_dump[i].pcTaskName : "?";
                break;
            }
        }
    }

    /* All values reported in milliseconds (run-time counters are us). */
    const uint64_t wall = wall_us / 1000u;
    const uint64_t sum_ms = sum / 1000u;
    const uint64_t isr = (wall > sum_ms) ? (wall - sum_ms) : 0u;
    const uint64_t wall2 = wall * 2u;
    const uint64_t isr2 = (wall2 > sum_ms) ? (wall2 - sum_ms) : 0u;

    static char s_line[384];   /* static: abort context, never stack */
    snprintf(s_line, sizeof(s_line),
             "TWDUMP seq=%u cbs=%u lastcb=%ums wall=%ums sum=%ums isr~=%ums isr2~=%ums "
             "idle=%ums wifi=%ums tcpip=%ums main=%ums mdns=%ums tmr=%ums idle1=%ums "
             "sys_evt=%ums bt=%ums top1=%s:%ums top2=%s:%ums top3=%s:%ums",
             (unsigned)s_seq, (unsigned)s_cb_count, (unsigned)s_last_cb_rel_ms,
             (unsigned)wall, (unsigned)sum_ms, (unsigned)isr, (unsigned)isr2,
             (unsigned)(td_dump_delta_by_name(s_dump, n, "IDLE") / 1000u),
             (unsigned)(td_dump_delta_by_name(s_dump, n, "wifi") / 1000u),
             (unsigned)(td_dump_delta_by_name(s_dump, n, "tcpip") / 1000u),
             (unsigned)(td_dump_delta_by_name(s_dump, n, "main") / 1000u),
             (unsigned)(td_dump_delta_by_name(s_dump, n, "mdns") / 1000u),
             (unsigned)(td_dump_delta_by_name(s_dump, n, "esp_timer") / 1000u),
             (unsigned)(td_dump_delta_by_name(s_dump, n, "IDLE1") / 1000u),
             (unsigned)(td_dump_delta_by_name(s_dump, n, "sys_evt") / 1000u),
             (unsigned)(td_dump_delta_by_name(s_dump, n, "bt") / 1000u),
             top_name[0], (unsigned)top_ms[0],
             top_name[1], (unsigned)top_ms[1],
             top_name[2], (unsigned)top_ms[2]);

    /* ROM console only from here: no locks, no malloc. Flash validation of
     * diag6 showed every line printed AFTER the old esp_log_write() call was
     * missing from serial (TWDUMP-END absent, IEN/IPEND/IST/WRF lost) - the
     * log mutex + newlib in TWDT ISR context wedges the abort path and the
     * panic reentrancy guard restarts with suppressed output. The logger
     * belt-and-braces call was therefore REMOVED; rom_printf is the only
     * output path allowed in this function. */
    esp_rom_printf("%s\r\n", s_line);
    esp_rom_printf("STAGE1\r\n");
#if CONFIG_IDF_TARGET_ESP32S3
    /* Interrupt-controller + WiFi-domain registers: which CPU interrupt is
     * pending/stuck and which peripheral sources are asserting. */
    td_dump_intc();
    /* Public WiFi power/ctrl regs (soc/bb_reg.h, nrx_reg.h, fe_reg.h):
     * values show BB/RX/FE block power-state force bits if the MAC is wedged. */
    esp_rom_printf("WRF BBPD=0x%08x NRXPD=0x%08x FEGC=0x%08x\r\n",
                   (unsigned)REG_READ(BBPD_CTRL),
                   (unsigned)REG_READ(NRXPD_CTRL),
                   (unsigned)REG_READ(FE_GEN_CTRL));
#endif
    esp_rom_printf("TWDUMP-END\r\n");
#if 0
    /* Disabled (diag6 evidence): esp_intr_dump() uses newlib FILE streams /
     * fprintf - in TWDT ISR context it wedges the abort path; everything
     * after the old esp_log_write/log call produced no serial output.
     * Re-enable only if a lock-free dump path ever exists. */
    esp_intr_dump(NULL);
#endif
}

#if CONFIG_ESP_TASK_WDT_EN
/* Weak symbol from esp_task_wdt.h; the TWDT timeout ISR calls this right
 * before it aborts. On the v6.1 panic path (task_wdt_isr ->
 * xt_unhandled_exception -> esp_panic_handler -> panic_restart ->
 * esp_restart_noos) shutdown handlers never run, so this ISR hook is what
 * actually delivers the TWDUMP on a TWDT abort; the shutdown handler covers
 * esp_restart() paths (OTA reboot etc.). */
void esp_task_wdt_isr_user_handler(void)
{
    td_panic_dump();
}
#endif

static void td_register_panic_hooks(void)
{
    esp_register_shutdown_handler(td_panic_dump);
}

#else /* !CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS */

static void td_start_sampler(void)
{
    /* Other boards ship without run-time stats; stay quiet there instead of
     * adding a boot-time warning to every unrelated build. */
    ESP_LOGD(TD_TAG, "run-time stats off: sampler disabled (enable "
                     "CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS)");
}

static void td_register_panic_hooks(void) { /* no run-time counters to dump */ }
static void td_take_baseline(void) { /* no run-time counters to baseline */ }

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
    esp_log_level_set(TD_TAG, ESP_LOG_ERROR);
    /* Baseline BEFORE the sampler timer starts: TWDUMP diffs against this. */
    td_take_baseline();
    td_register_panic_hooks();
    td_start_sampler();
}
