#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <fstream>
#include <map>
#include <cmath>
#include <time.h>
#include "Drawer.h"
#include "RedLed.h"
#include "memory.h"
#include "../include/Typestate_Library/TypestateLibrary_stub.h"





#if defined(__clang__)
  #define WCET_ANNOTATE(msg) __attribute__((annotate(msg)))
#else
  #define WCET_ANNOTATE(msg)
#endif

// ==================== Timing primitives ====================
#if defined(__aarch64__)
// AArch64 precise busy-wait using CNTVCT_EL0
static inline uint64_t armv8_cntfrq(void) { uint64_t f; __asm__ volatile("mrs %0, cntfrq_el0":"=r"(f)); return f; }
static inline uint64_t armv8_cntvct(void) { uint64_t c; __asm__ volatile("isb; mrs %0, cntvct_el0":"=r"(c)); return c; }
static inline void hw_delay_ms(uint32_t ms){
    if (!ms) return;
    const uint64_t freq = armv8_cntfrq();
    __uint128_t ticks_needed = (__uint128_t)freq * ms / 1000u;
    uint64_t start = armv8_cntvct();
    for(;;){
        uint64_t now = armv8_cntvct();
        if ((__uint128_t)(now - start) >= ticks_needed) break;
        __asm__ volatile("yield");
    }
}
#else
// Portable busy-wait using CLOCK_MONOTONIC_RAW (for x86_64 dev runs)
static inline void hw_delay_ms(uint32_t ms){
    if (!ms) return;
    struct timespec now, end;
    clock_gettime(CLOCK_MONOTONIC, &now);
    end = now;
    end.tv_sec  += (time_t)(ms / 1000);
    end.tv_nsec += (long)(ms % 1000) * 1000000L;
    if (end.tv_nsec >= 1000000000L) { end.tv_sec++; end.tv_nsec -= 1000000000L; }
    do { clock_gettime(CLOCK_MONOTONIC, &now); }
    while (now.tv_sec < end.tv_sec ||
          (now.tv_sec == end.tv_sec && now.tv_nsec < end.tv_nsec));
}


static inline void hw_delay_ms_t(uint32_t ms){
    if (!ms) return;
    struct timespec now, end;
    clock_gettime(CLOCK_MONOTONIC, &now);
    end = now;
    end.tv_sec  += (time_t)(ms / 1000);
    end.tv_nsec += (long)(ms % 1000) * 1000000L;
    if (end.tv_nsec >= 1000000000L) { end.tv_sec++; end.tv_nsec -= 1000000000L; }
    do { clock_gettime(CLOCK_MONOTONIC, &now); }
    while (now.tv_sec < end.tv_sec ||
          (now.tv_sec == end.tv_sec && now.tv_nsec < end.tv_nsec));
}


static inline void hw_delay_ms_f(double ms)
{
    if (ms <= 0.0) return;

    struct timespec now, end;
    clock_gettime(CLOCK_MONOTONIC, &now);
    end = now;

    // ms -> seconds + nanoseconds
    double sec_total = ms / 1000.0;           // convert ms to seconds
    time_t add_sec   = (time_t)sec_total;     // whole seconds
    double frac_sec  = sec_total - (double)add_sec;
    long   add_nsec  = (long)(frac_sec * 1e9 + 0.5);  // fractional part to ns (rounded)

    end.tv_sec  += add_sec;
    end.tv_nsec += add_nsec;

    if (end.tv_nsec >= 1000000000L) {
        end.tv_sec++;
        end.tv_nsec -= 1000000000L;
    }

    do {
        clock_gettime(CLOCK_MONOTONIC, &now);
    } while (now.tv_sec < end.tv_sec ||
            (now.tv_sec == end.tv_sec && now.tv_nsec < end.tv_nsec));
}

/*
 * Integer-millisecond convenience wrapper.
 * Keeps your old hw_delay_ms(uint32_t) API working.
 */

/*
 * simulate_ms: "simulated work" expressed in milliseconds (float).
 * This simply uses the wall-clock delay, so simulate_ms(0.90f)
 * will wait for approximately 0.9 ms in real time.
 */
static inline void simulate_ms(float ms)
{
    if (ms <= 0.0f) return;
    hw_delay_ms_f((double)ms);
}

/*
 * Compatibility macro for your existing code:
 *
 *   simulate_(SIM_UNITS_FROM_VAL(0.90f));
 *
 * Here, 1 "unit" = 0.1 ms, so 0.90f -> about 9 units -> ~0.9 ms.
 */
#define SIM_UNITS_FROM_VAL(x)  ((uint32_t)((10.0f * (x)) + 0.5f))

/*
 * simulate_: takes "units" where each unit is 0.1 ms.
 * This keeps your old call sites working exactly the same way,
 * but now using real wall-clock time under Linux/WSL/BBB.
 */
static inline void simulate_(uint32_t units)
{
    if (units == 0u) return;
    double ms = (double)units * 0.1;   // units * 0.1 ms
    hw_delay_ms_f(ms);
}


#endif

using namespace TypestateLibrary::Template;

#define F_CPU 16000000UL

#ifdef WCET_ANNOTATE
#  undef WCET_ANNOTATE
#endif
#define WCET_ANNOTATE(...)

#define WCET_CAT2(a,b) a##b
#define WCET_CAT(a,b)  WCET_CAT2(a,b)

#define WCET_AT(payload)                                                    \
  static const char __attribute__((used, section(".wcet_next")))            \
  WCET_CAT(__wcet_next_, __COUNTER__)[] = payload

#ifndef CYCLES_PER_LOOP
#define CYCLES_PER_LOOP 7u
#endif

// ============================================================
// Trace event core
// ============================================================
typedef enum {
    RT_EV_BEGIN = 1,
    RT_EV_END   = 2
} rt_event_type_t;

typedef uint16_t rt_id_t;

typedef struct {
    uint64_t        tick_ns;
    rt_id_t         id;
    rt_event_type_t type;
} rt_event_t;

#ifndef RT_TRACE_CAPACITY
#define RT_TRACE_CAPACITY 65536u
#endif

static rt_event_t rt_trace_buf[RT_TRACE_CAPACITY];
static uint32_t   rt_trace_len = 0;

static inline uint64_t rt_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull +
           (uint64_t)ts.tv_nsec;
}

static inline void rt_trace_reset(void)
{
    rt_trace_len = 0;
}

static inline void rt_trace_record(rt_id_t id, rt_event_type_t type)
{
    if (rt_trace_len >= RT_TRACE_CAPACITY) return;
    rt_trace_buf[rt_trace_len].tick_ns = rt_now_ns();
    rt_trace_buf[rt_trace_len].id      = id;
    rt_trace_buf[rt_trace_len].type    = type;
    ++rt_trace_len;
}

#define TRACE_BEGIN(ID) rt_trace_record((rt_id_t)(ID), RT_EV_BEGIN)
#define TRACE_END(ID)   rt_trace_record((rt_id_t)(ID), RT_EV_END)

// ============================================================
// PillBox trace IDs
// ============================================================
enum {
    PB_CONSTRUCTOR        = 1,
    PB_DESTRUCTOR         = 2,
    PB_ACTIVATE           = 3,
    PB_ADD_DRAWERS        = 4,
    PB_PROCESS_TIME       = 5,
    PB_DEACTIVATE         = 6,
    PB_SWITCH_ON          = 7,
    PB_SWITCH_OFF         = 8,
    PB_BLINK              = 9,
    PB_OPEN_DRAWER        = 10,
    PB_CHECK_TIMEOUTS     = 11,
    PB_PRINT_STATUS       = 12,
    PB_FIND_DRAWER_INDEX  = 13,
    PB_CYCLE              = 14
};

static inline const char* pillbox_trace_label(rt_id_t id)
{
    switch (id) {
        case PB_ACTIVATE:          return "Activate_pillBox";
        case PB_ADD_DRAWERS:       return "addDrawers";
        case PB_PROCESS_TIME:      return "Process_System_Time";
        case PB_DEACTIVATE:        return "Deactivate_Pill_Box";
        case PB_SWITCH_ON:         return "Switch_ON";
        case PB_SWITCH_OFF:        return "Switch_OFF";
        case PB_BLINK:             return "Blink";
        case PB_OPEN_DRAWER:       return "OpenDrawer";
        case PB_CHECK_TIMEOUTS:    return "CheckTimeouts";
        case PB_PRINT_STATUS:      return "PrintStatus";
        case PB_FIND_DRAWER_INDEX: return "findDrawerIndex";
        case PB_CYCLE:             return "PB_CYCLE";
        default:                   return "UNKNOWN";
    }
}

static inline void rt_trace_dump_pretty(void)
{
    if (rt_trace_len == 0) {
        puts("[trace] (empty)");
        return;
    }

    uint64_t t0 = rt_trace_buf[0].tick_ns;
    puts("---- TRACE (relative, microseconds) ----");
    for (uint32_t i = 0; i < rt_trace_len; ++i) {
        const rt_event_t *e = &rt_trace_buf[i];
        uint64_t rel_ns = e->tick_ns - t0;
        double   rel_us = (double)rel_ns / 1000.0;
        const char *type_str = (e->type == RT_EV_BEGIN) ? "BEGIN" : "END  ";
        printf("[%5u] +%12.3f us  %-5s  id=%u  %-24s\n",
               i, rel_us, type_str,
               (unsigned)e->id, pillbox_trace_label(e->id));
    }
    puts("----------------------------------------");
}

static inline void rt_trace_dump_durations(void)
{
    if (rt_trace_len == 0) return;

    static uint8_t matched[RT_TRACE_CAPACITY];
    for (uint32_t i = 0; i < rt_trace_len; ++i) matched[i] = 0;

    puts("---- DURATIONS (BEGIN->END, milliseconds) ----");
    for (int end_i = 0; end_i < (int)rt_trace_len; ++end_i) {
        const rt_event_t *end = &rt_trace_buf[end_i];
        if (end->type != RT_EV_END) continue;

        for (int beg_i = end_i - 1; beg_i >= 0; --beg_i) {
            const rt_event_t *beg = &rt_trace_buf[beg_i];
            if (beg->type == RT_EV_BEGIN &&
                beg->id   == end->id &&
                !matched[beg_i]) {

                matched[beg_i] = matched[end_i] = 1;
                uint64_t dt_ns = end->tick_ns - beg->tick_ns;
                double   dt_ms = (double)dt_ns / 1e6;
                printf("%-24s : %.6f ms\n",
                       pillbox_trace_label(beg->id), dt_ms);
                break;
            }
        }
    }
    puts("---------------------------------------------");
}

static inline void rt_trace_dump_means(void)
{
    if (rt_trace_len == 0) {
        puts("[trace] (empty, no means)");
        return;
    }

    enum { RT_ID_MAX = 64 };
    double   sum_ms[RT_ID_MAX];
    unsigned cnt[RT_ID_MAX];
    static uint8_t matched[RT_TRACE_CAPACITY];

    for (int i = 0; i < RT_ID_MAX; ++i) {
        sum_ms[i] = 0.0;
        cnt[i]    = 0u;
    }
    for (uint32_t i = 0; i < rt_trace_len; ++i) matched[i] = 0;

    for (int end_i = 0; end_i < (int)rt_trace_len; ++end_i) {
        const rt_event_t *end = &rt_trace_buf[end_i];
        if (end->type != RT_EV_END) continue;

        for (int beg_i = end_i - 1; beg_i >= 0; --beg_i) {
            const rt_event_t *beg = &rt_trace_buf[beg_i];
            if (beg->type == RT_EV_BEGIN &&
                beg->id   == end->id &&
                !matched[beg_i]) {

                matched[beg_i] = matched[end_i] = 1;
                double dur_ms = (double)(end->tick_ns - beg->tick_ns) / 1e6;

                if (beg->id < RT_ID_MAX) {
                    sum_ms[beg->id] += dur_ms;
                    cnt[beg->id]    += 1u;
                }
                break;
            }
        }
    }

    puts("---- MEAN DURATIONS PER ID (milliseconds) ----");
    for (rt_id_t id = 0; id < RT_ID_MAX; ++id) {
        if (cnt[id] == 0u) continue;
        double mean = sum_ms[id] / (double)cnt[id];
        printf("%-24s : mean = %.6f ms  (n=%u)\n",
               pillbox_trace_label(id), mean, cnt[id]);
    }
    puts("----------------------------------------------");
}

struct RtStats {
    int    n = 0;
    double mean = 0.0;
    double M2 = 0.0;
    double minv = 0.0;
    double maxv = 0.0;

    void add(double x) {
        n++;
        if (n == 1) {
            mean = x;
            M2   = 0.0;
            minv = x;
            maxv = x;
            return;
        }
        if (x < minv) minv = x;
        if (x > maxv) maxv = x;

        double delta  = x - mean;
        mean         += delta / (double)n;
        double delta2 = x - mean;
        M2           += delta * delta2;
    }

    double stddev_sample() const {
        if (n < 2) return 0.0;
        return std::sqrt(M2 / (double)(n - 1));
    }
};

static inline void rt_trace_dump_csv_with_stddev(const char* filename)
{
    if (rt_trace_len == 0) return;

    std::ofstream file(filename);
    file << "FUNCTION,Iteration,Duration_ms\n";

    static uint8_t matched[RT_TRACE_CAPACITY];
    for (uint32_t i = 0; i < rt_trace_len; ++i) matched[i] = 0;

    std::map<rt_id_t,int>     iteration_count;
    std::map<rt_id_t,RtStats> stats;

    for (int end_i = 0; end_i < (int)rt_trace_len; ++end_i) {
        const rt_event_t *end = &rt_trace_buf[end_i];
        if (end->type != RT_EV_END) continue;

        for (int beg_i = end_i - 1; beg_i >= 0; --beg_i) {
            const rt_event_t *beg = &rt_trace_buf[beg_i];

            if (beg->type == RT_EV_BEGIN &&
                beg->id   == end->id &&
                !matched[beg_i]) {

                matched[beg_i] = matched[end_i] = 1;

                double dt_ms = (double)(end->tick_ns - beg->tick_ns) / 1e6;
                int it = ++iteration_count[beg->id];

                file << pillbox_trace_label(beg->id)
                     << "," << it
                     << "," << dt_ms
                     << "\n";

                stats[beg->id].add(dt_ms);
                break;
            }
        }
    }

    file << "\nSUMMARY\n";
    file << "State,Mean_ms,StdDev_ms,Min_ms,Max_ms,Samples\n";

    for (const auto &kv : stats) {
        rt_id_t id = kv.first;
        const RtStats &s = kv.second;

        file << pillbox_trace_label(id) << ","
             << s.mean << ","
             << s.stddev_sample() << ","
             << s.minv << ","
             << s.maxv << ","
             << s.n
             << "\n";
    }

    file.close();
}

// ============================================================
// PillBox model
// ============================================================
enum class domain {
    Idle = 0,
    Active = 1,
    NonActive = 2,
    Pill_Time_On = 3,
    RedLedON = 4,
    RedLedOFF = 5,
    RedLedBlinking = 6
};

enum PillMessageCode {
    MSG_NONE = 0,
    MSG_TAKE = 1,
    MSG_CLOSE = 2,
    MSG_MISSED = 3,
    MSG_NOT_CLOSED = 4,
    MSG_ACTIVATED = 5,
    MSG_DEACTIVATED = 6,
    MSG_NO_SCHEDULE = 7
};

class PillBox {
public:
    static const int MAX_DRAWERS = 3;

    PillBox();
    ~PillBox();

    __attribute__((noinline, used)) void Activate_pillBox();
    __attribute__((noinline, used)) void addDrawers(Drawer* d);
    __attribute__((noinline, used)) Drawer* Process_System_Time(int h, int m);
    __attribute__((noinline, used)) void Deactivate_Pill_Box();
    __attribute__((noinline, used)) void Switch_ON(Drawer* d);
    __attribute__((noinline, used)) void Switch_OFF(Drawer* d);
    __attribute__((noinline, used)) void Blink(Drawer* d);

    __attribute__((noinline, used)) void OpenDrawer(Drawer* d);
    __attribute__((noinline, used)) void CheckTimeouts();
    __attribute__((noinline, used)) void PrintStatus();

private:
    Drawer* DrawersBox[MAX_DRAWERS];
    bool drawerOpened[MAX_DRAWERS];
    unsigned int drawerTimer[MAX_DRAWERS];
    bool drawerTaken[MAX_DRAWERS];
    bool drawerMissed[MAX_DRAWERS];
    bool drawerCloseAlert[MAX_DRAWERS];
    int drawerCount;

    RedLed redled;

    bool isActive;
    int currentTimeHour;
    int currentTimeMinute;
    unsigned int systemTick;

    Drawer* lastTriggeredDrawer;
    int lastTriggeredIndex;

    int outMessageCode;
    int logMessageCode;

    volatile int debug_state;
    volatile int debug_value0;
    volatile int debug_value1;

    __attribute__((noinline, used)) int findDrawerIndex(Drawer* d);
};

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
PillBox::PillBox()
    : drawerCount(0),
      isActive(false),
      currentTimeHour(0),
      currentTimeMinute(0),
      systemTick(0),
      lastTriggeredDrawer(nullptr),
      lastTriggeredIndex(-1),
      outMessageCode(MSG_NONE),
      logMessageCode(MSG_NONE),
      debug_state(0),
      debug_value0(0),
      debug_value1(0)
{
  

    for (int i = 0; i < MAX_DRAWERS; ++i) {
        DrawersBox[i] = nullptr;
        drawerOpened[i] = false;
        drawerTimer[i] = 0;
        drawerTaken[i] = false;
        drawerMissed[i] = false;
        drawerCloseAlert[i] = false;
    }

   
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
PillBox::~PillBox() {
    TRACE_BEGIN(PB_DESTRUCTOR);
    TRACE_END(PB_DESTRUCTOR);
}

WCET_AT("wcet_ms_exact=0.001125; cycles=18");
void PillBox::Activate_pillBox() {
    TRACE_BEGIN(PB_ACTIVATE);

    if (!isActive) {
        isActive = true;
        outMessageCode = MSG_ACTIVATED;
        logMessageCode = MSG_ACTIVATED;
        debug_state = 1;
    } else {
        debug_state = 2;
    }

    TRACE_END(PB_ACTIVATE);
}

WCET_AT("wcet_ms_exact=0.002250; cycles=36");
void PillBox::addDrawers(Drawer* d) {
    TRACE_BEGIN(PB_ADD_DRAWERS);

    if (d == nullptr) {
        debug_state = -1;
        TRACE_END(PB_ADD_DRAWERS);
        return;
    }

    if (drawerCount >= MAX_DRAWERS) {
        debug_state = -2;
        TRACE_END(PB_ADD_DRAWERS);
        return;
    }

    DrawersBox[drawerCount] = d;
    drawerOpened[drawerCount] = false;
    drawerTimer[drawerCount] = 0;
    drawerTaken[drawerCount] = false;
    drawerMissed[drawerCount] = false;
    drawerCloseAlert[drawerCount] = false;
    drawerCount++;

    debug_state = 10;
    debug_value0 = drawerCount;

    TRACE_END(PB_ADD_DRAWERS);
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
Drawer* PillBox::Process_System_Time(int h, int m) {
    TRACE_BEGIN(PB_PROCESS_TIME);

    currentTimeHour = h;
    currentTimeMinute = m;
    systemTick = systemTick + 1u;

    debug_state = 20;
    debug_value0 = h;
    debug_value1 = m;

    lastTriggeredDrawer = nullptr;
    lastTriggeredIndex = -1;
    outMessageCode = MSG_NO_SCHEDULE;

    for (int i = 0; i < MAX_DRAWERS; ++i) {
        Drawer* d = DrawersBox[i];
        if (d == nullptr) {
            continue;
        }

        int dh = d->get_the_hour();
        int dm = d->get_minutes();

        if (h == dh) {
            if (m == dm) {
                lastTriggeredDrawer = d;
                lastTriggeredIndex = i;
                drawerTimer[i] = systemTick;
                drawerTaken[i] = false;
                drawerMissed[i] = false;
                drawerCloseAlert[i] = false;
                outMessageCode = MSG_TAKE;
                logMessageCode = MSG_TAKE;
                debug_state = 21;
                debug_value0 = i;
                TRACE_END(PB_PROCESS_TIME);
                return d;
            }
        }
    }

    TRACE_END(PB_PROCESS_TIME);
    return nullptr;
}

WCET_AT("wcet_ms_exact=0.001187; cycles=19");
void PillBox::Deactivate_Pill_Box() {
    TRACE_BEGIN(PB_DEACTIVATE);

    if (isActive) {
        isActive = false;
        outMessageCode = MSG_DEACTIVATED;
        logMessageCode = MSG_DEACTIVATED;
        debug_state = 30;
    } else {
        debug_state = 31;
    }

    TRACE_END(PB_DEACTIVATE);
}

WCET_AT("wcet_ms_exact=40.052687; cycles=640843");
void PillBox::Switch_ON(Drawer* d) {
    TRACE_BEGIN(PB_SWITCH_ON);

    if (d == nullptr) {
        debug_state = -3;
        TRACE_END(PB_SWITCH_ON);
        return;
    }

    redled.setRedLedState(REDLED_ON);

    int idx = findDrawerIndex(d);
    if (idx != -1) {
        drawerTimer[idx] = systemTick;
        outMessageCode = MSG_TAKE;
        logMessageCode = MSG_TAKE;
        debug_state = 40;
        debug_value0 = idx;
    } else {
        debug_state = -4;
    }

    hw_delay_ms(40);

    TRACE_END(PB_SWITCH_ON);
}

WCET_AT("wcet_ms_exact=6.013438; cycles=96215");
void PillBox::Switch_OFF(Drawer* d) {
    TRACE_BEGIN(PB_SWITCH_OFF);

    if (d == nullptr) {
        debug_state = -5;
        TRACE_END(PB_SWITCH_OFF);
        return;
    }

    redled.setRedLedState(REDLED_OFF);
    d->SetDrawerStateCode(DRAWER_CLOSED);

    int idx = findDrawerIndex(d);
    if (idx != -1) {
        drawerOpened[idx] = false;
        drawerTaken[idx] = true;
        drawerCloseAlert[idx] = false;
        drawerTimer[idx] = systemTick;
        outMessageCode = MSG_NONE;
        logMessageCode = MSG_NONE;
        debug_state = 50;
        debug_value0 = idx;
    } else {
        debug_state = -6;
    }

    hw_delay_ms(6);

    TRACE_END(PB_SWITCH_OFF);
}

WCET_AT("wcet_ms_exact=0.004938; cycles=79");
void PillBox::Blink(Drawer* d) {
    TRACE_BEGIN(PB_BLINK);

    if (d == nullptr) {
        debug_state = -7;
        TRACE_END(PB_BLINK);
        return;
    }

    redled.setRedLedState(REDLED_BLINKING);

    int idx = findDrawerIndex(d);
    if (idx != -1) {
        if (drawerOpened[idx] == false) {
            outMessageCode = MSG_TAKE;
            logMessageCode = MSG_TAKE;
        } else {
            outMessageCode = MSG_CLOSE;
            logMessageCode = MSG_NOT_CLOSED;
            drawerCloseAlert[idx] = true;
        }
        debug_state = 60;
        debug_value0 = idx;
    } else {
        debug_state = -8;
    }

    TRACE_END(PB_BLINK);
}

WCET_AT("wcet_ms_exact=40.052500; cycles=640840");
void PillBox::OpenDrawer(Drawer* d) {
    TRACE_BEGIN(PB_OPEN_DRAWER);

    if (d == nullptr) {
        TRACE_END(PB_OPEN_DRAWER);
        return;
    }

    int idx = findDrawerIndex(d);
    if (idx != -1) {
        drawerOpened[idx] = true;
        drawerTimer[idx] = systemTick;
        d->SetDrawerStateCode(DRAWER_OPEN);
        outMessageCode = MSG_CLOSE;
        logMessageCode = MSG_CLOSE;
        debug_state = 70;
        debug_value0 = idx;
    }

    hw_delay_ms(40);

    TRACE_END(PB_OPEN_DRAWER);
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void PillBox::CheckTimeouts() {
    TRACE_BEGIN(PB_CHECK_TIMEOUTS);

    for (int i = 0; i < MAX_DRAWERS; ++i) {
        Drawer* d = DrawersBox[i];
        if (d == nullptr) {
            continue;
        }

        if (!drawerTaken[i] && !drawerOpened[i] && (systemTick - drawerTimer[i] > 3u)) {
            drawerMissed[i] = true;
            redled.setRedLedState(REDLED_OFF);
            outMessageCode = MSG_NONE;
            logMessageCode = MSG_MISSED;
            debug_state = 80;
            debug_value0 = i;
        }

        if (drawerOpened[i] && (systemTick - drawerTimer[i] > 3u)) {
            redled.setRedLedState(REDLED_OFF);
            outMessageCode = MSG_NONE;
            logMessageCode = MSG_NOT_CLOSED;
            debug_state = 81;
            debug_value0 = i;
        }
    }

    TRACE_END(PB_CHECK_TIMEOUTS);
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void PillBox::PrintStatus() {
    TRACE_BEGIN(PB_PRINT_STATUS);

    debug_state = 90;
    debug_value0 = isActive ? 1 : 0;
    debug_value1 = drawerCount;

    for (int i = 0; i < MAX_DRAWERS; ++i) {
        Drawer* d = DrawersBox[i];
        int opened = 0;
        int taken = 0;

        if (d != nullptr) {
            opened = drawerOpened[i] ? 1 : 0;
            taken = drawerTaken[i] ? 1 : 0;
        }

        debug_value0 = debug_value0 + opened;
        debug_value1 = debug_value1 + taken;
    }

    TRACE_END(PB_PRINT_STATUS);
}

WCET_AT("wcet_ms_exact=0.001187; cycles=19");
int PillBox::findDrawerIndex(Drawer* d) {
    
    for (int i = 0; i < MAX_DRAWERS; ++i) {
        if (DrawersBox[i] == d) {
            TRACE_END(PB_FIND_DRAWER_INDEX);
            return i;
        }
    }

   
    return -1;
}

// ============================================================
// Typestate
// ============================================================
using PillBox_typestate = Typestate_Template<
    Cycle<ms(120)>,
    Timed_State<
        domain::Idle,
        &PillBox::Activate_pillBox,
        TimeGuard<0, 0, lower(ms(0)), upper(ms(10)), Criticality::High>,
        domain::Active
    >,
    Timed_State<
        domain::Active,
        &PillBox::Process_System_Time,
        TimeGuard<0, 0, lower(ms(0)), upper(ms(2)), Criticality::High>,
        domain::Pill_Time_On
    >,
    Timed_State<
        domain::Pill_Time_On,
        &PillBox::Switch_ON,
        TimeGuard<0, 0, lower(ms(40)), upper(ms(50)), Criticality::High>,
        domain::RedLedON
    >,
    Timed_State<
        domain::RedLedON,
        &PillBox::OpenDrawer,
        TimeGuard<0, 0, lower(ms(40)), upper(ms(50)), Criticality::Soft>,
        domain::RedLedBlinking
    >,
    Timed_State<
        domain::RedLedBlinking,
        &PillBox::Switch_OFF,
        TimeGuard<0, 0, lower(ms(5)), upper(ms(10)), Criticality::Soft>,
        domain::RedLedOFF
    >,
    Timed_State<
        domain::RedLedOFF,
        &PillBox::Deactivate_Pill_Box,
        TimeGuard<0, 0, lower(ms(0)), upper(ms(2)), Criticality::High>,
        domain::NonActive
    >
>;

using TypestateClassConnectorPillbox = TypestateClassConnector<PillBox, PillBox_typestate>;
static TypestateClassConnectorPillbox FS;

static inline void init_typestate_arm_pillbox() {
    FS.display();
}

static inline void pillbox_trace_run_100()
{
    rt_trace_reset();

    for (int iter = 0; iter < 100; ++iter) {
        TRACE_BEGIN(PB_CYCLE);

        PillBox pb;
        Drawer d1, d2, d3;

        // IMPORTANT:
        // Set drawer schedules so one drawer is actually due at 8:30.
        // Replace these with your real Drawer API.
        d1.hour =8;
        d1.minutes =30;

        d2.hour = 9;
        d2.minutes =0;

        d3.hour =10;
        d3.minutes =15;

        pb.addDrawers(&d1);
        pb.addDrawers(&d2);
      

        pb.Activate_pillBox();

        Drawer* due = pb.Process_System_Time(8, 30);

        if (due != nullptr) {
            pb.Switch_ON(due);     // ~40 ms
            pb.OpenDrawer(due);    // ~40 ms
            pb.Switch_OFF(due);    // ~6 ms
        }

        pb.PrintStatus();
        pb.Deactivate_Pill_Box();

        TRACE_END(PB_CYCLE);
    }
}

int main()
{
    printf("PILLBOX traced run (100 iterations)\n");

    pillbox_trace_run_100();

    rt_trace_dump_pretty();
    rt_trace_dump_durations();
    rt_trace_dump_means();
    rt_trace_dump_csv_with_stddev("trace(PILLBOX)_results.csv");

    return 0;
}