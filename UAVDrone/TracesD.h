#ifndef DRONE_TRACE_H
#define DRONE_TRACE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#if defined(__clang__)
  #define WCET_ANNOTATE(msg) __attribute__((annotate(msg)))
#else
  #define WCET_ANNOTATE(msg)
#endif

// ==================== Timing primitives ====================
#if defined(__aarch64__)
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
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ----- Event types -----
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
// Drone IDs
// ============================================================
enum {
    DR_INIT_FMM          = 1,
    DR_INIT_FLIGHTPLAN   = 2,
    DR_CHECK_START       = 3,
    DR_IDLEMOTORS        = 4,
    DR_CHECK_ALTITUDE    = 5,
    DR_ACQUIRE           = 6,
    DR_TRAVELTOWAYPOINT  = 7,
    DR_NEUTRAL_DECISION  = 8,
    DR_DESCEND           = 9,
    DR_KILLMOTORS        = 10,
    DR_SETMOTORS         = 11,
    DR_CYCLE             = 12
};

static inline const char* drone_trace_label(rt_id_t id)
{
    switch (id) {
        case DR_INIT_FMM:         return "initialize_fmm";
        case DR_INIT_FLIGHTPLAN:  return "initialize_flightplan";
        case DR_CHECK_START:      return "checkStart";
        case DR_IDLEMOTORS:       return "idlemotors";
        case DR_CHECK_ALTITUDE:   return "checkAltitude";
        case DR_ACQUIRE:          return "acquire";
        case DR_TRAVELTOWAYPOINT: return "traveltowaypoint";
        case DR_NEUTRAL_DECISION: return "neutralDecision";
        case DR_DESCEND:          return "descend";
        case DR_KILLMOTORS:       return "killmotors";
        case DR_SETMOTORS:        return "setmotors";
        case DR_CYCLE:            return "DR_CYCLE";
        default:                  return "UNKNOWN";
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
        printf("[%5u] +%12.3f us  %-5s  id=%u  %-20s\n",
               i, rel_us, type_str,
               (unsigned)e->id, drone_trace_label(e->id));
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
                printf("%-20s : %.6f ms\n",
                       drone_trace_label(beg->id), dt_ms);
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

    for (int i = 0; i < RT_ID_MAX; ++i) { sum_ms[i] = 0.0; cnt[i] = 0u; }
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
        printf("%-20s : mean = %.6f ms  (n=%u)\n",
               drone_trace_label(id), mean, cnt[id]);
    }
    puts("----------------------------------------------");
}

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
#include <fstream>
#include <map>
#include <cmath>

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

                file << drone_trace_label(beg->id)
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

        file << drone_trace_label(id) << ","
             << s.mean << ","
             << s.stddev_sample() << ","
             << s.minv << ","
             << s.maxv << ","
             << s.n
             << "\n";
    }

    file.close();
}

// Forward declaration
class DroneController;

// Run the same sequence as your main, 100 times
static inline void drone_trace_run_100(DroneController &drone)
{
    rt_trace_reset();

    constexpr float dt = 0.1f;

    for (int iter = 0; iter < 100; ++iter) {
        TRACE_BEGIN(DR_CYCLE);

        drone.initialize_fmm();
        drone.initialize_flightplan();

        drone.go_signal = 1;

        if (drone.checkStart() == WAIT_GO) {
            drone.killmotors();
            TRACE_END(DR_CYCLE);
            continue;
        }

        drone.idlemotors();

        if (drone.glng_counter == 0) {
            drone.glng_counter = 50;
        }
        drone.glng_counter--;

        if (drone.glng_counter > 0) {
            TRACE_END(DR_CYCLE);
            continue;
        }

        if (drone.checkAltitude() == ALT_NOT_REACHED) {
            drone.setmotors(ASCEND_DC);
            drone.curdrone.y += 1.0;
            TRACE_END(DR_CYCLE);
            continue;
        }

        drone.acquire();
        drone.traveltowaypoint(drone.wp_index, 1.0, 2.0, dt);

        switch (drone.neutralDecision()) {
        case CONTINUE_MISSION:
            drone.acquire();
            drone.traveltowaypoint(drone.wp_index, 1.0, 2.0, dt);
            break;

        case LAND_NOW:
            drone.descend();
            drone.curdrone.y -= 1.0;
            drone.killmotors();
            break;

        default:
            break;
        }

        TRACE_END(DR_CYCLE);
    }
}
#endif

#endif // DRONE_TRACE_H