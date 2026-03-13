#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <fstream>
#include <map>
#include <limits>
#include <cmath>

#define F_CPU 16000000UL

// ============================================================
// WCET annotation support
// ============================================================
#if defined(__clang__)
  #define WCET_ANNOTATE(msg) __attribute__((annotate(msg)))
#else
  #define WCET_ANNOTATE(msg)
#endif

#ifdef WCET_ANNOTATE
#  undef WCET_ANNOTATE
#endif
#define WCET_ANNOTATE(...)

#define WCET_CAT2(a,b) a##b
#define WCET_CAT(a,b)  WCET_CAT2(a,b)

#define WCET_AT(payload)                                                    \
  static const char __attribute__((used, section(".wcet_next")))            \
  WCET_CAT(__wcet_next_, __COUNTER__)[] = payload

// ============================================================
// Timing primitives
// ============================================================
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

static inline void hw_delay_ms_f(double ms)
{
    if (ms <= 0.0) return;

    struct timespec now, end;
    clock_gettime(CLOCK_MONOTONIC, &now);
    end = now;

    double sec_total = ms / 1000.0;
    time_t add_sec   = (time_t)sec_total;
    double frac_sec  = sec_total - (double)add_sec;
    long   add_nsec  = (long)(frac_sec * 1e9 + 0.5);

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

// ============================================================
// Drone model / state machine definitions
// ============================================================
static inline double dist3(double x1,double y1,double z1,
                           double x2,double y2,double z2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

enum StartDecision   { WAIT_GO, START_GO };
enum AltDecision     { ALT_NOT_REACHED, ALT_REACHED };
enum NeutralDecision { CONTINUE_MISSION, LAND_NOW };

#define OFF     0
#define ON      10
#define ASCEND  20
#define AQUIRE  30
#define TRAVEL  40
#define NEUTRAL 50
#define DESCEND 60

#define OFF_DC     0.00
#define IDLE_DC    0.10
#define ASCEND_DC  0.30
#define DESCEND_DC 0.20

class DroneController {
public:
    struct fmm_type {
        double f1a,f1b;
        double m1x,m1y,m1z;
        double f2a,f2b;
        double m2x,m2y,m2z;
        double f3a,f3b;
        double m3x,m3y,m3z;
        double f4a,f4b;
        double m4x,m4y,m4z;

        double weight;
        double cgx,cgy,cgz;
        double ip,iy,ir;

        double mfce;
    };

    DroneController()
    {
        curstate = OFF;
        go_signal = 0;
        glng_counter = 0L;

        batt_voltage = 1.2 * 6.0;
        batt_current = 0.00;
        batt_energy  = 2.5 * 6.0;
        cur_batt_energy = 0.00;

        done = 0;

        for (int i=0;i<8;i++) esc[i].dutycycle = 0.0;
    }

    struct curdrone_type {
        double x,y,z;
        double vx,vy,vz;
        double ax,ay,az;

        double tp,tr,ty;
        double vtp,vtr,vty;
        double atp,atr,aty;

        double weight;
    };

    struct esc_type {
        double dutycycle;
    };

    struct waypoint_type {
        double x,y,z;
    };

    struct fplan_type {
        double initial_altitude;
        waypoint_type waypoint[1000];
        int num_waypoints;
    };

    struct vector3D_type {
        double x,y,z;
    };

    struct vector3DP_type {
        double d,t1,t2;
    };

public:
    __attribute__((noinline,used)) void initialize_fmm();
    __attribute__((noinline,used)) void initialize_flightplan();
    __attribute__((noinline,used)) void save_flightplan(const char *fname);
    __attribute__((noinline,used)) int  load_flightplan(const char *fname);
    __attribute__((noinline,used)) StartDecision checkStart();
    __attribute__((noinline,used)) AltDecision checkAltitude();

    __attribute__((noinline,used)) void makevector3D(vector3D_type *v, double x, double y, double z);

    __attribute__((noinline,used)) void fmm_model(float timeslice);

    __attribute__((noinline,used)) void setmotor(int motor, double dutycycle);
    __attribute__((noinline,used)) void setmotors(double dutycycle);
    __attribute__((noinline,used)) void killmotors();
    __attribute__((noinline,used)) void idlemotors();
    __attribute__((noinline,used)) NeutralDecision neutralDecision();
    __attribute__((noinline,used)) void travelStep();
    __attribute__((noinline,used)) void descend();
    __attribute__((noinline,used)) void acquire();
    __attribute__((noinline,used)) void traveltowaypoint(int &wp_index, double wp_radius, double cruise_speed, double dt);

public:
    int curstate;
    int go_signal;
    long glng_counter;

    fmm_type fmm;
    curdrone_type curdrone;
    esc_type esc[8];

    fplan_type fplan;

    double batt_voltage;
    double batt_current;
    double batt_energy;
    double cur_batt_energy;
    int wp_index = 0;
    int done;

    vector3D_type tmp13D, tmp23D, tmp33D;
    vector3DP_type tmp13DP, tmp22DP, tmp33DP;
};

WCET_AT("wcet_ms_exact=0.010125; cycles=162");
void DroneController::initialize_fmm()
{
    TRACE_BEGIN(DR_INIT_FMM);

    // motor pod 1
    fmm.f1a=0.00; fmm.f1b=0.00;
    fmm.m1x=0.00; fmm.m1y=0.00; fmm.m1z=0.00;

    // motor pod 2
    fmm.f2a=0.00; fmm.f2b=0.00;
    fmm.m2x=0.00; fmm.m2y=0.00; fmm.m2z=0.00;

    // motor pod 3
    fmm.f3a=0.00; fmm.f3b=0.00;
    fmm.m3x=0.00; fmm.m3y=0.00; fmm.m3z=0.00;

    // motor pod 4
    fmm.f4a=0.00; fmm.f4b=0.00;
    fmm.m4x=0.00; fmm.m4y=0.00; fmm.m4z=0.00;

    // mass
    fmm.weight=0.00;

    // moments of inertia
    fmm.ip=0.00;
    fmm.iy=0.00;
    fmm.ir=0.00;

    // other var
    fmm.mfce=100.0;

    // initialize curdrone
   curdrone.x=0.0; curdrone.y=100.0; curdrone.z=0.0;
    curdrone.vx=0.0; curdrone.vy=0.0; curdrone.vz=0.0;
    curdrone.ax=0.0; curdrone.ay=0.0; curdrone.az=-9.8;
  
    curdrone.weight=fmm.weight;

    TRACE_END(DR_INIT_FMM);
}



WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void DroneController::initialize_flightplan()
{
    TRACE_BEGIN(DR_INIT_FLIGHTPLAN);

    fmm_model(0.01f);

    fplan.initial_altitude = 100.0;
    fplan.num_waypoints = 2;

    for (int a = 0; a < 1000; a++) {
        fplan.waypoint[a].x = 0.0;
        fplan.waypoint[a].y = 0.0;
        fplan.waypoint[a].z = 0.0;
    }

    fplan.waypoint[0].x = 0.0;
    fplan.waypoint[0].y = 100.0;
    fplan.waypoint[0].z = 0.0;

    fplan.waypoint[1].x = 0.0;
    fplan.waypoint[1].y = 100.0;
    fplan.waypoint[1].z = 0.0;

    TRACE_END(DR_INIT_FLIGHTPLAN);
}
  
WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void DroneController::save_flightplan(const char *fname)
{
    int a;
    FILE *fptr;

    fptr=fopen(fname,"w");
    if (!fptr) return;

    fprintf(fptr,"%2.2lf\n",fplan.initial_altitude);
    fprintf(fptr,"%d\n",fplan.num_waypoints);

    for (a=0;a<fplan.num_waypoints;a++)
        fprintf(fptr,"%2.2lf, %2.2lf, %2.2lf\n",
                fplan.waypoint[a].x,fplan.waypoint[a].y,fplan.waypoint[a].z);

    fclose(fptr);
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
int DroneController::load_flightplan(const char *fname)
{
    (void)fname;
    return 0;
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void DroneController::makevector3D(vector3D_type *v, double x, double y, double z)
{
    v->x=x; v->y=y; v->z=z;
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void DroneController::fmm_model(float timeslice)
{
    float tpm1a, tpm1b, tpm2a, tpm2b, tpm3a, tpm3b, tpm4a, tpm4b;
    float trm1a, trm1b, trm2a, trm2b, trm3a, trm3b, trm4a, trm4b;

    float daap;
    float daar;

    float fxm1a, fxm1b, fxm2a, fxm2b, fxm3a, fxm3b, fxm4a, fxm4b;
    float fym1a, fym1b, fym2a, fym2b, fym3a, fym3b, fym4a, fym4b;
    float fzm1a, fzm1b, fzm2a, fzm2b, fzm3a, fzm3b, fzm4a, fzm4b;
    float tff, daf;
    float tfu, dau;
    float tfr, dar;

    float fw;

    vector3D_type fm1a3D, fm1b3D, fm2a3D, fm2b3D, fm3a3D, fm3b3D, fm4a3D, fm4b3D;

    fw = (float)curdrone.weight * 9.8f;

    fmm.f1a = (float)esc[0].dutycycle * (float)fmm.mfce;
    fmm.f1b = (float)esc[1].dutycycle * (float)fmm.mfce;
    fmm.f2a = (float)esc[2].dutycycle * (float)fmm.mfce;
    fmm.f2b = (float)esc[3].dutycycle * (float)fmm.mfce;

    fmm.f3a = (float)esc[4].dutycycle * (float)fmm.mfce;
    fmm.f3b = (float)esc[5].dutycycle * (float)fmm.mfce;
    fmm.f4a = (float)esc[6].dutycycle * (float)fmm.mfce;
    fmm.f4b = (float)esc[7].dutycycle * (float)fmm.mfce;

    makevector3D(&fm1a3D, 0.0f, 0.0f, (float)fmm.f1a);
    makevector3D(&fm1b3D, 0.0f, 0.0f, (float)fmm.f1b);
    makevector3D(&fm2a3D, 0.0f, 0.0f, (float)fmm.f2a);
    makevector3D(&fm2b3D, 0.0f, 0.0f, (float)fmm.f2b);
    makevector3D(&fm3a3D, 0.0f, 0.0f, (float)fmm.f3a);
    makevector3D(&fm3b3D, 0.0f, 0.0f, (float)fmm.f3b);
    makevector3D(&fm4a3D, 0.0f, 0.0f, (float)fmm.f4a);
    makevector3D(&fm4b3D, 0.0f, 0.0f, (float)fmm.f4b);

    auto hypot2f = [](float a, float b) -> float { return sqrtf(a*a + b*b); };

    const float m1xy = hypot2f((float)fmm.m1x, (float)fmm.m1y);
    const float m1yz = hypot2f((float)fmm.m1y, (float)fmm.m1z);
    const float m2yz = hypot2f((float)fmm.m2y, (float)fmm.m2z);
    const float m3yz = hypot2f((float)fmm.m3y, (float)fmm.m3z);
    const float m4yz = hypot2f((float)fmm.m4y, (float)fmm.m4z);

    (void)m1xy; (void)m1yz; (void)m2yz; (void)m3yz; (void)m4yz;

    const float eps = 1e-6f;

    tpm1a = (float)fm1a3D.z * (float)fmm.m1x;
    tpm1b = (float)fm1b3D.z * (float)fmm.m1x;
    tpm2a = (float)fm2a3D.z * (float)fmm.m2x;
    tpm2b = (float)fm2b3D.z * (float)fmm.m2x;
    tpm3a = (float)fm3a3D.z * (float)fmm.m3x;
    tpm3b = (float)fm3b3D.z * (float)fmm.m3x;
    tpm4a = (float)fm4a3D.z * (float)fmm.m4x;
    tpm4b = (float)fm4b3D.z * (float)fmm.m4x;

    trm1a = (float)fm1a3D.z * (float)fmm.m1y;
    trm1b = (float)fm1b3D.z * (float)fmm.m1y;
    trm2a = (float)fm2a3D.z * (float)fmm.m2y;
    trm2b = (float)fm2b3D.z * (float)fmm.m2y;
    trm3a = (float)fm3a3D.z * (float)fmm.m3y;
    trm3b = (float)fm3b3D.z * (float)fmm.m3y;
    trm4a = (float)fm4a3D.z * (float)fmm.m4y;
    trm4b = (float)fm4b3D.z * (float)fmm.m4y;

    (void)tpm1b; (void)tpm2a; (void)tpm2b; (void)tpm3a; (void)tpm3b; (void)tpm4a; (void)tpm4b;
    (void)trm1a; (void)trm1b; (void)trm2a; (void)trm2b; (void)trm3a; (void)trm3b; (void)trm4a; (void)trm4b;

    daap = (8.0f * tpm1a) / ((float)fmm.ip + eps);
    daar = (8.0f * tpm1a) / ((float)fmm.ip + eps);

    curdrone.atp += daap;
    curdrone.atr += daar;

    curdrone.vtp += curdrone.atp * timeslice;
    curdrone.vtr += curdrone.atr * timeslice;

    curdrone.tp  += curdrone.vtp * timeslice;
    curdrone.tr  += curdrone.vtr * timeslice;

    const float tp = (float)curdrone.tp;
    const float tr = (float)curdrone.tr;
    const float ty = (float)curdrone.ty;

    const float sin_tp = sinf(tp);
    const float sin_tr = sinf(tr);
    const float cos_ty = cosf(ty);

    fxm1a = (float)fm1a3D.z * sin_tp;
    fxm1b = (float)fm1b3D.z * sin_tp;
    fxm2a = (float)fm2a3D.z * sin_tp;
    fxm2b = (float)fm2b3D.z * sin_tp;
    fxm3a = (float)fm3a3D.z * sin_tp;
    fxm3b = (float)fm3b3D.z * sin_tp;
    fxm4a = (float)fm4a3D.z * sin_tp;
    fxm4b = (float)fm4b3D.z * sin_tp;

    tff = fxm1a + fxm1b + fxm2a + fxm2b + fxm3a + fxm3b + fxm4a + fxm4b;
    daf = tff * (float)curdrone.weight;

    fym1a = (float)fm1a3D.z * sin_tr;
    fym1b = (float)fm1b3D.z * sin_tr;
    fym2a = (float)fm2a3D.z * sin_tr;
    fym2b = (float)fm2b3D.z * sin_tr;
    fym3a = (float)fm3a3D.z * sin_tr;
    fym3b = (float)fm3b3D.z * sin_tr;
    fym4a = (float)fm4a3D.z * sin_tr;
    fym4b = (float)fm4b3D.z * sin_tr;

    tfr = fym1a + fym1b + fym2a + fym2b + fym3a + fym3b + fym4a + fym4b;
    dar = tfr * (float)curdrone.weight;

    fzm1a = (float)fm1a3D.z * cos_ty;
    fzm1b = (float)fm1b3D.z * cos_ty;
    fzm2a = (float)fm2a3D.z * cos_ty;
    fzm2b = (float)fm2b3D.z * cos_ty;
    fzm3a = (float)fm3a3D.z * cos_ty;
    fzm3b = (float)fm3b3D.z * cos_ty;
    fzm4a = (float)fm4a3D.z * cos_ty;
    fzm4b = (float)fm4b3D.z * cos_ty;

    tfu = fzm1a + fzm1b + fzm2a + fzm2b + fzm3a + fzm3b + fzm4a + fzm4b - fw;
    dau = tfu * (float)curdrone.weight;

    curdrone.ax += daf;
    curdrone.ay += dar;
    curdrone.az += dau;

    curdrone.vx += curdrone.ax * timeslice;
    curdrone.vy += curdrone.ay * timeslice;
    curdrone.vz += curdrone.az * timeslice;

    curdrone.x  += curdrone.vx * timeslice;
    curdrone.y  += curdrone.vy * timeslice;
    curdrone.z  += curdrone.vz * timeslice;
     hw_delay_ms(9);
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void DroneController::setmotor(int motor, double dutycycle)
{
    esc[motor].dutycycle=dutycycle;
}

WCET_AT("wcet_ms_exact=0.002188; cycles=35");
void DroneController::setmotors(double dutycycle)
{
    TRACE_BEGIN(DR_SETMOTORS);

    esc[0].dutycycle=dutycycle;
    esc[1].dutycycle=dutycycle;
    esc[2].dutycycle=dutycycle;
    esc[3].dutycycle=dutycycle;
    esc[4].dutycycle=dutycycle;
    esc[5].dutycycle=dutycycle;
    esc[6].dutycycle=dutycycle;
    esc[7].dutycycle=dutycycle;

    TRACE_END(DR_SETMOTORS);
}

WCET_AT("wcet_ms_exact=9.014500; cycles=144232");
void DroneController::acquire()
{
    TRACE_BEGIN(DR_ACQUIRE);

   // setmotors(AQUIRE);
    hw_delay_ms(9);

    TRACE_END(DR_ACQUIRE);
}

WCET_AT("wcet_ms_exact=4.008438; cycles=64135");
void DroneController::killmotors()
{
    TRACE_BEGIN(DR_KILLMOTORS);

    //setmotors(0.0);
    hw_delay_ms(4);

    TRACE_END(DR_KILLMOTORS);
}

WCET_AT("wcet_ms_exact=5.010000; cycles=80160");
void DroneController::idlemotors()
{
    TRACE_BEGIN(DR_IDLEMOTORS);

    hw_delay_ms(5);
   // setmotors(IDLE_DC);

    TRACE_END(DR_IDLEMOTORS);
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void DroneController::traveltowaypoint(int &wp_index, double wp_radius, double cruise_speed, double dt)
{
    TRACE_BEGIN(DR_TRAVELTOWAYPOINT);

    waypoint_type wp = fplan.waypoint[wp_index];
    double d = dist3(curdrone.x,curdrone.y,curdrone.z, wp.x,wp.y,wp.z);

    if (d <= wp_radius) {
        wp_index++;
        if (wp_index < fplan.num_waypoints) curstate = NEUTRAL;
        else curstate = AQUIRE;

        TRACE_END(DR_TRAVELTOWAYPOINT);
        return;
    }

    double dx = wp.x - curdrone.x;
    double dy = wp.y - curdrone.y;
    double dz = wp.z - curdrone.z;
    double inv = 1.0 / d;

    curdrone.x += dx * inv * cruise_speed * dt;
    curdrone.y += dy * inv * cruise_speed * dt;
    curdrone.z += dz * inv * cruise_speed * dt;

    TRACE_END(DR_TRAVELTOWAYPOINT);
}

WCET_AT("wcet_ms_exact=0.000812; cycles=13");
NeutralDecision DroneController::neutralDecision()
{
    TRACE_BEGIN(DR_NEUTRAL_DECISION);

    NeutralDecision r;
    if (wp_index >= fplan.num_waypoints)
        r = LAND_NOW;
    else
        r = CONTINUE_MISSION;

    TRACE_END(DR_NEUTRAL_DECISION);
    return r;
}

WCET_AT("wcet_ms_exact=0.000500; cycles=8");
StartDecision DroneController::checkStart()
{
    TRACE_BEGIN(DR_CHECK_START);
    StartDecision r = (go_signal ? START_GO : WAIT_GO);
    TRACE_END(DR_CHECK_START);
    return r;
}

WCET_AT("wcet_ms_exact=0.001813; cycles=29");
AltDecision DroneController::checkAltitude()
{
    TRACE_BEGIN(DR_CHECK_ALTITUDE);
    AltDecision r = (curdrone.y >= fplan.initial_altitude) ? ALT_REACHED : ALT_NOT_REACHED;
    TRACE_END(DR_CHECK_ALTITUDE);
    return r;
}

WCET_AT("wcet_ms_exact=19.026500; cycles=304424");
void DroneController::descend()
{
    TRACE_BEGIN(DR_DESCEND);

   // setmotors(DESCEND_DC);
    hw_delay_ms(19);

    TRACE_END(DR_DESCEND);
}

void DroneController::travelStep()
{
    // no-op
}

// ============================================================
// Run 100 cycles, same style as traffic_trace_run_100
// ============================================================
static inline void drone_trace_run_100()
{
    rt_trace_reset();

    for (int iter = 0; iter < 100; ++iter) {
        DroneController drone;

        TRACE_BEGIN(DR_CYCLE);

        printf("Iteration %d\n", iter + 1);

        printf("Initializing drone mathematical model...");
        drone.initialize_fmm();
        printf("initialized.\n");

        printf("Initializing Flight Plan...");
        drone.initialize_flightplan();
        printf("Initialized.\n");

        drone.go_signal = 1;
  drone.wp_index = 0;
       

            if (drone.checkStart() == WAIT_GO) {
                drone.killmotors();
                printf("[OFF] waiting for GO\n");
                continue;
            }

            drone.idlemotors();

            if (drone.glng_counter == 0) {
                drone.glng_counter = 50;
            }
          //  drone.glng_counter--;

           // if (drone.glng_counter > 0) {
           //     printf("[ON] warmup... %ld\n", drone.glng_counter);
            //    continue;
          //  }
 if (drone.checkAltitude() == ALT_REACHED) {   // 6 -> 7
            drone.acquire();                           // 7 -> 8
            drone.traveltowaypoint(drone.wp_index, 1.0, 2.0, 0.1); // 8 -> 9

            if (drone.neutralDecision() == CONTINUE_MISSION) {     // 9 -> 7
                drone.acquire();                                    // 7 -> 8
                drone.traveltowaypoint(drone.wp_index, 1.0, 2.0, 0.1); // 8 -> 9

                if (drone.neutralDecision() == LAND_NOW) {         // 9 -> 10
                    drone.descend();                               // 10 -> 4
                }
            }
        }
    
         

     
        

        TRACE_END(DR_CYCLE);
    }
}

// ============================================================
// main
// ============================================================
int main()
{
    printf("DRONED traced run (100 iterations)\n");

    drone_trace_run_100();

    rt_trace_dump_pretty();
    rt_trace_dump_durations();
    rt_trace_dump_means();
    rt_trace_dump_csv_with_stddev("trace(DRONE)_results.csv");

    printf("\nDaemon shutting down...returning to OS\n\n");
    return 0;
}