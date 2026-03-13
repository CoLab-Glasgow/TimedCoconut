#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define F_CPU 16000000UL
#include "../hw_de.h"

#ifdef WCET_ANNOTATE
#  undef WCET_ANNOTATE
#endif
#define WCET_ANNOTATE(...) /


#define WCET_CAT2(a,b) a##b
#define WCET_CAT(a,b)  WCET_CAT2(a,b)


#define WCET_AT(payload)                                                    \
  static const char __attribute__((used, section(".wcet_next")))              \
  WCET_CAT(__wcet_next_, __COUNTER__)[] = payload
// Stanford Systems graphics support (keep as-is if you need it)
extern "C" {
//#include "ssmotor_graph3.c"
}

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
// state machine constants
#define OFF     0
#define ON      10
#define ASCEND  20
#define AQUIRE  30
#define TRAVEL  40
#define NEUTRAL 50
#define DESCEND 60

// common duty cycles
#define OFF_DC     0.00
#define IDLE_DC    0.10
#define ASCEND_DC  0.30
#define DESCEND_DC 0.20

class DroneController {
public:
    // --- model types (kept the same, just nested) ---
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

    // Optional: zero ESCs
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
  

    // “looks like TrafficLight”: methods declared in the class
  __attribute__((noinline,used))  void initialize_fmm();
  __attribute__((noinline,used))  void initialize_flightplan();
  __attribute__((noinline,used))  void save_flightplan(const char *fname);
   __attribute__((noinline,used)) int  load_flightplan(const char *fname);
  __attribute__((noinline,used))  StartDecision checkStart();
  __attribute__((noinline,used))  AltDecision checkAltitude();

  __attribute__((noinline,used))  void makevector3D(vector3D_type *v, double x, double y, double z);

   __attribute__((noinline,used)) void fmm_model(float timeslice);

   __attribute__((noinline,used)) void setmotor(int motor, double dutycycle);
    __attribute__((noinline,used)) void setmotors(double dutycycle);
   __attribute__((noinline,used))  void killmotors();
   __attribute__((noinline,used)) void idlemotors();
   __attribute__((noinline,used)) NeutralDecision neutralDecision();
   __attribute__((noinline,used)) void travelStep();
   __attribute__((noinline,used)) void descend();
     __attribute__((noinline,used)) void acquire();
    // NOTE: depends on your loop/state machine variables, so we pass what it needs
   __attribute__((noinline,used)) void traveltowaypoint(int &wp_index, double wp_radius, double cruise_speed, double dt);

public:
    // --- the old globals become class members (same names) ---
    int curstate;
    int go_signal;
    long glng_counter;

    fmm_type fmm;
    curdrone_type curdrone;
    esc_type esc[8];

    fplan_type fplan;

    // general globals
    double batt_voltage;
    double batt_current;
    double batt_energy;
    double cur_batt_energy;
   int wp_index = 0;
    int done;

    // temp vectors (kept as members instead of globals)
    vector3D_type tmp13D, tmp23D, tmp33D;
    vector3DP_type tmp13DP, tmp22DP, tmp33DP;
};




WCET_AT("wcet_ms_exact=0.010125; cycles=162");
void DroneController::initialize_fmm()
{
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
    curdrone.x=0.0; curdrone.y=0.0; curdrone.z=0.0;
    curdrone.vx=0.0; curdrone.vy=0.0; curdrone.vz=0.0;
    curdrone.ax=0.0; curdrone.ay=0.0; curdrone.az=-9.8;

    curdrone.weight=fmm.weight;
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void DroneController::initialize_flightplan()
{

    fmm_model(0.01f); // assuming 10ms timeslice
    int a;
    double  fp1[][3] = {
        {0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},
        {0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}
    };

    fplan.initial_altitude=100.0;
    fplan.num_waypoints = 10;

    for (a=0;a<1000;a++) {
        fplan.waypoint[a].x=0.00;
        fplan.waypoint[a].y=0.0;
        fplan.waypoint[a].z=0.0;
    }

    for (a=0;a<fplan.num_waypoints;a++) {
        fplan.waypoint[a].x=fp1[a][0];
        fplan.waypoint[a].y=fp1[a][1];
        fplan.waypoint[a].z=fp1[a][2];
    }

   // save_flightplan("default.dfp");
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
    // code later
    return 0;
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void DroneController::makevector3D(vector3D_type *v, double x, double y, double z)
{
    v->x=x; v->y=y; v->z=z;
}

// If you can, also make curdrone/fmm fields float to avoid float<->double conversions.
// At minimum: keep locals float and use float math functions.
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

    // Forces (make sure fmm.mfce and esc[].dutycycle are float if possible)
    fmm.f1a = (float)esc[0].dutycycle * (float)fmm.mfce;
    fmm.f1b = (float)esc[1].dutycycle * (float)fmm.mfce;
    fmm.f2a = (float)esc[2].dutycycle * (float)fmm.mfce;
    fmm.f2b = (float)esc[3].dutycycle * (float)fmm.mfce;

    // BUGFIX in your original: you assigned f3a twice and f4a twice.
    fmm.f3a = (float)esc[4].dutycycle * (float)fmm.mfce;
    fmm.f3b = (float)esc[5].dutycycle * (float)fmm.mfce;
    fmm.f4a = (float)esc[6].dutycycle * (float)fmm.mfce;
    fmm.f4b = (float)esc[7].dutycycle * (float)fmm.mfce;

    // If makevector3D expects double, consider creating makevector3Df(...) to keep everything float.
    makevector3D(&fm1a3D, 0.0f, 0.0f, (float)fmm.f1a);
    makevector3D(&fm1b3D, 0.0f, 0.0f, (float)fmm.f1b);
    makevector3D(&fm2a3D, 0.0f, 0.0f, (float)fmm.f2a);
    makevector3D(&fm2b3D, 0.0f, 0.0f, (float)fmm.f2b);
    makevector3D(&fm3a3D, 0.0f, 0.0f, (float)fmm.f3a);
    makevector3D(&fm3b3D, 0.0f, 0.0f, (float)fmm.f3b);
    makevector3D(&fm4a3D, 0.0f, 0.0f, (float)fmm.f4a);
    makevector3D(&fm4b3D, 0.0f, 0.0f, (float)fmm.f4b);

    // Replace pow(pow(a,2)+pow(b,2),0.5) with sqrtf(a*a + b*b)
    // Replace cos(atan(y/x)) with x / sqrt(x*x + y*y) (more stable + cheaper)

    auto hypot2f = [](float a, float b) -> float { return sqrtf(a*a + b*b); };

    const float m1xy = hypot2f((float)fmm.m1x, (float)fmm.m1y);
    const float m1yz = hypot2f((float)fmm.m1y, (float)fmm.m1z);
    const float m2yz = hypot2f((float)fmm.m2y, (float)fmm.m2z);
    const float m3yz = hypot2f((float)fmm.m3y, (float)fmm.m3z);
    const float m4yz = hypot2f((float)fmm.m4y, (float)fmm.m4z);

    // Avoid divide-by-zero
    const float eps = 1e-6f;

    // cos(atan(y/x)) * sqrt(x^2+y^2) simplifies to x   (when using the same x,y pair)
    // Because cos(atan(y/x)) = x / sqrt(x^2+y^2). Multiply by sqrt(x^2+y^2) => x.
    // Your expression: fm?.z * cos(atan(m1y/m1x)) * sqrt(m1x^2+m1y^2) => fm?.z * m1x
    // So all these tpm terms can be simplified massively:
    tpm1a = (float)fm1a3D.z * (float)fmm.m1x;
    tpm1b = (float)fm1b3D.z * (float)fmm.m1x;
    tpm2a = (float)fm2a3D.z * (float)fmm.m2x;   // (you probably intended m2x here)
    tpm2b = (float)fm2b3D.z * (float)fmm.m2x;
    tpm3a = (float)fm3a3D.z * (float)fmm.m3x;
    tpm3b = (float)fm3b3D.z * (float)fmm.m3x;
    tpm4a = (float)fm4a3D.z * (float)fmm.m4x;
    tpm4b = (float)fm4b3D.z * (float)fmm.m4x;

    // For trm terms: fm?.z * cos(atan(z/y)) * sqrt(z^2+y^2) simplifies to fm?.z * y
    // since cos(atan(z/y)) = y / sqrt(y^2+z^2).
    trm1a = (float)fm1a3D.z * (float)fmm.m1y;
    trm1b = (float)fm1b3D.z * (float)fmm.m1y;
    trm2a = (float)fm2a3D.z * (float)fmm.m2y;
    trm2b = (float)fm2b3D.z * (float)fmm.m2y;
    trm3a = (float)fm3a3D.z * (float)fmm.m3y;
    trm3b = (float)fm3b3D.z * (float)fmm.m3y;
    trm4a = (float)fm4a3D.z * (float)fmm.m4y;
    trm4b = (float)fm4b3D.z * (float)fmm.m4y;

    // Your original daap/daar sums add the same term 8 times. Keep behavior:
    daap = (8.0f * tpm1a) / ((float)fmm.ip + eps);
    daar = (8.0f * tpm1a) / ((float)fmm.ip + eps);

    curdrone.atp += daap;
    curdrone.atr += daar;

    curdrone.vtp += curdrone.atp * timeslice;
    curdrone.vtr += curdrone.atr * timeslice;

    curdrone.tp  += curdrone.vtp * timeslice;
    curdrone.tr  += curdrone.vtr * timeslice;

    // Use sinf/cosf (float versions)
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

    hw_delay_ms(5);
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void DroneController::setmotor(int motor, double dutycycle)
{
    esc[motor].dutycycle=dutycycle;
}

WCET_AT("wcet_ms_exact=0.002188; cycles=35");
void DroneController::setmotors(double dutycycle)
{
    esc[0].dutycycle=dutycycle;
    esc[1].dutycycle=dutycycle;
    esc[2].dutycycle=dutycycle;
    esc[3].dutycycle=dutycycle;
    esc[4].dutycycle=dutycycle;
    esc[5].dutycycle=dutycycle;
    esc[6].dutycycle=dutycycle;
    esc[7].dutycycle=dutycycle;
   
    
}

WCET_AT("wcet_ms_exact=9.014500; cycles=144232");
void DroneController::acquire(){
    setmotors(AQUIRE);
    hw_delay_ms(9);
}

WCET_AT("wcet_ms_exact=4.008438; cycles=64135");
void DroneController::killmotors()
{
    setmotors(0.0);
    hw_delay_ms(4);
}

WCET_AT("wcet_ms_exact=5.010000; cycles=80160");
void DroneController::idlemotors()
{
   hw_delay_ms(5); 
    setmotors(IDLE_DC);
}



WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void DroneController::traveltowaypoint(int &wp_index, double wp_radius, double cruise_speed, double dt)
{
    waypoint_type wp = fplan.waypoint[wp_index];
    double d = dist3(curdrone.x,curdrone.y,curdrone.z, wp.x,wp.y,wp.z);

    if (d <= wp_radius) {
        wp_index++;
        if (wp_index >= fplan.num_waypoints) curstate = NEUTRAL;
        else curstate = AQUIRE;
        return;
    }

    double dx = wp.x - curdrone.x;
    double dy = wp.y - curdrone.y;
    double dz = wp.z - curdrone.z;
    double inv = 1.0 / d;

    curdrone.x += dx * inv * cruise_speed * dt;
    curdrone.y += dy * inv * cruise_speed * dt;
    curdrone.z += dz * inv * cruise_speed * dt;

   
}


WCET_AT("wcet_ms_exact=0.000812; cycles=13");
NeutralDecision DroneController::neutralDecision()


{ 
    if (wp_index >= fplan.num_waypoints)
        return LAND_NOW;

    return CONTINUE_MISSION;
}


WCET_AT("wcet_ms_exact=0.000500; cycles=8");
StartDecision DroneController::checkStart()
{
    return (go_signal ? START_GO : WAIT_GO);
}



WCET_AT("wcet_ms_exact=0.001813; cycles=29");
AltDecision DroneController::checkAltitude()
{
  
    return (curdrone.y >= fplan.initial_altitude) ? ALT_REACHED : ALT_NOT_REACHED;
}



WCET_AT("wcet_ms_exact=19.026500; cycles=304424");
 void DroneController::descend() { setmotors(DESCEND_DC); hw_delay_ms(19); }



enum DS {
    INIT_MODEL_S,//0
    INIT_PLAN_S,//1
    StartDecision_S,//2
    Check,//3 
    OFF_S,//4
    ON_S, //5
    ASCEND_S,//6
    AQUIRE_S, //7
    TRAVEL_S,//8
    NEUTRAL_S,//9
    DESCEND_S//10
};


#include "../include/Typestate_Library/TypestateLibrary_stub.h"
using namespace TypestateLibrary::Template;

using Drone_Typestate = Typestate_Template<

    Cycle<ms(60) >, // model update cycle (not a real-time constraint, just a demo periodic task)

    Timed_State<
        DS::INIT_MODEL_S,
        &DroneController::initialize_fmm,
        TimeGuard<0,0, lower(ms(0)),upper(ms(0)), Criticality::Firm>,
        DS::INIT_PLAN_S
    >,

    Timed_State<
        DS::INIT_PLAN_S,
        &DroneController::initialize_flightplan,
        TimeGuard<0,0, lower(ms(0)), upper(ms(10)), Criticality::High>,
        DS::Check
    >,

    
    
    Timed_Branch_State<
        DS::Check,
        &DroneController::checkStart,
        TimeGuard<0,0, lower(ms(0)), upper(ms(1)), Criticality::High>,
        ChoiceList<
            Choice<WAIT_GO,  DS::OFF_S>,
            Choice<START_GO, DS::ON_S>
        >
    >,
    

    
    Timed_State<
        DS::ON_S,
        &DroneController::idlemotors,
        TimeGuard<0,0, lower(ms(5)), upper(ms(10)), Criticality::High>,
        DS::ASCEND_S
    >,

    
    Timed_Branch_State<
        DS::ASCEND_S,
        &DroneController::checkAltitude,
        TimeGuard<0,0, lower(ms(0)), upper(ms(1)), Criticality::High>,
        ChoiceList<
            Choice<ALT_NOT_REACHED, DS::ASCEND_S>,
            Choice<ALT_REACHED,     DS::AQUIRE_S>
        >
    >,

     Timed_State<
        DS::ASCEND_S,
        &DroneController::setmotors,
        TimeGuard<0,0, lower(ms(0)), upper(ms(1)), Criticality::High>, DS::ASCEND_S>,

    
    Timed_State<
        DS::AQUIRE_S,
        &DroneController::acquire,
        TimeGuard<0,0, lower(ms(0)), upper(ms(10)), Criticality::High>,
        DS::TRAVEL_S
    >,

    
    Timed_State<
        DS::TRAVEL_S,
        &DroneController::traveltowaypoint,
        TimeGuard<0,0, lower(ms(10)), upper(ms(30)), Criticality::High>,
        DS::NEUTRAL_S
    >,

    
    Timed_Branch_State<
        DS::NEUTRAL_S,
        &DroneController::neutralDecision,
        TimeGuard<0,0, lower(ms(0)), upper(ms(1)), Criticality::High>,
        ChoiceList<
            Choice<CONTINUE_MISSION, DS::AQUIRE_S>,
            Choice<LAND_NOW,         DS::DESCEND_S>
        >
    >,

    // -------------------------------------------------
    // DESCEND — land, then return to OFF
    // -------------------------------------------------

    Timed_State<
        DS::DESCEND_S,
        &DroneController::descend,
        TimeGuard<0,0, lower(ms(0)), upper(ms(20)), Criticality::High>,
        DS::OFF_S
    >,
    Timed_State<
        DS::OFF_S,
        &DroneController::killmotors,
        TimeGuard<0,0, lower(ms(1)), upper(ms(10)), Criticality::High>,DS::OFF_S
    >
>;
using Drone_flag = TypestateClassConnector<DroneController, Drone_Typestate>;
static Drone_flag RW_ARM;

static inline void init_typestate_arm() {
  RW_ARM.display();
}
