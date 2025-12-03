// RobotSingle.h – ARM7 style, single system
#pragma once
#include "../include/Typestate_Library/TypestateLibrary_stub.h"
using namespace TypestateLibrary::Template;
#define F_CPU 16000000UL
#include "../in/hw_de.h"

#ifdef WCET_ANNOTATE
#  undef WCET_ANNOTATE
#endif
#define WCET_ANNOTATE(...) /


#define WCET_CAT2(a,b) a##b
#define WCET_CAT(a,b)  WCET_CAT2(a,b)


#define WCET_AT(payload)                                                    \
  static const char __attribute__((used, section(".wcet_next")))              \
  WCET_CAT(__wcet_next_, __COUNTER__)[] = payload
#include <stdlib.h>  // rand()
#include <stdio.h>   // printf
#include <math.h>    // sinf, cosf, tanf
enum class boolean { FALSE, TRUE };
class Robot {
public:
  // Data members
  float force_data[3] = {0.0f, 0.0f, 0.0f};
  float set_points[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float angle         = 0.0f;
   boolean  angle_changed = boolean::FALSE;

  // Methods (declarations only)
 __attribute__((noinline, used)) void  Read_force_data();
  __attribute__((noinline, used)) void  Read_path_angle();
  __attribute__((noinline, used)) float ComputePathAngle();
  __attribute__((noinline, used)) void  Compute_set_points_fast(float angle);
  __attribute__((noinline, used)) void  Compute_set_points_slow(float angle);
  __attribute__((noinline, used)) void  AdjustActuators(int n);
  __attribute__((noinline, used)) void  End();
};

WCET_AT("wcet_ms_exact=2.802313; cycles=44837; us_ceil=2803; F_CPU=16000000");
void Robot::Read_force_data() {
  // Simulate reading force data from sensors
  force_data[0] = 0.5f;
  force_data[1] = 0.3f;     
  force_data[2] = 0.8f;
  simulate_(SIM_UNITS_FROM_VAL(2.90f)); 
}

WCET_AT("wcet_ms_exact=2.807500; cycles=44920; us_ceil=2808; F_CPU=16000000");
void Robot::Read_path_angle() {
 // simulate reading path angle 
   angle = angle + 5.0f;
   simulate_(SIM_UNITS_FROM_VAL(2.90f)); 
  
}
WCET_AT("wcet_ms_exact=2.002562; cycles=32041; us_ceil=2003; F_CPU=16000000");
void Robot::AdjustActuators(int n) {


    if (n <= 0) {
        printf("Actuators adjusted.\n");
     
        return;            
    }
    
    int current_pos = 0.0;
    hw_delay_ms(2); 
    this->AdjustActuators(n - 1);   
   
}




WCET_AT("wcet_ms_exact=4.669187; cycles=74707; us_ceil=4670; F_CPU=16000000");
float Robot::ComputePathAngle() {
  simulate_(SIM_UNITS_FROM_VAL(4.6)); 
   return 0.546f+1;
}

// Slow path: recompute using angle as a simple gain
WCET_AT("wcet_ms_exact=4.023438; cycles=64375; us_ceil=4024; F_CPU=16000000");
void Robot::Compute_set_points_slow(float angle) {
  hw_delay_ms(4.0f);
  set_points[0] = force_data[0] + angle;
  set_points[1] = force_data[1] + angle;
  set_points[2] = force_data[2] + angle;
  set_points[3] = force_data[0] + angle;
  set_points[4] = force_data[1] + angle;
  set_points[5] = force_data[2] + angle;


}


WCET_AT("wcet_ms_exact=0.468438; cycles=7495; us_ceil=469; F_CPU=16000000");
void Robot::Compute_set_points_fast(float angle) {
 simulate_(SIM_UNITS_FROM_VAL(0.40f));
  set_points[0] = force_data[0];
  set_points[1] = force_data[1];
  set_points[2] = force_data[2];
  set_points[3] = force_data[0];
  set_points[4] = force_data[1];
  set_points[5] = force_data[2];

 
}
WCET_AT("wcet_ms_exact=0.000812; cycles=13; us_ceil=1; F_CPU=16000000");
void Robot::End() {
 hw_delay_ms(0.20f);
}



enum class RS {
  Idle,
  ForceRead,
  AngleRead,
  AngleComputed,
  Compute,
  ComputedSL,
  Computedfast,
  Adjusting
};





using R_Typestate = Typestate_Template<Cycle<ms(20)>,
    Timed_State<
RS::Idle,
        &Robot::Read_force_data,
        TimeGuard< 0, ms(3), Criticality::High, 0>,
        RS::ForceRead
    >,

    
    Timed_State<
        RS::ForceRead,
        &Robot::Read_path_angle,
        TimeGuard< 0, ms(3), Criticality::High, 0>,
        RS::AngleRead
    >,

   
  Timed_State<
        RS::AngleRead,
        &Robot::ComputePathAngle,
        TimeGuard< 0, ms(5), Criticality::High, 0>,
        RS::AngleComputed
    >,

      Timed_State<RS::AngleComputed, &Robot::Compute_set_points_slow,TimeGuard< 0, ms(0), Criticality::Soft,0>, RS::Adjusting>,
 Timed_State<RS::AngleComputed,&Robot::Compute_set_points_fast, TimeGuard< 0, ms(0), Criticality::Soft,0>, RS::Adjusting>,
  
  
    Timed_State<RS::Adjusting,&Robot::AdjustActuators,TimeGuard< 0, ms(5), Criticality::High,0>, RS::Adjusting>,
   Timed_State<RS::Adjusting,&Robot::End,TimeGuard<0, ms(3), Criticality::High,0>, RS::Idle>
>;



using Robot_flag = TypestateClassConnector<Robot, R_Typestate>;
//[[gnu::used]] static Tank_Flag_ARM __ts_force_connector;


//[[gnu::used]] 
static Robot_flag          RW_ARM;

// (optional) 
static inline void init_typestate_arm() {
  // R_ARM.display(); 
  RW_ARM.display();
}
