// RobotSingle.h – ARM7 style, single system
#pragma once
#include "../include/Typestate_Library/TypestateLibrary_stub.h"
using namespace TypestateLibrary::Template;
#define F_CPU 16000000UL
#include "../hw_de.h"
#include "MiniPID.cpp"
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


#define T1 20.0 /* period for force (20 ms) */
#define T2 80.0 /* period for vision (80 ms) */
#define T3 28.0 /* period for control (28 ms) */


// Robot-Arm.h (single header version)
// Adds missing methods: ComputePathAngle() and ApplyModePIDConfig(Mode)
// Keeps your internal-object model and PID pipeline
#pragma once

#include <stdint.h>
#include <math.h>
#include "MiniPID.h"   // include header; compile MiniPID.cpp separately in real builds

enum Mode { Slow, Fast };

class Robot {
public:
  float obj_x = 0.0f;     
  float obj_y = 0.0f;      
  float obj_vx = 0.40f;   
  float obj_vy = 0.00f;    
  bool  obj_present = true;
  float pick_x = 1.2f;
  float pick_window = 0.05f;
  bool  gripper_closed = false;
  float force_data[3] = {0,0,0};
  float angle = 0.0f;
  float last_angle = 0.0f;
  bool  angle_changed = false;
  Mode mode = Fast;            // current mode
  Mode pending_mode = Fast;    // candidate next mode
  bool mode_changed = false;
  uint8_t slow_votes = 0;
  uint8_t fast_votes = 0;
  uint8_t holdoff_ticks = 0;
  double actual[6] = {0,0,0,0,0,0};
  double target_q[6] = {0,0,0,0,0,0};
  double command[6] = {0,0,0,0,0,0};
  // PIDs...
  MiniPID pid0 = MiniPID(0.8, 0.02, 0.10);
  MiniPID pid1 = MiniPID(0.8, 0.02, 0.10);
  MiniPID pid2 = MiniPID(0.8, 0.02, 0.10);
  MiniPID pid3 = MiniPID(0.8, 0.02, 0.10);
  MiniPID pid4 = MiniPID(0.8, 0.02, 0.10);
  MiniPID pid5 = MiniPID(0.8, 0.02, 0.10);
  __attribute__((noinline, used)) void InitPID();
  __attribute__((noinline, used) )void Read_force_data();
  __attribute__((noinline, used)) void Read_path_angle();       
  __attribute__((noinline, used)) Mode ConfigMode();            
  __attribute__((noinline, used)) Mode ComputePathAngle();      
  __attribute__((noinline, used)) void ApplyModePIDConfig(Mode m);
  __attribute__((noinline, used)) void ConfigurePIDForMode();   // applies pending_mode if mode_changed
  __attribute__((noinline, used)) void Compute_targets_fast();
  __attribute__((noinline, used)) void Compute_targets_slow();
  __attribute__((noinline, used)) void ComputePID();
  __attribute__((noinline, used)) void ApplyActuators();
  __attribute__((noinline, used)) void End();
};


WCET_AT("wcet_ms_exact=0.141813; cycles=2269");
 void Robot::InitPID() {
  MiniPID* P[6] = {&pid0,&pid1,&pid2,&pid3,&pid4,&pid5};
  const double outMin = -1.0, outMax = 1.0;
  for (int i=0;i<6;i++){
    P[i]->setOutputLimits(outMin, outMax);
    P[i]->setMaxIOutput(0.25);
    P[i]->setOutputRampRate(0.05);
    P[i]->setOutputFilter(0.10);
    P[i]->setSetpointRange(2.0);
    P[i]->setDirection(false);
    P[i]->reset();
  }
  mode = Fast;
  pending_mode = Fast;
  mode_changed = false;
  slow_votes = fast_votes = 0;
  holdoff_ticks = 0;
  angle = last_angle = 0.0f;
  angle_changed = false;
}



WCET_AT("wcet_ms_exact=0.817750; cycles=13084");
 void Robot::Read_path_angle() {
  last_angle = angle;
  angle = obj_present ? atan2f(obj_vy, obj_vx) : 0.0f;
  angle_changed = (fabsf(angle - last_angle) > 0.15f);
  simulate_0_8ms();
}

WCET_AT("wcet_ms_exact=0.815750; cycles=13052");
 void Robot::Read_force_data() {
  force_data[0] = 0.0f;
  force_data[1] = 0.0f;
  force_data[2] = 0.0f;
  if (obj_present && gripper_closed) {
    force_data[0] = 0.6f;
    force_data[1] = 0.4f;
    force_data[2] = 0.8f;
  }
simulate_0_8ms();
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
 Mode Robot::ConfigMode() {
  return pending_mode;
}


WCET_AT("wcet_ms_exact=0.004250; cycles=68");
 Mode Robot::ComputePathAngle() {
  if (holdoff_ticks > 0) holdoff_ticks--;
  const bool changed = angle_changed;
  if (changed) { if (slow_votes < 255) slow_votes++; fast_votes = 0; }
  else         { if (fast_votes < 255) fast_votes++; slow_votes = 0; }
  pending_mode = mode;
  mode_changed = false;

  if (holdoff_ticks == 0) {
    if (mode == Fast && slow_votes >= 3) {         
      pending_mode = Slow;
      mode_changed = true;
      holdoff_ticks = 10;                          
    } else if (mode == Slow && fast_votes >= 5) {  
      pending_mode = Fast;
      mode_changed = true;
      holdoff_ticks = 10;
    }
  }

  return pending_mode;
  
}


WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
 void Robot::ApplyModePIDConfig(Mode m) {
  pending_mode = m;
  mode_changed = (pending_mode != mode);
  ConfigurePIDForMode();
}

WCET_AT("wcet_ms_exact=0.948562; cycles=15177");
 void Robot::ConfigurePIDForMode() {
  if (!mode_changed) return; 

  MiniPID* P[6] = {&pid0,&pid1,&pid2,&pid3,&pid4,&pid5};

  if (pending_mode == Fast) {
    for (int i=0;i<6;i++){
      P[i]->setPID(1.0, 0.02, 0.08);
      P[i]->setOutputRampRate(0.08);
      P[i]->setOutputFilter(0.05);
      P[i]->setMaxIOutput(0.30);
      P[i]->reset(); 
    }
  } else { 
    for (int i=0;i<6;i++){
      P[i]->setPID(0.6, 0.03, 0.12);
      P[i]->setOutputRampRate(0.03);
      P[i]->setOutputFilter(0.15);
      P[i]->setMaxIOutput(0.20);
      P[i]->reset();
    }
  }

  mode = pending_mode;
  mode_changed = false;
   simulate_0_8ms();

}


WCET_AT("wcet_ms_exact=0.005000; cycles=80");
 void Robot::Compute_targets_fast() {
 target_q[0] = force_data[0];
target_q[1] = force_data[1];
target_q[2] = force_data[2];
target_q[3] = force_data[0];
target_q[4] = force_data[1];
target_q[5] = force_data[2];
}

WCET_AT("wcet_ms_exact=0.820500; cycles=13128");
 void Robot::Compute_targets_slow() {
target_q[0] = force_data[0] + angle;
target_q[1] = force_data[1] + angle;
target_q[2] = force_data[2] + angle;
target_q[3] = force_data[0] + angle;
target_q[4] = force_data[1] + angle;  
target_q[5] = force_data[2] + angle;
simulate_0_8ms();
}

WCET_AT("wcet_ms_exact=0.297500; cycles=4760");
 void Robot::ComputePID() {
  command[0] = pid0.getOutput(actual[0], target_q[0]);
  command[1] = pid1.getOutput(actual[1], target_q[1]);
  command[2] = pid2.getOutput(actual[2], target_q[2]);
  command[3] = pid3.getOutput(actual[3], target_q[3]);
  command[4] = pid4.getOutput(actual[4], target_q[4]);
  command[5] = pid5.getOutput(actual[5], target_q[5]);
}

WCET_AT("wcet_ms_exact=1.019500; cycles=16312");
void Robot::ApplyActuators() {
  actual[0] += 0.2f * (target_q[0] - actual[0]);
   actual[1] += 0.2f * (target_q[1] - actual[1]);
    actual[2] += 0.2f * (target_q[2] - actual[2]);
     actual[3] += 0.2f * (target_q[3] - actual[3]);
      actual[4] += 0.2f * (target_q[4] - actual[4]);
      actual[5] += 0.2f * (target_q[5] - actual[5]);
      hw_delay_ms(1);
  
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
 void Robot::End() { /* end-of-cycle */ }

static constexpr int CONTROL_PERIOD_MS = 20;

enum class RS {
  Init,
  ReadForce,
  ReadAngle,
  DecideMode,       // returns Mode and sets pending_mode/mode_changed
  ConfigureMode,    // applies PID config if mode_changed
  DispatchCompute,  // branch based on CURRENT mode
  ComputeFast,
  ComputeSlow,
  ComputePID,
  Apply,
  End
};

using R_Typestate = Typestate_Template<
  Cycle<ms(11)>,
  Timed_State<
    RS::Init,
    &Robot::InitPID,
    TimeGuard<0,0,lower(ms(0)), upper(ms(1)), Criticality::High>,
    RS::ReadForce
  >,
  Timed_State<
    RS::ReadForce,
    &Robot::Read_force_data,
    TimeGuard<0,0,lower(ms(0)), upper(ms(2)), Criticality::High>,
    RS::ReadAngle
  >,
  Timed_State<
    RS::ReadAngle,
    &Robot::Read_path_angle,
    TimeGuard<0,0,lower(ms(0)), upper(ms(1)), Criticality::High>,
    RS::DecideMode
  >,
  Timed_State<
    RS::DecideMode,
    &Robot::ComputePathAngle, // wrapper that calls DecideMode()
    TimeGuard<0,0,lower(ms(0)), upper(ms(1)), Criticality::High>,
    RS::ConfigureMode
  >,

  Timed_State<
    RS::ConfigureMode,
    &Robot::ConfigurePIDForMode,
    TimeGuard<0,0,lower(ms(0)), upper(ms(1)), Criticality::High>,
    RS::DispatchCompute
  >,

  Timed_Branch_State<
    RS::DispatchCompute,
    &Robot::ConfigMode,
    TimeGuard<0,0,lower(ms(0)), upper(ms(0)), Criticality::High>,
    ChoiceList<
      Choice<Mode::Fast, RS::ComputeFast>,
      Choice<Mode::Slow, RS::ComputeSlow>
    >
  >,
  Timed_State<
    RS::ComputeFast,
    &Robot::Compute_targets_fast,
    TimeGuard<0,0,lower(ms(0)), upper(ms(0)), Criticality::High>,
    RS::ComputePID
  >,

  Timed_State<
    RS::ComputeSlow,
    &Robot::Compute_targets_slow,
    TimeGuard<0,0,lower(ms(0)), upper(ms(1)), Criticality::High>,
    RS::ComputePID
  >,

  // 7) PID control computation (motor commands)
  Timed_State<
    RS::ComputePID,
    &Robot::ComputePID,
    TimeGuard<0,0,lower(ms(0)), upper(ms(1)), Criticality::High>,
    RS::Apply
  >,

  // 8) Apply commands
  Timed_State<
    RS::Apply,
    &Robot::ApplyActuators,
    TimeGuard<0,0,lower(ms(1)), upper(ms(2)), Criticality::High>,
    RS::End
  >,

  // 9) End of cycle
  Timed_State<
    RS::End,
    &Robot::End,
    TimeGuard<0,0,lower(ms(0)), upper(ms(1)), Criticality::High>,
    RS::ReadForce
  >
>;


using Robot_flag = TypestateClassConnector<Robot, R_Typestate>;
static Robot_flag RW_ARM;

static inline void init_typestate_arm() {
  RW_ARM.display();
}
