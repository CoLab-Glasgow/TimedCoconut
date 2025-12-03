#include <cstdint>
#include <cstdio>
#include "../hw_de.h"


#include "../include/Typestate_Library/TypestateLibrary_stub.h"
using namespace TypestateLibrary::Template;


#ifdef WCET_ANNOTATE
#  undef WCET_ANNOTATE
#endif
#define WCET_ANNOTATE(...) /* disabled: using #pragma wcet next */

//  WCET marker macro 
#define WCET_CAT2(a,b) a##b
#define WCET_CAT(a,b)  WCET_CAT2(a,b)


#define WCET_AT(payload)                                                    \
  static const char __attribute__((used, section(".wcet_next")))              \
  WCET_CAT(__wcet_next_, __COUNTER__)[] = payload


#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef PCLK_HZ
#define PCLK_HZ F_CPU
#endif




class FoodMixer {
public:
  float tankLevel   = 0.0f;
  bool  valveStatus = false;
  __attribute__((noinline)) float ReadLevelSensor();
  __attribute__((noinline)) float CalcFillPercent(float tank, float maxLevel);
  __attribute__((noinline)) void  SetActuator(float percent);
  
};

WCET_AT("wcet_ms_exact=0.934187; cycles=14947; us_ceil=935; F_CPU=16000000");
float FoodMixer::ReadLevelSensor() {
simulate_(SIM_UNITS_FROM_VAL(1.00f));  

  return tankLevel;
}
   
WCET_AT("wcet_ms_exact=0.939125; cycles=15026; us_ceil=940; F_CPU=16000000");
float FoodMixer::CalcFillPercent(float tank, float maxLevel) {
    if (maxLevel <= 0.0f) return 0.0f;
  simulate_(SIM_UNITS_FROM_VAL(1.00f));
return (tank / maxLevel) * 100.0f;
}

WCET_AT("wcet_ms_exact=0.711250; cycles=11380; us_ceil=712; F_CPU=16000000");
void FoodMixer::SetActuator(float percent) {
  if (percent < 30.0f) valveStatus = true;
  else if (percent > 50.0f) valveStatus = false;
   simulate_(SIM_UNITS_FROM_VAL(0.80f));
}




enum class S {READ, COMPUTE, CONTROL };

using FoodMixer_Typestate = Typestate_Template<Cycle<ms(3)>,
Timed_State<S::READ,&FoodMixer::ReadLevelSensor,TimeGuard<ms(1),ms(1),Criticality::Soft,ms(0)>,S::COMPUTE>,
Timed_State<S::COMPUTE,&FoodMixer::CalcFillPercent,TimeGuard<ms(1), ms(1),Criticality::High,ms(1)>, S::CONTROL>,
Timed_State<S::CONTROL,&FoodMixer::SetActuator,TimeGuard<ms(1),ms(0.8),Criticality::High,ms(2)>, S::READ>
>;




using Tank_Flag_ARM = TypestateClassConnector<FoodMixer, FoodMixer_Typestate>;
[[gnu::used]] static Tank_Flag_ARM __ts_force_connector;


//[[gnu::used]] 
static Tank_Flag_ARM          RW_ARM;

// (optional) 
static inline void init_typestate_arm() {
  // R_ARM.display(); 
  RW_ARM.display();
}
