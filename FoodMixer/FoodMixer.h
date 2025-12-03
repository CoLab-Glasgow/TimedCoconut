#define F_CPU 48000000UL
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>


#include "../in/hw_de_avr.h"

#ifdef WCET_ANNOTATE
#  undef WCET_ANNOTATE
#endif
#define WCET_ANNOTATE(...) 


#define WCET_CAT2(a,b) a##b
#define WCET_CAT(a,b)  WCET_CAT2(a,b)


#define WCET_AT(payload)                                                    \
  static const char __attribute__((used, section(".wcet_next")))              \
  WCET_CAT(__wcet_next_, __COUNTER__)[] = payload

#include "../include/Typestate_Library/TypestateLibrary_stub.h"
using namespace TypestateLibrary::Template;


class FoodMixer {
public:
  float tankLevel   = 0.0f;
  bool  valveStatus = false;
  __attribute__((noinline)) float ReadLevelSensor();
  __attribute__((noinline)) void  SetActuator(float percent);
  __attribute__((noinline)) float CalcFillPercent(float tank, float maxLevel);
};

WCET_AT("wcet_ms_exact=2.700813; cycles=43213; us_ceil=2701; F_CPU=16000000");
float FoodMixer::ReadLevelSensor() {
  
    _delay_ms(0.900f);
  return tankLevel;
}
         

WCET_AT("wcet_ms_exact=0.500; cycles=8; us_ceil=1; F_CPU=16000000");
float FoodMixer::CalcFillPercent(float tank, float maxLevel) {
    if (maxLevel <= 0.0f) return 0.0f;
 _delay_ms(0.900f);
return (tank / maxLevel) * 100.0f;

    return  100.0f;
}
WCET_AT("wcet_ms_exact=0.7250; cycles=148; us_ceil=10; F_CPU=16000000");
void FoodMixer::SetActuator(float percent) {
  if (percent < 30.0f) valveStatus = true;
  else if (percent > 50.0f) valveStatus = false;
  _delay_ms(0.725f);
  
}



enum class S {
   READ , COMPUTE, CONTROL
};

using namespace TypestateLibrary::Template;


using FoodMixer_Typestate = Typestate_Template<Cycle<ms(3)>,
Timed_State<S::READ,&FoodMixer::ReadLevelSensor,TimeGuard<ms(1),ms(1),Criticality::High,ms(0)>,S::COMPUTE>,
Timed_State<S::COMPUTE,&FoodMixer::CalcFillPercent,TimeGuard<ms(1), ms(1),Criticality::High,ms(1)>, S::CONTROL>,
Timed_State<S::CONTROL,&FoodMixer::SetActuator,TimeGuard<ms(1),ms(1),Criticality::High,ms(2)>, S::READ>
>;


using Tank_Flag = TypestateClassConnector<FoodMixer, FoodMixer_Typestate>;

static FoodMixer_Typestate R;
static Tank_Flag RW;

inline void init(){
  R.display();
 RW.display();
}
