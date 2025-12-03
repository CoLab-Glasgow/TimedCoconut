#pragma once
#include "../include/Typestate_Library/TypestateLibrary_stub.h"
using namespace TypestateLibrary::Template;
#define F_CPU 16000000UL
#include "../in/hw_de.h"

#ifdef WCET_ANNOTATE
#  undef WCET_ANNOTATE
#endif
#define WCET_ANNOTATE(...) 


#define WCET_CAT2(a,b) a##b
#define WCET_CAT(a,b)  WCET_CAT2(a,b)


#define WCET_AT(payload)                                                    \
  static const char __attribute__((used, section(".wcet_next")))              \
  WCET_CAT(__wcet_next_, __COUNTER__)[] = payload
#include <stdlib.h>  // rand()
#include <stdio.h>   // printf
#include <math.h>    // sinf, cosf, tanf


class TrafficLight {
public:
__attribute__((noinline,used))
    void SwitchToRed();
   __attribute__((noinline, used)) 
    void SwitchToGreen() ;
__attribute__((noinline, used))
    void SwitchToAmber();

};

WCET_AT("wcet_ms_exact=14.413375; cycles=470614; us_ceil=29414; F_CPU=16000000");
void TrafficLight::SwitchToAmber(){
    simulate_(SIM_UNITS_FROM_VAL(15.0f));
    }
WCET_AT("wcet_ms_exact=29.413375; cycles=470614; us_ceil=29414; F_CPU=16000000");
void TrafficLight::SwitchToRed(){
   simulate_(SIM_UNITS_FROM_VAL(29.50f));
}

WCET_AT("wcet_ms_exact=29.413375; cycles=470614; us_ceil=29414; F_CPU=16000000");
void TrafficLight::SwitchToGreen(){
    simulate_(SIM_UNITS_FROM_VAL(29.50f));
}


#if !defined(__AVR__)
enum LS { Green, Yellow, Red, };
using namespace TypestateLibrary::Template;
using TrafficLight_Typestate = Typestate_Template<Cycle<ms(75)>,
    Timed_State<LS::Red, &TrafficLight::SwitchToAmber, TimeGuard<0,ms(15),Criticality::High,0>, LS::Yellow>,
    Timed_State<LS::Yellow, &TrafficLight::SwitchToGreen, TimeGuard<0,ms(30),Criticality::High,0>, LS::Green>,
    Timed_State<LS::Green, &TrafficLight::SwitchToRed, TimeGuard<0,ms(30),Criticality::High,0>, LS::Red>
>;

static TypestateClassConnector<TrafficLight, TrafficLight_Typestate> LCflag;
#endif
