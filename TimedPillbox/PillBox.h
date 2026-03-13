#pragma once

#include <stdint.h>
#include <stddef.h>
#include "Drawer.h"
#include "RedLed.h"
#include "memory.h"
#include "../include/Typestate_Library/TypestateLibrary_stub.h"


#include "../hw_de.h"
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
}

WCET_AT("wcet_ms_exact=0.001125; cycles=18");
void PillBox::Activate_pillBox() {
    if (!isActive) {
        isActive = true;
        outMessageCode = MSG_ACTIVATED;
        logMessageCode = MSG_ACTIVATED;
        debug_state = 1;
    } else {
        debug_state = 2;
    }
}

WCET_AT("wcet_ms_exact=0.002250; cycles=36");
void PillBox::addDrawers(Drawer* d) {
    if (d == nullptr) {
        debug_state = -1;
        return;
    }

    if (drawerCount >= MAX_DRAWERS) {
        debug_state = -2;
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
}

WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
Drawer* PillBox::Process_System_Time(int h, int m) {
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
                return d;
            }
        }
    }

    return nullptr;
}

WCET_AT("wcet_ms_exact=0.001187; cycles=19");
void PillBox::Deactivate_Pill_Box() {
    if (isActive) {
        isActive = false;
        outMessageCode = MSG_DEACTIVATED;
        logMessageCode = MSG_DEACTIVATED;
        debug_state = 30;
    } else {
        debug_state = 31;
    }
}


WCET_AT("wcet_ms_exact=40.052687; cycles=640843");
void PillBox::Switch_ON(Drawer* d) {
    if (d == nullptr) {
        debug_state = -3;
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
}


WCET_AT("wcet_ms_exact=6.013438; cycles=96215");
void PillBox::Switch_OFF(Drawer* d) {
    if (d == nullptr) {
        debug_state = -5;
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
}

WCET_AT("wcet_ms_exact=0.004938; cycles=79");
void PillBox::Blink(Drawer* d) {
    if (d == nullptr) {
        debug_state = -7;
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
}

WCET_AT("wcet_ms_exact=40.052500; cycles=640840");
void PillBox::OpenDrawer(Drawer* d) {
    if (d == nullptr) {
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


}
WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void PillBox::CheckTimeouts() {
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
}
WCET_AT("wcet_ms_exact=NaA; cycles=NaA");
void PillBox::PrintStatus() {
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
}

WCET_AT("wcet_ms_exact=0.001187; cycles=19");
int PillBox::findDrawerIndex(Drawer* d) {
    for (int i = 0; i < MAX_DRAWERS; ++i) {
        if (DrawersBox[i] == d) {
            return i;
        }
    }
    return -1;
}



enum class domain {
    Idle ,
    Add,
    Active ,
    NonActive ,
    Pill_Time_On ,
    RedLedON ,
    RedLedOFF ,
    RedLedBlinking ,
    Print
};

using PillBox_typestate = Typestate_Template<
    Cycle<ms(140)>,
    Timed_State<
        domain::Idle,
        &PillBox::addDrawers,
        TimeGuard<0, 0, lower(ms(0)), upper(ms(1)), Criticality::High>,
        domain::Add
    >,
    Timed_State<
        domain::Add,
        &PillBox::addDrawers,
        TimeGuard<0, 0, lower(ms(0)), upper(ms(1)), Criticality::High>,
        domain::Add
    >,
    Timed_State<
        domain::Add,
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
        &PillBox::PrintStatus,
        TimeGuard<0, 0, lower(ms(0)), upper(ms(2)), Criticality::High>,
        domain::Print
    >,
    Timed_State<
        domain::Print,
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
