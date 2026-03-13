#include "PillBox.h"

int main() {
    Drawer drawer1(3, 50);
    Drawer drawer2(8, 40);
    Drawer drawer3(12, 0);

    PillBox pillbox1;
    pillbox1.addDrawers(&drawer1);
    pillbox1.addDrawers(&drawer2);
    

    pillbox1.Activate_pillBox();

    const int simulationSteps = 6;
    bool redLedOn = false;
    bool blinking = false;

    for (int step = 0; step < simulationSteps; ++step) {
        Drawer* scheduledDrawer = nullptr;

        
    scheduledDrawer = pillbox1.Process_System_Time(3, 50);
        
        pillbox1.Switch_ON(scheduledDrawer);
                redLedOn = true;

                pillbox1.OpenDrawer(scheduledDrawer);
                blinking = true;
          
                pillbox1.Switch_OFF(scheduledDrawer);
                redLedOn = false;
                blinking = false;
           
        
        pillbox1.PrintStatus();
    }

    pillbox1.Deactivate_Pill_Box();
    return 0;
}