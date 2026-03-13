#include "DroneC.h"

int main()
{
    DroneController drone;

    constexpr long CYCLE_MS = 100;
    constexpr float dt = 0.1f;

    printf("DRONED v0.89 drone flight control daemon\n");

    printf("Initializing drone mathematical model...");
    drone.initialize_fmm();
    printf("initialized.\n");

    printf("Initializing Flight Plan...");
    drone.initialize_flightplan();
    printf("Initialized.\n");

    // Demo: allow mission start
    drone.go_signal = 1;

    printf("Entering main loop...\n");

    for (int step = 0; step < 200; ++step) {

        // STATE 3 -> 4 or 5
        if (drone.checkStart() == WAIT_GO) {
            // STATE 4
            drone.killmotors();
            printf("[OFF] waiting for GO\n");
            continue;
        }

        // STATE 5 -> 6
        drone.idlemotors();

        if (drone.glng_counter == 0) {
            drone.glng_counter = 50;
        }
        drone.glng_counter--;

        if (drone.glng_counter > 0) {
            printf("[ON] warmup... %ld\n", drone.glng_counter);
            continue;
        }

        // STATE 6 -> 6 or 7
        if (drone.checkAltitude() == ALT_NOT_REACHED) {
            // still in STATE 6, so setmotors is legal here
            drone.setmotors(ASCEND_DC);
            drone.curdrone.y += 1.0;
            printf("[ASCEND] y=%2.2lf\n", drone.curdrone.y);
            continue;
        }

        // altitude reached: now in STATE 7
        drone.acquire();  // 7 -> 8
       //  drone.descend();  
        drone.traveltowaypoint(drone.wp_index, 1.0, 2.0, 0.1); // 8 -> 9

        // STATE 9 -> 7 or 10
        switch (drone.neutralDecision()) {
        case CONTINUE_MISSION:
            // neutralDecision_cond_0 gives STATE 7
            drone.acquire();  // 7 -> 8
            drone.traveltowaypoint(drone.wp_index, 1.0, 2.0, 0.1); // 8 -> 9
            printf("[NEUTRAL] continuing mission...\n");
            break;

        case LAND_NOW:
            // neutralDecision_cond_1 gives STATE 10
            drone.descend();      // 10 -> 4
            drone.curdrone.y -= 1.0;
            printf("[DESCEND] y=%2.2lf\n", drone.curdrone.y);

            drone.killmotors();   // 4 -> 4
            printf("[OFF] landed.\n");
            break;

        default:
            // Avoid typestate-illegal calls here.
            // At this point the analyzer may still consider you in state 9.
            printf("[WARN] neutralDecision returned unknown value.\n");
            break;
        }

        // demo model update if needed
        // printf("x=%2.2lf, y=%2.2lf, z=%2.2lf\n",
        //        drone.curdrone.x, drone.curdrone.y, drone.curdrone.z);
    }

    printf("\nDaemon shutting down...returning to OS\n\n");
    return 0;
}