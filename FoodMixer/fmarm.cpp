#include "FoodMixer-arm.h"

int main(void) {
    FoodMixer mixer;
    const float MAX_LEVEL_MM = 30.0f;
    while (true) {
        float tankLevel = mixer.ReadLevelSensor();
        float percent   = mixer.CalcFillPercent(tankLevel, MAX_LEVEL_MM);
        mixer.SetActuator(percent);
    }
    return 0;
}
