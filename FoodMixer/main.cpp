#include "FoodMixer.h"
[[noreturn]]
int main() {
  
    FoodMixer mixer;
  
    const int MAX_LEVEL_MM = 30; 
    while(1){
    
    int tankLevel = mixer.ReadLevelSensor();   
    int percent = mixer.CalcFillPercent(tankLevel, MAX_LEVEL_MM);
    mixer.SetActuator(percent);
   
    }
        return 0;
}
