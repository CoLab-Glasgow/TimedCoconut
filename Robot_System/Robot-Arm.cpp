#include "Robot-Arm.h"

int main() {
  Robot r;
  while (1) { 
    r.InitPID(); 
    r.Read_force_data();   // 1 -> 2
    r.Read_path_angle();   // 2 -> 3
    r.ComputePathAngle();
    r.ConfigurePIDForMode();
    const enum Mode m = r.ConfigMode();
    switch (m)
    {
    case Mode::Fast :
      r.Compute_targets_fast();
      break;
      case Mode::Slow :
      r.Compute_targets_slow();  
      break;
    
    default:
      break;
    }
    r.ComputePID();       // 8 -> 9
    r.ApplyActuators();   // 9 -> 10
    r.End();              // 10 -> 1
  }
}