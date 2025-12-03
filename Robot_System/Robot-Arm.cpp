
#include "Robot-Arm.h"

int main() {

   Robot r;
   while(1){
     
   r.Read_force_data();
   r.Read_path_angle();
   float b= r.ComputePathAngle();

   if (r.angle_changed==boolean::FALSE) {
     r.Compute_set_points_slow(b);
   } else {
    r.Compute_set_points_fast(b);
  }
  r.AdjustActuators(2);
  r.End();
   }
  return 0;
}
