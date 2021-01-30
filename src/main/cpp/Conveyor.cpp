#include "Motors.h"

#define conveyorSpeed 0.5
#define liftPower 0.5

void conveyorBelt(int multiplier){
  belt.Set(multiplier * conveyorSpeed);
  syncLift(multiplier * liftPower);
}

void syncLift(double input){
    lift1.Set(input);
    lift2.Set(-input);
}

// void conveyor(int mode, bool shooting){
//   bool conveyor;
//   bool manual;
//   bool errorState;
//   switch(mode){
//     //auto control(normal)
//     case 0:
//       if(!sensorIntake.Get() && sensorExit.Get()){
//         belt.Set(conveyorSpeed);
//         conveyor = 1;
//       } else {
//         belt.Set(0);
//         conveyor = 0;
//       }
//       syncLift(0);
//       manual = 0;
//       errorState = 0;
//       break;      
//     //shoot(going towards shooter)
//     case 1:
//       belt.Set(conveyorSpeed);
//       syncLift(liftPower);
//       manual = 0;
//       errorState = 0;
//       break;
//     //poot(going towards intake)
//     case 2:
//       belt.Set(-conveyorSpeed);
//       syncLift(-liftPower);
//       manual = 0;
//       errorState = 0;
//       break;
//     //manual control (uncomplete)
//     case 3:
//       if(!shooting){
//         belt.Set(-logicontroller.GetRawAxis(3));
//         syncLift(-logicontroller.GetRawAxis(3));
//       } else {
//         belt.Set(-logicontroller.GetRawAxis(3));
//         syncLift(-logicontroller.GetRawAxis(3));
//       }
//       manual = 1;
//       errorState = 0;
//       break;
//     default:
//       errorState = 1;
//       break;
//   }
    
//     //output
//     frc::SmartDashboard::PutBoolean("Conveyor Belt On?:", conveyor);
//     frc::SmartDashboard::PutBoolean("Manual Control On?:", manual);
//     frc::SmartDashboard::PutBoolean("Full?", !sensorExit.Get());
// }