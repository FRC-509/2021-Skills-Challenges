#include "Motors.h"
#include <cmath>

//Drive Functions
//  Sync left and right wheel motors
void leftDrive(double power){
  leftFrontFalcon.Set(ControlMode::PercentOutput, power);
  leftBackFalcon.Set(ControlMode::PercentOutput, power);
}
void rightDrive(double power){
  rightFrontFalcon.Set(ControlMode::PercentOutput, -power);
  rightBackFalcon.Set(ControlMode::PercentOutput, -power);
}
//  Drive smoothing
float driveCurve(float input){
  float output;
  float linearity = 0.5;
  output = linearity * input + (1-linearity) * pow(input, 3);
  return output;
}
//Intake motors
// void intake(double power, bool intakeSol){
//   //Setting intake power
//   MCGintake.Set(ControlMode::PercentOutput, -power);
//   //Setting intake position
//   if (intakeSol){
//     intakeSolOpen.Set(false);
//     intakeSolClose.Set(true);
//   } else {
//     intakeSolOpen.Set(true);
//     intakeSolClose.Set(false);
//   }
// }
//  Combined Drive Function
void drive(float left, float right, bool reverse){
  if (!reverse){
    leftDrive(driveCurve(left) - driveCurve(right));
    rightDrive(driveCurve(left) + driveCurve(right));
  } else {
    leftDrive(-driveCurve(left) - driveCurve(right));
    rightDrive(-driveCurve(left) + driveCurve(right));
  }

    // //intaking
    // if (intaking){
    //   intakeSolUp = 1;
    //   intake(intakePower, intakeSolUp);
    // } else {
    //   intakeSolUp = 0;
    //   intake(0, intakeSolUp);
    // }
  //Output Intake Position to SmartDashboard
  // frc::SmartDashboard::PutBoolean("Intake Down?", intakeSolUp);
}




// void climber(double elevatorPosition, bool manual){
  
//   double upError;
//   double downError;
//   upError = elevatorUp - elevatorPosition;
//   downError = elevatorDown - elevatorPosition;
//   //Manual Elevator control
//   if(manual){
//     if(l_stick.GetRawButton(3)){
//       brakeSolOff.Set(true);
//       brakeSolOn.Set(false);
//       elevator.Set(l_stick.GetRawAxis(1));
//     } else {
//       elevator.Set(0);
//       brakeSolOff.Set(false);
//       brakeSolOn.Set(true);
//     }
//   } else {
//     //experimental PID
//     if(l_stick.GetRawButton(2)){
//       if(1 > upError > -1){
//         brakeSolOff.Set(true);
//         brakeSolOn.Set(false);
//         elevator.Set(PID(upError, elevatorKp, elevatorKi));
//       } else {
//         brakeSolOff.Set(false);
//         brakeSolOn.Set(true);        
//         elevator.Set(0);
//       }
//     } else if(l_stick.GetRawButton(8)) {
//       if(1 > downError > -1){
//         brakeSolOff.Set(true);
//         brakeSolOn.Set(false);
//         elevator.Set(PID(downError, elevatorKp, elevatorKi));
//       } else {
//         brakeSolOff.Set(false);
//         brakeSolOn.Set(true);        
//         elevator.Set(0);
//       }
//     } else {
//       elevator.Set(0);
//       brakeSolOff.Set(false);
//       brakeSolOn.Set(true);
//     }

//   }
//   //Skywalker Control
//   skywalker.Set(ControlMode::PercentOutput, logicontroller.GetRawAxis(2)); 
// }