#include "Motors.h"
#include "PID.h"
#include <cmath>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

//  Hood PID
#define hoodDown 0
#define hoodKp 0.005
#define hoodKi 0
double hoodAngle;
double hoodSetpoint;
double hoodPosition;

double hoodTarget;
double hoodTargetDegrees;
double hoodTargetEncoder;
//  Turret PID
#define turretKp 0.05
#define turretKi 0
#define minOffset 10
#define turretDefaultSetpoint 0
double turretPosition;
double horizontalOffset;
double shooterInput;

double targetDistance;
double vFeetPerSecond;

bool hoodTracking();
bool turretTracking();

//Limelight Data Table
auto llinst = nt::NetworkTableInstance::GetDefault();
auto lltable = llinst.GetTable("limelight");

void syncShooters(double power){
  //  MAKE SURE THIS ISNT BACKWARDS
  // l_shooter.Set(ControlMode::PercentOutput, power);
  // r_shooter.Set(ControlMode::PercentOutput, -power);
}
double cotan (double angle){ return 1/tan(angle); }
double shooterSpeed(){
  hoodAngle = atan((2*(73/12))/targetDistance);
  //hoodAngle = (50*M_PI)/180;
  //hoodAngle in radians
  vFeetPerSecond = (2*(sqrt(((73/12)*cotan(hoodAngle)*32.185)/sin(2*hoodAngle))));
  // shooterInput = shooterRPM/(1.25);
  // return shooterInput;
  // frc::SmartDashboard::PutNumber("feet per second", vFeetPerSecond);
}

void shoot (bool active){
  if(!active) return;

  targetDistance = ((18)*(pow(lltable->GetNumber("ta", 0), -.509)));

  if(turretTracking() && hoodTracking()){
    syncShooters(shooterSpeed());
  }
}

//turretTracking function, outputs true if at position
bool turretTracking(){
  double horizontalOffset;
  horizontalOffset = lltable->GetNumber("tx", 0);
  turret.Set(PID(-horizontalOffset, turretKp, turretKi));
  //turretSet((0 - horizontalOffset));
  if (5 <= horizontalOffset && horizontalOffset <= 5){
    return true;
  } else {
    return false;
  }
  // frc::SmartDashboard::PutNumber("horizontal offset", horizontalOffset);
}
//hoodTracking function, outputs true if at position    
bool hoodTracking(){
  
  double error;
  hoodTarget = atan((2*(73/12))/targetDistance);
  hoodTargetDegrees = ((hoodTarget/M_PI)*180);
  hoodTargetEncoder = ((-2 * hoodTargetDegrees) + 100);
  //50deg to 70deg(hood) 
  //hood.Set(logicontroller.GetRawAxis(1));
  
  hood.Set(PID((hoodTargetEncoder - hoodEncoder.GetPosition()), hoodKp, hoodKi));
  if (-0.1 <= error && error <= 0.1){
    return true;  
  } else {
    return false;
  }
  // frc::SmartDashboard::PutNumber("hoodTarget", hoodTarget);
}
//aiming function(aims, and outputs if it's aimed)
// bool aiming(bool manual){
  
//   bool aimed;
//   if(!manual){  
//     if ((turretTracking()) && hoodTracking()){
//       aimed = 1;
//       return aimed;
//     } else {
//       aimed = 0;
//       return aimed;
//     }
//   } else {
//     turretSet(-logicontroller.GetRawAxis(0));
//     hoodSet(-logicontroller.GetRawAxis(1));
//   }
//   frc::SmartDashboard::PutBoolean("aimed?", aimed);
// }
// double shooterSubsystem(int mode, bool shootCommand){
//   if(mode == 3){
//     if (shootCommand){
//       limelightOn(1);
//       mode = 1;
//     } else {
//       limelightOn(0);
//       mode = 0;
//     }
//   } else {
//     if (shootCommand){
//       limelightOn(1);
//       mode = 1;
//     } else {
//       limelightOn(0);
//       mode = 0;
//     }
//   }
  
//   //in development
//   //bool shootCommand
//   //shootCommand = logicontroller.GetRawButton(8);
//   bool aimCommand;
//   bool manualControl;
//   bool conveyorOn;
//   bool spitting;
//   bool shooterOn;
//   bool aimMode;
//   if (mode == 3){
//     aimMode = 1;
//   } else {
//     aimMode = 0;
//   }
//   bool targetFound;
//   targetFound = limelightTargetAquired();  
    
//   double shooterInput;
//   shooterInput = shooterSpeed();
  
//   //
//   switch(shooterMode){
    
//     //Intake
//     case 0:
//       conveyor(0, shootCommand);
//       manualControl = 0;
//       break;
//     //Shooting
//     case 1:
//       conveyor(1, shootCommand);
//       if(aiming(mode) && shootCommand){
//         syncShooters(shooterInput);
//       } else {
//         syncShooters(0);
//       }
//       manualControl = 0;
//       break;
//     //Spit
//     case 2:
//       shooterOn = syncShooters(0);
//       conveyor(2, shootCommand);
//       manualControl = 0;
//       break;
//     //"Manual Control"
//     case 3:
//       turret.Set(logicontroller.GetRawAxis(0));
//       hoodSet(logicontroller.GetRawAxis(1));
//       conveyor(3, shootCommand);
//       //shooterOn = syncShooters(3250);
//       manualControl = 1;
//       break;    
//   }
  
//   if ((aiming(aimMode) || manualControl) && shootCommand){
//     if(manualControl){    
//       shooterOn = syncShooters(2750);
//       //temp fix
//       conveyor(3, shootCommand);
//       conveyorOn = true;
//     } else {
//       shooterOn = syncShooters(shooterInput);
//       //temp fix
//       conveyor(3, shootCommand);
//       conveyorOn = true;
//     }
//   } else {
//     syncShooters(0);
//     conveyorOn = false;
//   }
//   //smart dashboard output
//   frc::SmartDashboard::PutBoolean("SHOOTER ON?:", shooterOn);
//   frc::SmartDashboard::PutBoolean("Manual Control?", manualControl);
//   frc::SmartDashboard::PutBoolean("CONVEYOR BELT:", conveyorOn);
//   frc::SmartDashboard::PutNumber("shooter mode for real", mode);
//   return shooterInput;
// }