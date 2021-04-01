//  TO-DO:
//    Get hood to track
//    Get shooter speed set properly

#include "util/Motors.h"
#include "util/PID.h"
#include <cmath>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

//debugging
#include <frc/smartdashboard/SmartDashboard.h>

extern bool shooterReady;

//  Hood PID
#define hoodDown 0
#define hoodKp 0.005
#define hoodKi 0
double hoodAngle;
double hoodSetpoint;
double hoodPosition;
#define hoodMax 0
#define hoodMin -40

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

bool syncShooters(double input){
  // l_shooter.Set(ControlMode::Velocity, input * (shooterAdjustment) * 4096/600);
  // r_shooter.Set(ControlMode::Velocity, -1 * input * (shooterAdjustment) * 4096/600);
  l_shooter.Set(ControlMode::Velocity, input);
  r_shooter.Set(ControlMode::Velocity, -1 * input);
  if (input > 0 || input < 0){
    return true;
  } else {
    return false;
  }
}
double cotan (double angle){ return 1/tan(angle); }

// double shooterSpeed(){
//   hoodAngle = atan((2*(73/12))/targetDistance);
//   //hoodAngle = (50*M_PI)/180;
//   //hoodAngle in radians
//   vFeetPerSecond = (2*(sqrt(((73/12)*cotan(hoodAngle)*32.185)/sin(2*hoodAngle))));
//   // shooterInput = shooterRPM/(1.25);
//   return shooterInput;
//   // frc::SmartDashboard::PutNumber("feet per second", vFeetPerSecond);
// }

void HoodManual(double x){
  // hood.Set(x);
}

void shoot (bool active){
  frc::SmartDashboard::PutNumber("Hood encoder", hoodEncoder.GetPosition());

  if(!active){
    lltable->PutNumber("ledMode", 1);
    syncShooters(0);
    turret.Set(0);
    hood.Set(0);
    return;
  }

  double targetArea = lltable->GetNumber("ta", 0);
  // double targetDistance = ((18)*(pow(targetArea, -.509)));
  targetDistance = (18.6088 * pow(targetArea, -0.517431)) - 3.93319;
  double angle = 1/tan((2 * 6.52) / targetDistance);
  double vFpS = (2*(sqrt(((73/12)*cotan(angle)*32.185)/sin(2*angle))));
  frc::SmartDashboard::PutNumber("target distance calculated:", targetDistance);
  frc::SmartDashboard::PutNumber("angle:", angle);
  frc::SmartDashboard::PutNumber("speed:", vFpS);
  double sInput = vFpS * (12288/(2 * M_PI));
  frc::SmartDashboard::PutNumber("sInput:", sInput);

  lltable->PutNumber("ledMode", 3);

  //targetDistance = ((18)*(pow(lltable->GetNumber("ta", 0), -.509)));

  // if(turretTracking() && hoodTracking()){
    // syncShooters(shooterSpeed());
    if(turretTracking() && hoodTracking()){
      shooterReady = true;
      syncShooters(sInput);
    }
    else {
      shooterReady = false;
    }
  // }
}

//turretTracking function, outputs true if at position
bool turretTracking(){
  double horizontalOffset;
  horizontalOffset = lltable->GetNumber("tx", 0);
  turret.Set(PID(-horizontalOffset, turretKp, turretKi));

  if (horizontalOffset <= 2){
    return true;
  } else {
    return false;
  }
  
}

void hoodSet(double input){
  if(hoodEncoder.GetPosition() > hoodMin && hoodEncoder.GetPosition() < hoodMax){
    hood.Set(input);
  }
  else if (hoodEncoder.GetPosition() < hoodMin){
    if(input < 0){
      hood.Set(-input);
    }
    else hood.Set(input);
  }
  else if (hoodEncoder.GetPosition() > hoodMax){
    if(input > 0){
      hood.Set(-input);
    }
    else hood.Set(input);
  }
}

//hoodTracking function, outputs true if at position    
bool hoodTracking(){
  
  double error;
  hoodTarget = atan((2*(73/12))/targetDistance);
  hoodTargetDegrees = ((hoodTarget/M_PI)*180);
  hoodTargetEncoder = ((-2 * hoodTargetDegrees) + 100);

  error = hoodTargetEncoder - hoodEncoder.GetPosition();
  //50deg to 70deg(hood) 
  //hood.Set(logicontroller.GetRawAxis(1));
  frc::SmartDashboard::PutNumber("hood target: ", hoodTargetEncoder);
  frc::SmartDashboard::PutNumber("Hood encoder", hoodEncoder.GetPosition());
  hoodSet(PID(error, hoodKp, hoodKi));
  
  if (-2 <= error && error <= 2){
    return true;
  } else {
    return false;
  }
  // frc::SmartDashboard::PutNumber("hoodTarget", hoodTarget);
}