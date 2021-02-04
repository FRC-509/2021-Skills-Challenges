#include "util/Motors.h"
#include "util/PID.h"
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

#define shooterAdjustment 1.05

//Limelight Data Table
auto llinst = nt::NetworkTableInstance::GetDefault();
auto lltable = llinst.GetTable("limelight");

bool syncShooters(double input){
  l_shooter.Set(ControlMode::Velocity, input * (shooterAdjustment) * 4096/600);
  r_shooter.Set(ControlMode::Velocity, -1 * input * (shooterAdjustment) * 4096/600);
  if (input > 0 || input < 0){
    return true;  
  } else {
    return false;
  }
}
double cotan (double angle){ return 1/tan(angle); }
double shooterSpeed(){
  hoodAngle = atan((2*(73/12))/targetDistance);
  //hoodAngle = (50*M_PI)/180;
  //hoodAngle in radians
  vFeetPerSecond = (2*(sqrt(((73/12)*cotan(hoodAngle)*32.185)/sin(2*hoodAngle))));
  // shooterInput = shooterRPM/(1.25);
  return shooterInput;
  // frc::SmartDashboard::PutNumber("feet per second", vFeetPerSecond);
}

void shoot (bool active){
  if(!active){
    lltable->PutNumber("ledMode", 1);
    syncShooters(0);
    return;
  }

  lltable->PutNumber("ledMode", 3);

  targetDistance = ((18)*(pow(lltable->GetNumber("ta", 0), -.509)));

  // if(turretTracking() && hoodTracking()){
    // syncShooters(shooterSpeed());
    syncShooters(2750);
  // }
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