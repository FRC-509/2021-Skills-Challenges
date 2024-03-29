#include "util/Motors.h"
#include <cmath>

//  Velocity-based drive used for autonomous
void autoDrive(double v, double r, bool reverse){
  if(!reverse){
    leftFrontFalcon.Set(ControlMode::Velocity, (v + r) * 4096/600);
    leftBackFalcon.Set(ControlMode::Velocity, (v + r) * 4096/600);
    rightFrontFalcon.Set(ControlMode::Velocity, -(v - r) * 4096/600);
    rightBackFalcon.Set(ControlMode::Velocity, -(v - r) * 4096/600);
  }
  else {
    leftFrontFalcon.Set(ControlMode::Velocity, -(v + r) * 4096/600);
    leftBackFalcon.Set(ControlMode::Velocity, -(v + r) * 4096/600);
    rightFrontFalcon.Set(ControlMode::Velocity, (v - r) * 4096/600);
    rightBackFalcon.Set(ControlMode::Velocity, (v - r) * 4096/600);
  }
}

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

//  Combined Drive Function
void drive(float left, float right, bool reverse){
  if (!reverse){
    leftDrive(driveCurve(left) - driveCurve(right));
    rightDrive(driveCurve(left) + driveCurve(right));
  } else {
    leftDrive(-driveCurve(left) - driveCurve(right));
    rightDrive(-driveCurve(left) + driveCurve(right));
  }
}