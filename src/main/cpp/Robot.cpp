/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


//Configure Autonav Path
#include "autonav/Slalom.h"
#include "autonav/BarrelRacing.h"
#include "autonav/Bounce.h"
path* AutoNavPath = new BarrelRacing;


//LIBRARIES
#include "Robot.h"
#include <iostream>
#include <cmath>
#include <frc/Joystick.h>
// #include <frc/smartdashboard/SmartDashboard.h>
#include <frc/util/color.h>
#include <frc/Servo.h>
#include <frc/Filesystem.h>
#include <frc/Encoder.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <frc/WPILib.h>
#include <cameraserver/CameraServer.h>
#include <vision/VisionRunner.h>
#include <fstream>
using namespace std;

#include "subsystems/Shoot.h"
#include "subsystems/Conveyor.h"
#include "subsystems/Intake.h"
#include "subsystems/Climber.h"
int climbingMode;
#include "subsystems/ColorWheel.h"
#include "subsystems/Drivetrain.h"

enum class RobotMode {
  Paused, Intake, Shoot, Spit, Climbing
};
RobotMode mode;

//GLOBAL VARIABLES

//CONFIGURATION

//CONSTANTS & OTHER GLOBAL VARIABLES

//CONTROLLERS

frc::Joystick r_stick  {0};
frc::Joystick l_stick  {1};
frc::Joystick logicontroller {2};


//Upon robot startup
void Robot::RobotInit() {
  shoot(false);
  conveyorBelt(0);
  intake(false);
  climber(0, 0, 0);
  colorWheel(0);
  drive(0, 0, 0);



  //CURRENT LIMITING
  //limelightOn(1);  
  //Current Limiting SparkMAX
  // lift1.SetSmartCurrentLimit(10);
  // lift2.SetSmartCurrentLimit(10);
  // belt.SetSmartCurrentLimit(10);
  // turret.SetSmartCurrentLimit(10);
  // hood.SetSmartCurrentLimit(10);
  // elevator.SetSmartCurrentLimit(60);
  // colorWheelMotor.SetSmartCurrentLimit(10);
  
  //Current Limiting Falcons + Intake (WIP)
  cs::UsbCamera usbcamera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  cs::UsbCamera usbcamera2 = frc::CameraServer::GetInstance()->StartAutomaticCapture();
  //Adding Camera Feeds
  usbcamera.SetResolution(1,1);
  usbcamera.SetFPS(15);
  usbcamera2.SetResolution(1,1);
  usbcamera2.SetFPS(15);
  
    
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  // m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  // frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  // //home turret encoder(put on checklist)
  // turretInit = turretEncoder.GetPosition();
  // hoodInit = hoodEncoder.GetPosition();
  // elevatorInit = elevatorPoint.GetPosition();
  // //  Update Encoders
  // turretPosition = turretEncoder.GetPosition() - turretInit;
  // hoodPosition = hoodEncoder.GetPosition() - hoodInit;
  // elevatorPosition = elevatorPoint.GetPosition() - elevatorInit;
  // //Turn off Limelight LED
  // limelightOn(0);

  //Shooter encoder reset
  
  //Shooter current limiting

  /*l_shooter->EnableCurrentLimit(true); 
  l_shooter->ConfigContinuousCurrentLimit(40,15);
  l_shooter->ConfigPeakCurrentLimit(0,15);
  r_shooter->EnableCurrentLimit(true);
  r_shooter->ConfigContinuousCurrentLimit(40,15);
  r_shooter->ConfigPeakCurrentLimit(0,15);
  */
  //Set motors off initially
  // leftFrontFalcon.Set(ControlMode::PercentOutput, 0);
  // leftBackFalcon.Set(ControlMode::PercentOutput, 0);
  // rightFrontFalcon.Set(ControlMode::PercentOutput, 0);
  // rightBackFalcon.Set(ControlMode::PercentOutput, 0);
  // l_shooter.Set(ControlMode::PercentOutput, 0);
  // r_shooter.Set(ControlMode::PercentOutput, 0);
  // skywalker.Set(ControlMode::PercentOutput, 0);
  // colorWheelMotor.Set(0);
  // turret.Set(0);
  // belt.Set(0);
  // frc::SmartDashboard::PutString("CONVEYOR BELT:", "INACTIVE");
  // lift1.Set(0);
  // lift2.Set(0);
  // MCGintake.Set(ControlMode::PercentOutput, 0);
  
  // //Add colors to color match
  // m_colorMatcher.AddColorMatch(kBlueTarget);
  // m_colorMatcher.AddColorMatch(kGreenTarget);
  // m_colorMatcher.AddColorMatch(kRedTarget);
  // m_colorMatcher.AddColorMatch(kYellowTarget);
 

 // frc::SmartDashboard::PutNumber("Resolution: ", usbcamera.getResolution());

 //usbcamera2.SetResolution(1,1);
 //usbcamera2.SetFPS(25);

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  // frc::SmartDashboard::PutNumber("led pwm val", LEDPWM);
  //limelightOn(1);
}
/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */


void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  // cout << "Auto selected: " << m_autoSelected << endl;

  // if (m_autoSelected == kAutoNameCustom) {
  //   // Custom Auto goes here
  // } else {
  //   // Default Auto goes here
  // }
  // wpi::SmallString<64> deployDirectory;
  // frc::filesystem::GetDeployDirectory(deployDirectory);
  // wpi::sys::path::append(deployDirectory, "paths");
  // wpi::sys::path::append(deployDirectory, "YourPath.wpilib.json");
  // frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
}
void Robot::AutonomousPeriodic() {
  AutoNavPath->Run();
  // if (m_autoSelected == kAutoNameCustom) {
  //   // Custom Auto goes here
  // } else {
    
  // }
}
void Robot::TeleopInit() {
}
void Robot::TeleopPeriodic() {

  if(logicontroller.GetRawButton(7)){
    mode = RobotMode::Shoot;
  }
  else if(logicontroller.GetRawButton(6)){
    mode = RobotMode::Intake;
  }
  else if(logicontroller.GetRawButton(8)){
    mode = RobotMode::Spit;
  }
  else if(logicontroller.GetRawButton(1)){
    mode = RobotMode::Climbing;
    climbingMode = 1;
  }
  else if(logicontroller.GetRawButton(2)){
    mode = RobotMode::Climbing;
    climbingMode = 2;
  }
  else {
    mode = RobotMode::Paused;
  }

  switch(mode){
    case RobotMode::Paused:
      shoot(false);
      conveyorBelt(0);
      intake(false);
      climber(0, 0, 0);
      break;
    
    case RobotMode::Intake:
      shoot(false);
      conveyorBelt(1);
      intake(true);
      climber(0, 0, 0);
      break;
    
    case RobotMode::Shoot:
      shoot(true);
      conveyorBelt(1);
      intake(false);
      climber(0, 0, 0);
      break;
    
    case RobotMode::Spit:
      shoot(false);
      conveyorBelt(-1);
      intake(false);
      climber(0, 0, 0);
      break;
    
    case RobotMode::Climbing:
      shoot(false);
      conveyorBelt(0);
      intake(false);
      climber(climbingMode, logicontroller.GetRawAxis(1), logicontroller.GetRawAxis(0));
      break;
  }

  colorWheel(logicontroller.GetRawButton(3));
  drive(l_stick.GetRawAxis(0), r_stick.GetRawAxis(0), r_stick.GetRawButton(2));
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif