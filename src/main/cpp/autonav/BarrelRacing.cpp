#include "autonav/BarrelRacing.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include "util/Motors.h"
#include "subsystems/Drivetrain.h"

int ms = 0;

void autoDrive(double v, double r, bool reverse){
    leftFrontFalcon.Set(ControlMode::Velocity, (v + r) * 4096/600);
    leftBackFalcon.Set(ControlMode::Velocity, (v + r) * 4096/600);
    rightFrontFalcon.Set(ControlMode::Velocity, -(v - r) * 4096/600);
    rightBackFalcon.Set(ControlMode::Velocity, -(v - r) * 4096/600);

}

void BarrelRacing::Run(){
    ms += 20;

    // IT WORKS DON'T TOUCH THIS
    if(ms <= 1800){
        autoDrive(2000, -175, false);
    }
    else if(ms <=5500){
        autoDrive(1800, -910, false);
    }
    
    else if(ms <=7000){
         autoDrive(2000, -175, false);
    }
    else if(ms <=9600){
         autoDrive(1800, 735, false);
    }
    else if(ms <=10000){

    }
    else {
        autoDrive(0.0f, 0.0f, false);
    }
    
}