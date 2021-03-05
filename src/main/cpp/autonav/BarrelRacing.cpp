#include "autonav/BarrelRacing.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include "util/Motors.h"
#include "subsystems/Drivetrain.h"

int ms = 0;

void autoDrive(double v, double r, bool reverse){
    leftFrontFalcon.Set(ControlMode::Velocity, (-v - r) * 4096/600);
    leftBackFalcon.Set(ControlMode::Velocity, (-v - r) * 4096/600);
    rightFrontFalcon.Set(ControlMode::Velocity, (v + r) * 4096/600);
    rightBackFalcon.Set(ControlMode::Velocity, (v + r) * 4096/600);

}

void BarrelRacing::Run(){
    ms += 20;

    if(ms <= 2600){
        autoDrive(0.4f, 0.0f, false);
    }
    else if(ms <= 5600){
        autoDrive(0.3f, 0.12f, false);
    }
    else {
        autoDrive(0.0f, 0.0f, false);
    }
}