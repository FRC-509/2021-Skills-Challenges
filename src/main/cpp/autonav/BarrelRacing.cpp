#include "autonav/BarrelRacing.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Drivetrain.h"

int ms = 0;

void BarrelRacing::Run(){
    frc::SmartDashboard::PutNumber("Time", ms);

    ms += 20;

    if(ms <= 2400){
        drive(0.4f, 0.0f, false);
    
    }
    else if(ms <= 7500){
        drive(0.4f, 0.15f, false);
    }
    else {
        drive(0.0f, 0.0f, false);
    }
}