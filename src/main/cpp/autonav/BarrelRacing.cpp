#include "autonav/BarrelRacing.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Drivetrain.h"

//  THIS ONE WORKS

// int ms = 0;

void BarrelRacing::Run(){
    ms += 20;

    //forward
    if(ms <= 1800){
        autoDrive(2000, -175, false);
    }
    //around
    else if(ms <=5580){
        autoDrive(1800, -910, false);
    }
    //forward
    else if(ms <=7000){
         autoDrive(2000, -250, false);
    }
    //around
    else if(ms <=10000){
         autoDrive(1800, 700, false);
    }
    //forward
    else if(ms <=11500){
        autoDrive(2000, -175, false);
    }
    //around
    else if(ms <=13398){
         autoDrive(1800, 735, false);
    }
    //forward
    else if(ms <=18000){
        autoDrive(2000, -175, false);
    }
    //stop
    else {
        autoDrive(0.0f, 0.0f, false);
    }
    
}