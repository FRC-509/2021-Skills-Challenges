#include "autonav/Bounce.h"
#include "subsystems/Drivetrain.h"

void Bounce::Run(){
    ms+=20;

    //  ms = time in milliseconds
    //  autoDrive( speed, rotation, reverse )
    if(ms <= 1350){
        autoDrive(2000, 600, false);
    } 
    else if(ms <= 2800){
        autoDrive(-2000, -100, false);
    }
    else if(ms <= 4700){
        autoDrive(-2000, 1000, false);
    }
    else if(ms <= 6000){
        autoDrive(-2000, 175, false);
    }
    else if(ms <= 7500){
        autoDrive(2000, -175, false);
    }
    else if(ms <= 8000){
        autoDrive(2000, 1600, false);
    }
    else if(ms <= 8550){
        autoDrive(2000, -175, false);
    }
    else if(ms <= 9250){
        autoDrive(2000, 1600, false);
    }
    else if(ms <= 11000){
        autoDrive(2000, -175, false);
    }
    //  Needs work
    else if(ms <= 12300){
        autoDrive(-2000, 1000, false);
    }
    else {
        autoDrive(0.0f, 0.0f, false);
    }
}