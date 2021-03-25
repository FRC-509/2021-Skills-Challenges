#include "autonav/Bounce.h"
#include "subsystems/Drivetrain.h"

void Bounce::Run(){
    ms+=20;

    //  ms = time in milliseconds
    //  autoDrive( speed, rotation, reverse )
    if(ms <= 1000){

    }
    else {
        autoDrive(0.0f, 0.0f, false);
    }
}