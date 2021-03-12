#include "autonav/Slalom.h"
#include "subsystems/Drivetrain.h"

int ms = 0;

void Slalom::Run(){
    ms += 20;

    if(ms <= 1200){
        autoDrive(2000, -175, false);
    }
    else if(ms <= 2000){
        autoDrive(2000, -600, false);
    }
    else if(ms <= 3800){
        autoDrive(2000, -175, false);
    }
    else if(ms <= 4800){
        autoDrive(2000, -900, false);
    }
    else if(ms <= 8100){
        autoDrive(2000, 700, false);
    }
    else if(ms <= 9200){
        autoDrive(2000, -900, false);
    }
    else if(ms <= 10900){
        autoDrive(2000, -175, false);
    }
    else if(ms <= 11700){
        autoDrive(2000, -600, false);
    }
    else if(ms <= 12700){
        autoDrive(2000, -175, false);
    }
    else {
        autoDrive(0, 0, false);
    }
}