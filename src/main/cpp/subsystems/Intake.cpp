#include "util/Motors.h"

#define intakePower 0.75

void intake (bool down){
    if (down){
        intakeSolOpen.Set(false);
        intakeSolClose.Set(true);
        MCGintake.Set(ControlMode::PercentOutput, -intakePower);
    } else {
        intakeSolOpen.Set(true);
        intakeSolClose.Set(false);
        MCGintake.Set(ControlMode::PercentOutput, 0);
    }
}