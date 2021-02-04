#include "util/Motors.h"

#define conveyorSpeed 0.5
#define liftPower 0.5

void syncLift(double input){
    lift1.Set(input);
    lift2.Set(-input);
}

void conveyorBelt(int multiplier){
  belt.Set(multiplier * conveyorSpeed);
  syncLift(multiplier * liftPower);
}