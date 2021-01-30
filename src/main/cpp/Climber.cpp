#include "Motors.h"
#include "PID.h"

//  Elevator PID needs to be tuned
#define elevatorUp 0
#define elevatorDown 0
#define elevatorIntermediate 0
#define elevatorKp 0
#define elevatorKi 0
double elevatorSetPoint;
double elevatorError;

void climber(int setMode, double manualInput, double skywalkerInput){

    // switch(setMode){
    //     case 0:
    //         elevatorError = elevatorSetPoint - elevatorDown;
    //         elevator.Set(PID(elevatorError, elevatorKp, elevatorKi));
    //         break;
    //     case 1:
    //         elevatorError = elevatorSetPoint - elevatorUp;
    //         elevator.Set(PID(elevatorError, elevatorKp, elevatorKi));
    //         break;
    //     case 2:
    //         elevator.Set(manualInput);
    //         break;
    // }

    // skywalker.Set(ControlMode::PercentOutput, skywalkerInput);
}