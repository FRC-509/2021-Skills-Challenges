//Turns the Limelight LEDs on or off, returns status
bool limelightLEDs(bool ledState){
    if(ledState){
      //Turns the LEDs on
      lltable->PutNumber("ledMode", 3);
      return true;
    } else {
      //Turns the LEDs off
      lltable->PutNumber("ledMode", 1);
      return false;
    }
}
//Returns whether or not the Limelight has aquired a target
bool limelightTargetAquired(){
    bool targetAquired;
    targetAquired = lltable->GetNumber("tv", 0);
    if (targetAquired == 1){
      return true;
    } else {
      return false;
    }
}

//Limelight Output Function
float limelightOutput(int desiredOutput){
    
    bool errorState;
    
    float targetHorizontalDisplacement;
    float targetHeight;
    float targetArea; //in percent(1% = 1)    
    switch(desiredOutput){
      //outputting target x value
      case 0:
          targetHorizontalDisplacement = lltable->GetNumber("tx", 0);
          return targetHorizontalDisplacement;
          errorState = 0;
          break;
      //outputting target y value
      case 1:
          targetHeight = lltable->GetNumber("ty", 0);
          return targetHeight;
          errorState = 0;
          break;
      //outputting target area value(only used for inaccurate distance aquisition)
      case 2:
          targetArea = lltable->GetNumber("ta", 0);
          return targetArea;
          errorState = 0;
          break;
      case 3:
          
      default:
          errorState = 1;
          break;

    }
    frc::SmartDashboard::PutBoolean("Limelight Output Error?", errorState);
}
//Zeros hood and turret 
bool softStop(float max, float min, double motorInput, double motorPosition){
  if ((-motorInput >= max && motorPosition > 0) || (-motorInput <= min && motorPosition < 0)){
    return true;
  } else {
    return false;
  }
}

void setAll(double input) {
   
  bool AtTurretSoftStop;
    
  if (softStop(-turretMax, turretMax, input, turretPosition)){
    turret.Set(0);
    atTurretSoftStop = 1;
    return atTurretSoftStop;
  } else {
    turret.Set(PID(input-turretPosition, hoodKp, hoodKi));
    atTurretSoftStop = 0;
    return atTurretSoftStop;
  }

  frc::SmartDashboard::PutBoolean("Turret at soft stop?", atHoodSoftStop);
  
  bool atHoodSoftStop;
    
  if (softStop(hoodMax, hoodMin, input, hoodPosition)){
    hood.Set(0);
    atHoodSoftStop = 1;
    return atHoodSoftStop;
  } else {
    hood.Set(PID(hoodSetpoint-hoodPosition, hoodKp, hoodKi));
    atHoodSoftStop = 0;
    return atHoodSoftStop;
  }

  hood.Set(input);
  
  frc::SmartDashboard::PutBoolean("Hood at soft stop?", atSoftStop);
}
