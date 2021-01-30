double PID(double error, double Kp, double Ki){
  double p;
  double i;
  double integral;

  integral += error;
  p = Kp*error;
  i = Ki*integral; 
  return p+i;
}
//Soft Stop Function (true should stop)
bool softStop(float max, float min, double motorInput, double motorPosition){
  if ((-motorInput >= max && motorPosition > 0) || (-motorInput <= min && motorPosition < 0)){
    return true;
  } else {
    return false;
  }
}