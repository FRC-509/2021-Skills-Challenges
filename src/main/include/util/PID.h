#pragma once

double PID(double error, double Kp, double Ki);
bool softStop(float max, float min, double motorInput, double motorPosition);
