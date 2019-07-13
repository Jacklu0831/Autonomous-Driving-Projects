
#include "PID.h"

PID::PID() {
  Init(0, 0, 0);
}

PID::~PID() {
}

void PID::Init(double Kp, double Ki, double Kd) {
  // initialize coefficients pid
  K_p = Kp;
  K_i = Ki;
  K_d = Kd;

  // start with zero errors
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  // new errors
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return -p_error * K_p - i_error * K_i - d_error * K_d;
}
