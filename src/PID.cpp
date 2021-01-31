#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  // Initialize all coefficients to input of this function
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;

  // Initialize all errors to 0
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  prev_cte = 0.0;
}

void PID::UpdateError(double cte, double i_err_limit) {
  // p error is same as the cte
  p_error = cte;

  // d error is the difference between the current cte and previous cte
  d_error = cte - prev_cte;

  // i error is the accumulation of cte
  i_error += cte;
  // i error builds up as there's no reset. To avoid that, I put a limit.
  // I parameterized it since I'm using same PID class for speed and steeering
  // PIDs. 
  if(i_error > i_err_limit)
  {
    i_error = i_err_limit;
  }
  else if(i_error < (-1*i_err_limit))
  {
    i_error = -1*i_err_limit;
  }

  prev_cte = cte;
}

double PID::TotalError() {
  // total error is the sum of all errors multiplied with their corresponding 
  // coefficients.
  double total = p_error*Kp + i_error*Ki + d_error*Kd;
  return total;  
}