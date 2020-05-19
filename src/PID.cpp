#include "PID.h"


/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  d_error = 0;
  p_error = 0;
  i_error = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error =  cte - p_error;
  p_error =  cte;
  i_error += cte; 
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  // TODO: Add your total error calc here!
  return -Kp * p_error - Kd * d_error - Ki * i_error;  
}

void PID::ResetError()
{
  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;
}

void PID::SetTwiddleParams(Twiddle& tw)
{
  Kp = tw.params[0];
  Ki = tw.params[1];
  Kd = tw.params[2];
}

