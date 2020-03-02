#include "PID.h"
#include <algorithm>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
}

void PID::UpdateError(double cte)
{
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error_ = (cte - p_error_) / 1; //delta_t is;
  p_error_ = cte;
  i_error_ = cte + i_error_;
}

double PID::ComputePid()
{
  /**
   * TODO: Calculate and return the PID formula
   */
  double u{-Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_};

  return std::min({1.0, std::max({-1.0, u})});
}