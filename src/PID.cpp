#include "PID.h"

/**
 *  Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  return - p_error * Kp - i_error * Ki - d_error * Kd;  // TODO: Add your total error calc here!
}