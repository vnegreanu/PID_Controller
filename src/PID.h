#ifndef PID_H
#define PID_H

#include "Twiddle.h"

class PID {
 public:
  
  /* To keep track of setting the best error the first time hyperparamter tuning kicks off */
  bool isfirstTuned{false};
  
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  
  /**
   * Reset the Errors.
   * @output Reset the Error definitions
   */
  void ResetError();
     
  /**
   * Sets the hyperparameters to values
   * obtained by Twiddling
   */
  void SetTwiddleParams(Twiddle& tw);



 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
};

#endif  // PID_H