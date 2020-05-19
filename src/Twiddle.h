#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <cfloat>
#include <iostream>


class Twiddle
{
private:

  /* This function performs the runing of the ith hyper-parameter indexed by
   * idx
   */
  void ParamTuner(int idx, int steps,bool is_complete_track);

 /* This array holds the best param values obtained till now */
 double best_params[3];

  /* This array holds the differences in params */
  double dp_params[3];

  /* This holds the parameter index being currently optimized*/
  int idx_opt{0};

  /* This holds the status of the first breakout stage of tuning the ith param */
  bool st1_flag{false};

  /* This holds the status of the second breakout stage of tuning the ith param */
  bool st2_flag{false};

  /* This param stores the best error achieved till now */
  double best_error{DBL_MAX};

 public:

 /* This array holds the values configured for all
  * the 3 hyper-params
  */
  double params[3];

  /* This param stores the current error */
  double current_error{0.0};

  /* Used to set the current error achieved till now */
  void SetError(double cte);

  /* Used to set the best error */
  void SetBestError(int steps);

  /* Used to set the current error achieved till now */
  void ResetError(void);

  /* Train the PID controller to learn the hyperparamters */
  int HypTuning(int steps, bool is_complete_track);
  
  /*Print Twiddle DP Params */
  void PrintDPParams();
  
  /*Print Twiddle Best params*/
  void PrintBestParams();

  /* constructor */
  Twiddle() {}

  /* Destructor */
  ~Twiddle() {}

  /* Initialize */
  void Init(double Kp_, double Ki_, double Kd_)
  {
    params[0] =  Kp_;
    params[1] = Ki_;
    params[2] = Kd_;
    best_params[0] =  Kp_;
    best_params[1] = Ki_;
    best_params[2] = Kd_;

    /* These do_params were chosen based on the variety of values for
     * each paramter that worked during manual tuning. The idea here was
     * to allow Twiddle converge to values in the neighbourhood of manually
     * determined parameter values. Keeping these too high such would cause Twiddle
     * to take forever.
     */
    dp_params[0] = 0.2;
    dp_params[1] = 0.001;
    dp_params[2]= 0.75;
  }

};

#endif /* TWIDDLE_H */