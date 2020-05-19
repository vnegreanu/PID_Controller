#include <iostream>
#include "Twiddle.h"

using namespace std;

void Twiddle::PrintDPParams()
{
  std::cout<<"Twiddle DP params now are:"<<std::endl;
  std::cout<<dp_params[0]<<" "<<dp_params[1]<<" "<<dp_params[2]<<std::endl;
}

void Twiddle::PrintBestParams()
{
  std::cout<<"Twiddle best params now are:";
  std::cout<<best_params[0]<<" "<<best_params[1]<<" "<<best_params[2]<<std::endl;
}


void Twiddle::SetError(double cte)
{
  current_error += cte*cte;
}

 void Twiddle::SetBestError(int steps)
{
   if (steps!=0) {
     best_error = current_error/steps;
   }
   else {
     best_error = 0;
   }
}

void Twiddle::ResetError(void)
{
  current_error = 0;
}

int Twiddle::HypTuning(int steps, bool is_complete_track)
{
  double sum_p = 0;

  for(int i=0; i < 3; i++)
  {
    sum_p += dp_params[i];
  }

  if( sum_p < 0.002 ) return 0;
  else
  {
    ParamTuner( idx_opt,steps,is_complete_track);
    return -1;
  }
}

void Twiddle::ParamTuner(int idx, int steps,bool is_complete_track)
{
  cout<<"Current idx is:"<<idx<<"\n";
  if(!st1_flag)
  {
    params[idx] += dp_params[idx];
    ResetError();
    st1_flag = true;
    return;
  }

  if( ( current_error/steps < best_error ) && ( is_complete_track == true ) )
  {
    SetBestError(steps);

    /* Save the best params */
    best_params[0] = params[0];
    best_params[1] = params[1];
    best_params[2] = params[2];

    dp_params[idx]*= 1.1;
    st1_flag = false;
    idx_opt = idx_opt+1;

    if( idx_opt <= 2)
    {
      ParamTuner(idx_opt,steps,is_complete_track);
    }
    else
    {
      idx_opt = 0;
      ResetError();
      return;
    }

  }
  else
  {
    if(!st2_flag)
    {
      params[idx] -= 2*dp_params[idx];
      ResetError();
      st2_flag = true;
      return;
    }

    /* Only treat the average error as valid in order to compare it with the
     * last recorded best error if the car has been able to complete 1150 steps
     * i.e. the complete track, otherwise a pentalty is imposed in the sense that
     * the current error is treated as implicitly worse than the best error
     */
    if( ( current_error/steps < best_error ) && ( is_complete_track == true ) )
    {
      SetBestError(steps);

      /* Save the best params */
      best_params[0] = params[0];
      best_params[1] = params[1];
      best_params[2] = params[2];
      dp_params[idx] *= 1.1;
    }
    else
    {
      params[idx] += dp_params[idx];
      dp_params[idx] *= 0.9;
    }

    st1_flag = false;
    st2_flag = false;
    idx_opt = idx_opt+1;

    if( idx_opt <= 2)
    {
      ParamTuner(idx_opt,steps,is_complete_track);
    }
    else
    {
      idx_opt = 0;
      ResetError();
      return;
    }
  }

}