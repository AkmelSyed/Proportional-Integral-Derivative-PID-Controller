#include "PID.h"

using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  prev_cte = 0;
  /**
  The following 3 variables are used in the twiddle algorithm below. The way I
  derived the numbers was that I first tried to get the simulator to run fine
  with just my K variables (Kp, Ki and Kd) and no twiddle, thereafter I saw how
  the simulutor reacted when I would change the K variables by small numbers.
  I realized that changing them by 1/10 would change them a bit, but not to the
  point where it would make it uncomfortable and make the car go crazy. Hence I
  stuck with 0.1 as the initial values.
  **/
  p_error = 0.1;
  i_error = 0.1;
  d_error = 0.1;
  best_err = TotalError();
}

/**
The following two functions (UpdateCTE amd getEqn) are the PID portion of the
assignment (without the twiddle algorithm). After setting the right K variables
I saw that the algorithm worked as I expected. Tuning the initial parameters is
essential to see the implementation of PID to be working smoothly. I saw that
the car didn't overshoot much and it would quickly stabalize in the middle of
the road.
**/

void PID::UpdateCTE(double cte) {
  diff_cte = cte - prev_cte;
  prev_cte = cte;
  init_cte += cte;
}

double PID::getEqn() {
  return (-Kp*prev_cte) - (Ki*init_cte) - (Kd*diff_cte);
}

/**
Update the error for the PID algorithm via calling the twiddle algorithm
**/
void PID::UpdateError() {
  if(TotalError() > 0.001) {
    Twiddle(Kp, p_error);
    Twiddle(Ki, i_error);
    Twiddle(Kd, d_error);
  }
}

double PID::TotalError() {
  return p_error + i_error + d_error;
}

/**
Twiddle - Gradually improve the parameters for PID by small increments.
          As long as the error given by the algorithm is too big (>0.001),
          adjust the PID parameters.
**/
void PID::Twiddle(double K, double error) {
  K += error;
  double err = getEqn();

  if(err < best_err) {
    best_err = err;
    error *= 1.1;
  }
  else {
    K -= 2*error;
    err =  getEqn();

    if(err < best_err) {
      best_err = err;
      error *= 1.05;
    }
    else {
      K += error;
      error *= 0.95;
    }
  }
}
