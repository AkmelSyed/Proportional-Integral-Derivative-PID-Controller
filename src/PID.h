#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  double best_err;
  double init_cte;
  double diff_cte;
  double prev_cte;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  void UpdateCTE(double cte);

  double getEqn();

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError();

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Find the optimal K values for the PID algorithm 
  */
  void Twiddle(double K, double error);
};

#endif /* PID_H */
