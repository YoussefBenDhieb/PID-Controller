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
  * Total error
  */ 
  double t_cte;

  /*
  * Best error
  */ 
  double best_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle parameters
  */ 
  double dKp;
  double dKi;
  double dKd;

  /*
  * Counter of number of iterations for the twiddle
  */ 
  int c;

/*
  * Iterator for the twiddle to switch between kp, kd and ki
  */ 
  int it;

  /*
  * Number of iterations to call twiddle function
  */ 
  const int N_ITER = 200;

  /*
  * Throttle
  */ 
  double throttle;

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

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Returns the throttle.
  */
  double GetThrottle();

  /*
  * Calculate the result : steering value.
  */
  double Run();

  /*
  * Twiddle function.
  */
  void Twiddle();
};

#endif /* PID_H */
