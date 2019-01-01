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
  double total_err;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double dKp;
  double dKi;
  double dKd;
  double dp[3];

  int turn_index;

  bool twiddle_switch;

  /*
  *	total err counts
  */
  int  total_err_count;
  /*
  * twiddle step
  */
  int twiddle_step;

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
  //void Init(double Kp, double Ki, double Kd);
  void Init(double Kp, double Ki, double Kd, bool twiddle);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);
  double calSteerValue();

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double avgError();
  void resetError();

  /*
  * turn coef by twiddle
  */
  void twiddle_1();
  void twiddle_2(double &best_err);

  /*
  * print Coefficients
  */
  void printCoef();

  
};

#endif /* PID_H */
