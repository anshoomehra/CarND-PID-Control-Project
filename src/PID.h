#ifndef PID_H
#define PID_H
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;
  

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * For Twiddle
  */
  int index; // to keep track of the tune params
  std::vector<double> dp;
  int iter; // for calculating steps during the lap. 
  int steps; // for counting steps before start capturing err
  int eval_steps; // after params have settled, capture new error
  bool wait_flag; // used for second "else" inside first "else" in twiddle algorithm
  bool tw_decrease;
  double tol; // tolerance
  double err;
  double best_err;



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
  void UpdateError(double cte, double dt);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */