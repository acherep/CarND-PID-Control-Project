#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

class PID {
 public:
  /*
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;

  double dp;

  uWS::WebSocket<uWS::SERVER> ws;

  /*
   * Coefficients
   */
  double Kp;
  double Ki;
  double Kd;
  double cte;
  double sum_cte;
  int time_step;
  double previous_error;
  double total_error;
  double best_error;

  bool is_first_run;
  bool is_dp_decreased;
  int direction;

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
  void Init(double Kp, double Ki, double Kd, double cte);

  void InitWS(uWS::WebSocket<uWS::SERVER> ws);

  /*
   * Calculates Steering.
   */
  double getSteering(double cte);

  void UpdateCoefficient();

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double cte);

  double GetTotalError();

  /*
   * Calculate the total PID error.
   */
  double TotalError();

  void Reset();
  /*
   * Restart the simulator
   */
  void RestartSimulator();
};

#endif /* PID_H */
