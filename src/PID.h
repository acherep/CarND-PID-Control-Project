#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

class PID {
 private:
  uWS::WebSocket<uWS::SERVER> ws;

  /*
   * Coefficients
   */
  double Kp;
  double Ki;
  double Kd;

  /*
   * Errors
   */
  double cte;
  double sum_cte;
  double sum_cte_cte;
  double total_error;
  double best_error;

  int time_step;

  bool is_first_run;

  int twiddle_direction;
  // twiddle increment
  double dp;

  void Reset();

  // Twiddle desired parameter
  void Twiddle();

 public:
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

  /*
   * Initialize Simulator
   */
  void InitWS(uWS::WebSocket<uWS::SERVER> ws);

  /*
   * Calculate Steering
   */
  double getSteering(double cte);

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double cte);

  /*
   * Return the total PID error.
   */
  double GetTotalError();

  /*
   * Restart the simulator
   */
  void RestartSimulator();
};

#endif /* PID_H */
