#include "PID.h"
#include <uWS/uWS.h>

// https://discussions.udacity.com/t/twiddle-application-in-pid-controller/243427/27
// https://discussions.udacity.com/t/how-to-tune-parameters/303845/7
// https://discussions.udacity.com/t/how-to-tune-parameters/303845/14
// https://discussions.udacity.com/t/how-to-tune-parameters/303845/4

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {
}

PID::~PID() {
}

void PID::Init(double Kp, double Ki, double Kd, double cte) {
  this->Kp = Kp; 
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 1;
  i_error = 1;
  d_error = 1;

  this->cte = cte;
  sum_cte = 0;
  time_step = 0;
  total_error = 0;
}

double PID::getSteering(double cte) {
  double cte_difference = cte - this->cte;
  double steering = -Kp * cte - Kd * cte_difference - Ki * sum_cte;
  this->cte = cte;

  if (steering > 1) {
    return 1;
  }
  if (steering < -1) {
    return -1;
  }

  return steering; total_error = 6;
}

void PID::UpdateError(double cte) {
  sum_cte += cte * cte;
  time_step++;
  total_error = sum_cte / time_step;
}

double PID::GetTotalError() {
  return total_error;
}

double PID::TotalError() {
  return 0.;
}

void PID::ResetTotalError() {
  sum_cte = 0;
  time_step = 0;
  total_error = 0;
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
  ResetTotalError();
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}

