#include "PID.h"
#include <uWS/uWS.h>

// https://discussions.udacity.com/t/twiddle-application-in-pid-controller/243427/27
// https://discussions.udacity.com/t/how-to-tune-parameters/303845/7
// https://discussions.udacity.com/t/how-to-tune-parameters/303845/14
// https://discussions.udacity.com/t/how-to-tune-parameters/303845/4
// https://www.codeproject.com/Articles/1184735/Quick-Start-to-Use-Visual-Studio-Code-for-Cplusplu

// VS Code Google formatter
// https://stackoverflow.com/questions/45823734/vs-code-formatting-for

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double cte) {
  is_first_run = true;
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 1;
  i_error = 1;
  d_error = 1;
  dp = 0.01;

  this->cte = cte;
  sum_cte = 0;
  time_step = 0;
  total_error = 0;

  is_dp_decreased = false;
  direction = 0;
}

void PID::InitWS(uWS::WebSocket<uWS::SERVER> ws) { this->ws = ws; }

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

  return steering;
}

void PID::UpdateError(double cte) {
  if (time_step > 30) {
    sum_cte += cte * cte;
    total_error = sum_cte / (time_step - 30);
  }

  time_step++;
  cout << "Time_step: " << time_step << endl;

  if (time_step == 250) {
    if (is_first_run) {
      best_error = total_error;
      is_first_run = false;
      Kp += dp;
      direction = 1;
    } else {
      if (direction == 1) {
        if (total_error < best_error) {
          // dp *= 1.1;
          best_error = total_error;
          Kp += dp;
        } else {
          Kp -= 2 * dp;
          direction = -1;
        }
      } else {
        if (total_error < best_error) {
          // dp *= 1.1;
          best_error = total_error;
          Kp -= dp;
          direction = 1;
        } else {
        }
      }
      // if (total_error >= best_error) {
      //   if (direction == 1) {
      //     Kp -= 2 * dp;
      //     direction = -1;
      //   } else {
      //     if (total_error >= best_error) {
      //       Kp += dp;
      //       dp *= 0.9;
      //     } else {
      //       best_error = total_error;
      //     }
      //   }
      // } else {
      //   best_error = total_error;
      //   dp *= 1.1;
      //   Kp += dp;
      // }
    }
    // if (total_error < best_error) {
    //   best_error = total_error;
    //   dp *= 1.1;
    //   Kp += dp;
    // } else {
    //   if (!is_dp_decreased) {
    //     Kp -= dp;
    //     is_dp_decreased = true;
    //   } else {
    //     Kp += dp;
    //     dp *= 0.9;
    //     Kp += dp;
    //     is_dp_decreased = false;
    //   }
    // }
    cout << "Kp: " << Kp << ", dp: " << dp << endl;
    // update weights
    // Kp += dp;
    RestartSimulator();
  }
}

double PID::GetTotalError() { return total_error; }

double PID::TotalError() { return 0.; }

void PID::Reset() {
  sum_cte = 0;
  time_step = 0;
  total_error = 0;
}

void PID::RestartSimulator() {
  Reset();
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
