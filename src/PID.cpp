#include "PID.h"

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

  this->cte = cte;
}

double PID::getSteering(double cte) {
  double cte_difference = cte - this->cte;
  this->cte = cte;
  return -Kp * cte - Kd * cte_difference;
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}

