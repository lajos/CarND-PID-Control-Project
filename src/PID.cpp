#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	i_error = 0;
	d_error = 0;
}

void PID::UpdateError(const double& cte, const double& dt) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte * dt;
}

double PID::TotalError(const double& dt) {
	return -Kp * p_error - Kd * d_error / dt - Ki * i_error;
}

