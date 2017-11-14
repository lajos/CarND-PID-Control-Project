#include "Twiddle.h"
#include <iostream>

using namespace std;

Twiddle::Twiddle() {
	p.resize(3, 0);
	dp.resize(3, 0.5);
	dp[0] = 0.1;
	dp[1] = 0.01;
	dp[2] = 0.4;
	c_iter = 0;       // iteration counter
	tolerance = 0.05;
}

Twiddle::~Twiddle() {}

void Twiddle::setParams(const double& Kp, const double& Ki, const double& Kd) {
	p[0] = Kp;
	p[1] = Ki;
	p[2] = Kd;
}

void Twiddle::setDParams(const double& dKp, const double& dKi, const double& dKd) {
	dp[0] = dKp;
	dp[1] = dKi;
	dp[2] = dKd;
}

void Twiddle::step(double err) {
	cout << "twiddle step, iteration: " << c_iter << endl;
	//cout << "  state: " << state << endl;
	cout << "  error: " << err << endl;

	double sum_dp = 0;
	switch (state) {
		case Twiddle::Init:
			best_err = err;
			c_iter = 0;
			state = Twiddle::StartIter;
			break;
		case Twiddle::StartIter:
			for (auto n : dp) {
				sum_dp += n;
			}
			if (sum_dp < tolerance) {
				state = Twiddle::Done;
			} else {
				state = TwiddleState::ParamLoop;
				p_i = 0;
				step(err);
			}
			break;
		case Twiddle::ParamLoop:
			//if (p_i == 1) ++p_i;                          // disable (skip) i parameter tuning
			if (p_i == p.size()) {
				state = Twiddle::StartIter;
				++c_iter;
			} else {
				p[p_i] += dp[p_i];
				state = Twiddle::ParamTry;
				// wait for next step
			}
			break;
		case Twiddle::ParamTry:
			if (err < best_err) {
				best_err = err;
				dp[p_i] *= 1.1;
				++p_i;
				state = Twiddle::ParamLoop;
			} else {
				p[p_i] -= 2 * dp[p_i];
				// wait for next step
				state = Twiddle::ParamRetry;
			}
			break;
		case Twiddle::ParamRetry:
			if (err < best_err) {
				best_err = err;
				dp[p_i] *= 1.1;
			} else {
				p[p_i] += dp[p_i];
				dp[p_i] *= 0.9;
			}
			++p_i;
			state = Twiddle::ParamLoop;
			break;
		case Twiddle::Done:
			break;
		default:
			break;
	}

	cout << "    p:   " << p[0] << ", " << p[1] << ", " << p[2] << endl;
	cout << "    dp:  " << dp[0] << ", " << dp[1] << ", " << dp[2] << endl;
}

