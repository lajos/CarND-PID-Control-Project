#include "Twiddle.h"
#include <iostream>

using namespace std;


/*
def twiddle(tol=0.2):

    p = [0, 0, 0]
    dp = [1, 1, 1]
---state:init
	robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

---state:start_iter
    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
---state:param_loop
        for i in range(len(p)):
---state:param_try
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
---state:param_retry
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p, best_err
*/

Twiddle::Twiddle() {
	p.resize(3, 0);
	dp.resize(3, 0.5);
	dp[0] = 0.1;
	dp[1] = 0.01;
	dp[2] = 0.4;

	tolerance = 0.1;
}

Twiddle::~Twiddle() {}

void Twiddle::setParams(const double& Kp, const double& Ki, const double& Kd) {
	p[0] = Kp;
	p[1] = Ki;
	p[2] = Kd;
}

void Twiddle::step(double err) {
	cout << "twiddle step";
	cout << "  state: " << state << endl;
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
			if (p_i == p.size()) {
				state = Twiddle::StartIter;
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

