#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>

class Twiddle {
public:
	std::vector<double> p;
	std::vector<double> dp;

	double best_err;

	// iteration count
	size_t c_iter;

	// parameter index
	size_t p_i;

	double tolerance;

	enum TwiddleState {
		Init,
		StartIter,
		ParamLoop,
		ParamTry,
		ParamRetry,
		Done
	};

	Twiddle();
	virtual ~Twiddle();

	TwiddleState state;

	void step(double err);

	void setParams(const double& Kp, const double& Ki, const double& Kd);
	void setDParams(const double& dp, const double& di, const double& dd);

};

#endif
