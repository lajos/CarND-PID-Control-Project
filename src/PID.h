#ifndef PID_H
#define PID_H

class PID {
public:
	/*
	* Errors
	*/
	double p_error;
	double i_error;
	double d_error;

	/*
	* Coefficients
	*/
	double Kp;
	double Ki;
	double Kd;

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
	void UpdateError(const double& cte, const double& dt);

	/*
	* Calculate the total PID error.
	*/
	double TotalError(const double& dt);
};

#endif /* PID_H */
