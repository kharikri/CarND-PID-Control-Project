#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp = 0.0;
	Ki = 0.0;
	Kd = 0.0;
}

void PID::UpdateError(double cte) {
	    p_error = cte;
        d_error = cte - prev_cte
        i_error += cte
        prev_cte = cte
}

double PID::TotalError() {
	return (p_error + i_error + d_error);
}

