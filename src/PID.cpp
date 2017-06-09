#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->is_intialized = false;
    this->N = 0;
    this->total_error = 0;
}

void PID::UpdateError(double cte) {
    if (!is_intialized) {
        p_error = cte;
        i_error = cte;
        d_error = cte;
        total_error += cte * cte;
        N++;
        is_intialized = true;
        return;
    }
    d_error = cte - this->p_error;
    i_error += cte;
    p_error = cte;
    total_error += cte * cte;
    N++;
}

double PID::TotalError() {
    return (total_error / N);
}

double PID::ComputeSteerValue() {
    double steer_value;
    steer_value = - Kp * p_error - Kd * d_error - Ki * i_error;
    return steer_value;
}
