#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    prev_cte = 0.0;
  
    iter = 1;
    index = 0; // 1-Kp, 2=Ki, 3=Kd
    steps = 50;
    eval_steps =100; // steps+eval_steps=2*N from the lesson
    dp = {1.0,1.0,1.0};
    tol = 0.01;
    err = 0;
    best_err = std::numeric_limits<double>::max(); // initialize to max double value
    wait_flag = false;
}

void PID::UpdateError(double cte, double dt) {

	if ( iter == 1) {
		prev_cte = cte;
	}

	double diff_cte = cte - prev_cte;
	p_error = cte;
	i_error += cte;
	//i_error += cte * dt;
	d_error = diff_cte;
	//d_error = diff_cte / dt;

	prev_cte = cte;

	int eval_a = iter % steps;
	int eval_b = iter % (steps+eval_steps);

	if ( eval_b > steps) {
		err += cte * cte;
	}

	//cout <<  "iter: " << std::to_string(iter) << " Kp: " << Kp << " | Kd: " << Kd << " | Ki: " << Ki  << endl;
	cout <<  ">>iter: " << std::to_string(iter) << " p_err: " << p_error << " | i_err: " << i_error << " | d_err : " << d_error  << endl;
	if ( iter == (steps + eval_steps)) {
		cout << "best err : " << err << endl;
	}
	iter++;
}

double PID::TotalError() {
	double totalErr = -Kp * p_error  -Kd * d_error -Ki * i_error;
	//cout << "TotalError : " << totalErr << endl;
	return totalErr;
}