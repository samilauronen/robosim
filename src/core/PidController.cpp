#include "PidController.hpp"

#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

PidController::PidController(float p, float i, float d)
	: p_(p), i_(i), d_(d), integral_(0), prev_error_(0), first_run_(true), i_max_(0.5), max_(0.2)
{
}

void PidController::setGains(float p, float i, float d)
{
	p_ = p;
	i_ = i;
	d_ = d;
}

float PidController::update(float error, float dt)
{
	float output = 0;
	
	// proportional term
	float proportional = error * p_;

	// integral term, clamped
	integral_ += error * dt;
	if (integral_ > i_max_) integral_ = i_max_;
	else if (integral_ < -i_max_) integral_ = -i_max_;
	float integral =  i_ * integral_;

	// derivative term
	float derivative = d_ * (error - prev_error_) / dt;
	prev_error_ = error;

	// ignore derivative on the first run, since there is no previous error
	if (first_run_)
	{
		output = proportional + integral;
		first_run_ = false;
	}
	else
	{
		output = proportional + integral + derivative;
	}

	// clamp output
	if (output > max_) output = max_;
	else if (output < -max_) output = -max_;

	return output;
}
