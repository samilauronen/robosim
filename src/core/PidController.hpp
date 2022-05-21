#pragma once


class PidController {

public:
	PidController(float p, float i, float d);

	void setGains(float p, float i, float d);
	float update(float error, float dt);

private:
	float p_, i_, d_;
	float i_max_, max_;
	float integral_;
	float prev_error_;
	bool first_run_;
};