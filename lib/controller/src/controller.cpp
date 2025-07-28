#include "controller.h"

static float limiter(float in, float max, float min)
{
	if (in >= max)
		return max;
	else if (in <= min)
		return min;
	else
		return in;
}

PID::PID(float p_gain, float i_gain, float d_gain, int pid_max)
{
	this->p_gain = p_gain;
	this->i_gain = i_gain;
	this->d_gain = d_gain;
	this->pid_max = pid_max;
}

void PID::Reset()
{
	this->error_p = 0;
	this->error_d = 0;
	this->error_i = 0;
	this->pid_output = 0;
}

float PID::Controller(float gyro_input, float setpoint)
{
	error_p = setpoint - gyro_input;
	error_i += i_gain * error_p;

	error_i = limiter(error_i, pid_max, (-1) * pid_max);
	error_d = error_p - error_p_last;

	pid_output = (p_gain * error_p) + error_i + (d_gain * error_d);
	pid_output = limiter(pid_output, pid_max, (-1) * pid_max);
	error_p_last = error_p;

	return pid_output;
}

void PID::setPidGains(float p_gain, float i_gain, float d_gain)
{
	this->p_gain = p_gain;
	this->i_gain = i_gain;
	this->d_gain = d_gain;
}
void PID::setOutputMax(float pid_max)
{
	this->pid_max = pid_max;
}
void PID::setGainP(float p_gain)
{
	this->p_gain = p_gain;
}
void PID::setGainI(float i_gain)
{
	this->i_gain = i_gain;
}
void PID::setGainD(float d_gain)
{
	this->d_gain = d_gain;
}
