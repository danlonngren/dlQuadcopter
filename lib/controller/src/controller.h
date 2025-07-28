#ifndef controller_h
#define controller_h

// Components
#include "imu.h"
#include "motor.h"
#include "receiver.h"

// Main control algorithm
class Controller {
public:
	Controller(Imu& imuRef, Receiver& receiverRef, std::array<Motor, 4>& motor);

	void startMainLoop();
private:
	bool m_isRunning;
	Imu& m_imu;
  Receiver& m_receiver;
	std::array<Motor, 4>& m_motors;
};


class PID {
  private:
    float pid_output;
    float error_p, error_i, error_d, error_p_last;
    float p_gain, i_gain, d_gain, pid_max;

  public:
    PID(float p_gain, float i_gain, float d_gain, int output_max);

    float Controller(float gyro_input, float setpoint);
    void setPidGains(float p_gain, float i_gain, float d_gain);
    void setOutputMax(float newOutputMax);
    void setGainP(float p_gain);
    void setGainI(float i_gain);
    void setGainD(float d_gain);
    void Reset();
};

#endif
