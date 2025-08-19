#pragma once

// Components
#include "imu.h"
#include "simplePIDController.h"
#include "motor.h"
#include "receiver.h"

enum QuadcopterState {
    IDLE,
    RUNNING,
	  TAKEOFF,
    LANDING,
};

// Main control algorithm
class Quadcopter {

public:
	Quadcopter(Imu& imuRef, Receiver& receiverRef, MotorMixer& motor);

  void init();

	void startMainLoop();

private:
  	QuadcopterState m_state;

    bool m_isRunning;
    Imu& m_imu;
    Receiver& m_receiver;
    MotorMixer& m_motors;
    SimplePIDController m_pidRoll;
    SimplePIDController m_pidPitch;
    SimplePIDController m_pidYaw;
};
