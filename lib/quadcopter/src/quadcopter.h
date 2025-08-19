#pragma once

// Components
#include "imu.h"
#include "motor.h"
#include "receiver.h"

enum QuadcopterState {
    IDLE,
    RUNNING,
	TAKEOFF
};

// Main control algorithm
class Quadcopter {
private:
	QuadcopterState state;

public:
	Quadcopter(Imu& imuRef, Receiver& receiverRef, std::array<Motor, 4>& motor);

	void startMainLoop();

private:
	bool m_isRunning;
	Imu& m_imu;
  	Receiver& m_receiver;
	std::array<Motor, 4>& m_motors;
};
