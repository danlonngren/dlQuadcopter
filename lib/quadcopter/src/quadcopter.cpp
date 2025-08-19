#include "quadcopter.h"
#include <Arduino.h>


// --- Quadcopter ---
Quadcopter::Quadcopter(Imu& imuRef, Receiver& receiverRef, std::array<Motor, 4>& motor) :
		m_isRunning(false), 
		m_imu(imuRef), 
		m_receiver(receiverRef),
		m_motors(motor) {
	
}

void Quadcopter::startMainLoop() {

	while (m_isRunning) {
		m_imu.updateAttitude();

		switch (state) {
			case IDLE:
				break;
			case RUNNING:
				break;
			case TAKEOFF:
				break;
			default:
				m_isRunning = false;
				break;
		}

		delay(200);
	}

}
