#include "quadcopter.h"
#include <Arduino.h>

constexpr float PID_ROLL_KP = 1.0f;
constexpr float PID_ROLL_KI = 0.0f;
constexpr float PID_ROLL_KD = 0.0f;
constexpr float PID_ROLL_MAX = 100.0f;
constexpr float PID_ROLL_MIN = -100.0f;

constexpr float PID_PITCH_KP = 1.0f;
constexpr float PID_PITCH_KI = 0.0f;
constexpr float PID_PITCH_KD = 0.0f;
constexpr float PID_PITCH_MAX = 100.0f;
constexpr float PID_PITCH_MIN = -100.0f;

constexpr float PID_YAW_KP = 1.0f;
constexpr float PID_YAW_KI = 0.0f;
constexpr float PID_YAW_KD = 0.0f;
constexpr float PID_YAW_MAX = 100.0f;
constexpr float PID_YAW_MIN = -100.0f;

// --- Quadcopter ---
Quadcopter::Quadcopter(Imu& imuRef, Receiver& receiverRef, MotorMixer& motors) :
		m_state(QuadcopterState::IDLE),
		m_isRunning(false), 
		m_imu(imuRef),
		m_receiver(receiverRef),
		m_motors(motors),
		m_pidRoll(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, PID_ROLL_MAX, PID_ROLL_MIN),
		m_pidPitch(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PID_PITCH_MAX, PID_PITCH_MIN),
		m_pidYaw(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, PID_YAW_MAX, PID_YAW_MIN)
		{}

void Quadcopter::init() {
	// Initialize the receiver
	m_receiver.start();

	// Set initial motor outputs to zero
	m_motors.setOutputs(0.0f, 0.0f, 0.0f, 0.0f);

	// Start the IMU
	m_imu.updateAttitude();
}

void Quadcopter::startMainLoop() {

	while (m_isRunning) {
		m_imu.updateAttitude();

		switch (m_state) {
			case IDLE:
				break;
			case RUNNING:
				break;
			case TAKEOFF:
				break;
			case LANDING:
				break;
			default:
				m_isRunning = false;
				break;
		}

		delay(200);
	}

}
