#include "quadcopter.h"
#include <Arduino.h>

constexpr float PID_ROLL_KP = 1.0f;
constexpr float PID_ROLL_KI = 1.0f;
constexpr float PID_ROLL_KD = 1.0f;
constexpr float PID_ROLL_MAX = 100.0f;
constexpr float PID_ROLL_MIN = -100.0f;

constexpr float PID_PITCH_KP = 1.0f;
constexpr float PID_PITCH_KI = 1.0f;
constexpr float PID_PITCH_KD = 1.0f;
constexpr float PID_PITCH_MAX = 100.0f;
constexpr float PID_PITCH_MIN = -100.0f;

constexpr float PID_YAW_KP = 1.0f;
constexpr float PID_YAW_KI = 1.0f;
constexpr float PID_YAW_KD = 1.0f;
constexpr float PID_YAW_MAX = 100.0f;
constexpr float PID_YAW_MIN = -100.0f;

constexpr uint32_t MAIN_LOOP_DELAY = 200; // Main loop delay in milliseconds

constexpr uint8_t MOTOR_PIN_1 = 1;
constexpr uint8_t MOTOR_PIN_2 = 2;
constexpr uint8_t MOTOR_PIN_3 = 3;
constexpr uint8_t MOTOR_PIN_4 = 4;

constexpr uint8_t RECEIVER_PIN = 3;

constexpr uint8_t RECEVIER_CH_TROTTLE = 0;
constexpr uint8_t RECEVIER_CH_ROLL = 0;
constexpr uint8_t RECEVIER_CH_PITCH = 0;
constexpr uint8_t RECEVIER_CH_YAW = 0;

// --- Quadcopter ---
Quadcopter::Quadcopter() :
		m_state(QuadcopterState::IDLE),
		m_isRunning(false),
		m_imu(),
		m_receiver(RECEIVER_PIN),
		m_motors({Motor(MOTOR_PIN_1), Motor(MOTOR_PIN_2), Motor(MOTOR_PIN_3), Motor(MOTOR_PIN_4)}),
		m_pidRoll(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, PID_ROLL_MAX, PID_ROLL_MIN),
		m_pidPitch(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PID_PITCH_MAX, PID_PITCH_MIN),
		m_pidYaw(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, PID_YAW_MAX, PID_YAW_MIN)
		{}

void Quadcopter::init() {
	// Initialize the receiver
	m_receiver.start();

	// Set initial motor outputs to zero
	m_motors.setOutputs(0.0f, 0.0f, 0.0f, 0.0f);

	// Initialize PID controllers
	m_pidPitch.reset();
	m_pidRoll.reset();
	m_pidYaw.reset();

	// Start the IMU
	m_imu.updateAttitude();
}

void Quadcopter::startMainLoop() {

	while (m_isRunning) {
		// Read sensor data
		m_imu.updateAttitude();

		// Get receiver input
		float throttle = m_receiver.getCPPMChannel(RECEVIER_CH_TROTTLE) / 1000.0f; // Assuming channel 0 is used for throttle

		// Calculate PID outputs
		float rollInput = m_receiver.getCPPMChannel(RECEVIER_CH_ROLL) / 1000.0f; // Assuming channel 1 is used for roll
		float pitchInput = m_receiver.getCPPMChannel(RECEVIER_CH_PITCH) / 1000.0f; // Assuming channel 2 is used for pitch
		float yawInput = m_receiver.getCPPMChannel(RECEVIER_CH_YAW) / 1000.0f; // Assuming channel 3 is used for yaw

		// Evaluate PID controllers
		float throttleOutput = throttle; // Directly use throttle input
		float rollOutput = m_pidRoll.evaluate(m_imu.getAttitude().rpy.roll, rollInput, MAIN_LOOP_DELAY / 1000.0f);
		float pitchOutput = m_pidPitch.evaluate(m_imu.getAttitude().rpy.pitch, pitchInput, MAIN_LOOP_DELAY / 1000.0f);
		float yawOutput = m_pidYaw.evaluate(m_imu.getAttitude().rpy.yaw, yawInput, MAIN_LOOP_DELAY / 1000.0f);

		switch (m_state) {
			case IDLE:
				// Transition to RUNNING state if throttle is above threshold
				// TODO: Add startup sequence or checks
				if (throttle > 0.1f) {
					m_state = RUNNING;
					m_isRunning = true;
				}
				break;
			case RUNNING:
				// Set motor outputs based on PID results
				m_motors.setOutputs(throttleOutput, rollOutput, pitchOutput, yawOutput);
				break;
			case TAKEOFF:
				// Set motor outputs based on PID results
				m_motors.setOutputs(throttleOutput, rollOutput, pitchOutput, yawOutput);
				break;
			case LANDING:
				// Set motor outputs based on PID results
				m_motors.setOutputs(throttleOutput, rollOutput, pitchOutput, yawOutput);
				break;
			default:
				m_isRunning = false;
				break;
		}

		delay(MAIN_LOOP_DELAY);
	}

}
