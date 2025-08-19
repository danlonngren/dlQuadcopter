#include "quadcopter.h"
#include <Arduino.h>

constexpr float PID_ROLL_KP = 0.7f;
constexpr float PID_ROLL_KI = 0.0f;
constexpr float PID_ROLL_KD = 0.3f;
constexpr float PID_ROLL_INT_MAX = 20.0f;
constexpr float PID_ROLL_MAX = 100.0f;

constexpr float PID_PITCH_KP = PID_ROLL_KP;
constexpr float PID_PITCH_KI = PID_ROLL_KI;
constexpr float PID_PITCH_KD = PID_ROLL_KD;
constexpr float PID_PITCH_INT_MAX = PID_ROLL_INT_MAX;
constexpr float PID_PITCH_MAX = PID_ROLL_MAX;

constexpr float PID_YAW_KP = PID_ROLL_KP;
constexpr float PID_YAW_KI = PID_ROLL_KI;
constexpr float PID_YAW_KD = 0.0f;
constexpr float PID_YAW_INT_MAX = PID_ROLL_INT_MAX;
constexpr float PID_YAW_MAX = PID_ROLL_MAX;

constexpr uint32_t MAIN_LOOP_DELAY = 200; // Main loop delay in milliseconds

constexpr uint8_t MOTOR_PIN_1 = 0;
constexpr uint8_t MOTOR_PIN_2 = 1;
constexpr uint8_t MOTOR_PIN_3 = 2;
constexpr uint8_t MOTOR_PIN_4 = 3;

constexpr uint8_t RECEIVER_PIN = 11;

constexpr uint8_t RECEVIER_CH_TROTTLE = 0;
constexpr uint8_t RECEVIER_CH_ROLL = 0;
constexpr uint8_t RECEVIER_CH_PITCH = 0;
constexpr uint8_t RECEVIER_CH_YAW = 0;

constexpr bool ENABLE_LOGGING = false; 

// --- Quadcopter ---
Quadcopter::Quadcopter() :
		m_state(QuadcopterState::IDLE),
		m_isRunning(true),
		m_imu(),
		m_receiver(RECEIVER_PIN, true, 2500), // Assuming CPPM mode with 2000us max pulse width
		m_motors({Motor(MOTOR_PIN_1), Motor(MOTOR_PIN_2), Motor(MOTOR_PIN_3), Motor(MOTOR_PIN_4)}),
		m_pidRoll(	PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, PID_ROLL_INT_MAX, PID_ROLL_MAX),
		m_pidPitch(	PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PID_PITCH_INT_MAX, PID_PITCH_MAX),
		m_pidYaw(	PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, PID_YAW_INT_MAX, PID_YAW_MAX)
		{}

void Quadcopter::init() {
	if (ENABLE_LOGGING)
		Serial.println("Quadcopter initialization started");

	m_motors.init(); // Initialize motors

	m_imu.init(); // Initialize IMU

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
	float dt = MAIN_LOOP_DELAY / 1000.0f; // Convert milliseconds to seconds

	while (m_isRunning) {
		// Read sensor data
		m_imu.updateAttitude();

		// Get receiver input
		uint32_t throttle = m_receiver.getCPPMChannel(RECEVIER_CH_TROTTLE); // Assuming channel 0 is used for throttle

		// Calculate PID outputs
		uint32_t rollInput = m_receiver.getCPPMChannel(RECEVIER_CH_ROLL); // Assuming channel 1 is used for roll
		uint32_t pitchInput = m_receiver.getCPPMChannel(RECEVIER_CH_PITCH); // Assuming channel 2 is used for pitch
		uint32_t yawInput = m_receiver.getCPPMChannel(RECEVIER_CH_YAW); // Assuming channel 3 is used for yaw
		
		// Evaluate PID controllers
		float throttleOutput = throttle; // Directly use throttle input
		float rollOutput = m_pidRoll.evaluate(m_imu.getAttitude().rpy.roll, rollInput, dt);
		float pitchOutput = m_pidPitch.evaluate(m_imu.getAttitude().rpy.pitch, pitchInput, dt);
		float yawOutput = m_pidYaw.evaluate(m_imu.getAttitude().rpy.yaw, yawInput, dt);
				
		for (int i = 0; i < 8; i++) {
			Serial.print("Channel[");
			Serial.print(i);
			Serial.print("]:");
			Serial.print(m_receiver.getCPPMChannel(i));
			Serial.print(", ");
		}
		Serial.println();

		if (ENABLE_LOGGING) {
			Serial.print("throttle = " + String(throttle, 2) + ", ");
			Serial.print("rollInput = " + String(rollInput, 2) + ", ");
			Serial.print("pitchInput = " + String(pitchInput, 2) + ", ");
			Serial.print("yawInput = " + String(yawInput, 2) + ", ");
			Serial.print("Roll = " + String(m_imu.getAttitude().rpy.roll, 2) + ", ");
			Serial.print("Pitch = " + String(m_imu.getAttitude().rpy.pitch, 2) + ", ");
			Serial.print("Yaw = " + String(m_imu.getAttitude().rpy.yaw, 2) + ", ");
			Serial.print("rollOutput = " + String(rollOutput, 2) + ", ");
			Serial.print("pitchOutput = " + String(pitchOutput, 2) + ", ");
			Serial.println("yawOutput = " + String(yawOutput, 2));
		}

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
