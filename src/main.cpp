#include <Arduino.h>

#include "utils.h"
#include "imu.h"

#include "board_registers.h"

#include "receiver.h"
#include "motor.h"

// #include "Adafruit_TinyUSB.h"

void setup()
{
	// put your setup code here, to run once:
	// utils util;			   // Create an instance of the utils class

	// TinyUSB_Device_Init(0);

	// Set up the serial communication
	Serial.begin(115200);
	// while (!Serial) {
	// 	// Wait for the serial port to connect. Needed for native USB port only
	// }
	Serial.println("Serial port connected");
}

void loop()
{
	Serial.println("Entering loop");
	
	Serial.println("calibrate");
	Imu myImu;			   // Create an instance of the imu class
	// myImu.calibrate(20); // Call the Calibrate method to calibrate the MPU6050
	
	Serial.println("receiver1");
	// Receiver receiver1(3); // Create an instance of the Receiver class
	// receiver1.start(); // Start the receiver
	
	// Serial.println("motor1");
	// Motor motor1(6, 1000.0, Motor::PwmResolution::PWM_8BIT); // Create an instance of the Motor class
	// motor1.setMotorSpeed(80); // Set the motor speed to 50%

	while(true) {
		// Main loop
		// uint32_t pulseWidth = receiver1.getPulseWidth(); // Get the pulse width from the receiver
		// Serial.print("Pulse Width: ");
		// Serial.println(pulseWidth); // Print the pulse width to the serial monitor

		myImu.updateAttitude(); // Update the IMU data
		float roll = myImu.getAttitude().rpy.roll; // Get the roll value from the IMU
		float pitch = myImu.getAttitude().rpy.pitch; // Get the pitch value from the IMU
		float yaw = myImu.getAttitude().rpy.yaw; // Get the yaw value from the IMU

		Serial.print("Roll: ");
		Serial.print(roll, 3);
		Serial.print(" Pitch: ");
		Serial.print(pitch, 3);
		Serial.print(" Yaw: ");
		Serial.println(yaw, 3); // Print the roll, pitch, and yaw values to the serial monitor
		delay(1);
	}
}
