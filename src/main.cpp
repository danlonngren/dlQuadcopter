#include <Arduino.h>

#include "utils.h"
#include "imu.h"

#include "board_registers.h"

#include "receiver.h"
#include "motor.h"

#include "controller.h"

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

void loop() {
	Serial.println("Entering loop");
	
	Imu myImu;			   // Create an instance of the imu class
	Receiver receiver(3); // Create an instance of the Receiver class
	std::array<Motor, 4> motors = { Motor(1), Motor(2), Motor(3), Motor(4) };
	Controller controller(myImu, receiver, motors);

	while(true) {
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
