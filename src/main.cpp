#include <Arduino.h>

#include "utils.h"
#include "imu.h"

#include "board_registers.h"

#include "receiver.h"
#include "motor.h"

#include "quadcopter.h"	

// #include "Adafruit_TinyUSB.h"

Imu myImu;			   // Create an instance of the imu class
Receiver receiver(3); // Create an instance of the Receiver class
std::array<Motor, 4> motors = { Motor(1), Motor(2), Motor(3), Motor(4) };
Quadcopter quadcopter(myImu, receiver, motors);


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

	quadcopter.startMainLoop();
}
