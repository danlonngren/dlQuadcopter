#include <Arduino.h>

#include "utils.h"
#include "imu.h"

#include "board_registers.h"

#include "receiver.h"
#include "motor.h"

#include "quadcopter.h"	


Imu myImu;
Receiver receiver(3);
MotorMixer motorMixer({Motor(1), Motor(2), Motor(3), Motor(4)});
Quadcopter quadcopter(myImu, receiver, motorMixer);


void setup() {
	// Set up the serial communication
	Serial.begin(115200);
	Serial.println("Serial port connected");

	quadcopter.init(); // Initialize the quadcopter
}

void loop() {
	Serial.println("Entering loop");

	quadcopter.startMainLoop();
}
