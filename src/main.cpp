#include <Arduino.h>

#include "quadcopter.h"	

Quadcopter quadcopter;

void setup() {
	// Set up the serial communication
	Serial.begin(115200);
	while (!Serial); // Wait for serial port to connect (needed on some boards)

	Serial.println("Serial port connected");

	quadcopter.init(); // Initialize the quadcopter
}

void loop() {
	Serial.println("Entering loop");

	delay(1000); // Delay to allow serial output to stabilize
	quadcopter.startMainLoop();
}
