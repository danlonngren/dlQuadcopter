#include <Arduino.h>

#include "utils.h"
#include "imu.h"

#include "board_registers.h"

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  utils util; // Create an instance of the utils class
  imu myImu; // Create an instance of the imu class
  myImu.calibrate(2000); // Call the Calibrate method to calibrate the MPU6050

  int result = myFunction(2, 3);

  Serial.begin(9600);
  Serial.println(result); // Print the result to the Serial Monitor
  Serial.println(util.Pyth(1, 2, 3)); // Example usage of the Pyth function

}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}