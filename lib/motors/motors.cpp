#include "motors.h"

#include <Arduino.h>

#define SETA GPIOA_PSOR
#define CLRA GPIOA_PCOR
#define SETB GPIOB_PSOR
#define CLRB GPIOB_PCOR
#define SETC GPIOC_PSOR
#define CLRC GPIOC_PCOR
#define SETD GPIOD_PSOR
#define CLRD GPIOD_PCOR


motors::motors()
{
    // Setup the motor pins
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
}

void motors::setMotorSpeed(int motor, int speed)
{
  //150 ms pusle to keep motors from beeping
  for (int cal_int = 0; cal_int < 1000; cal_int++) {
    SETD = (1 << 4); //6
    SETD = (1 << 2); //7
    SETD = (1 << 3); //8
    SETC = (1 << 3); //9
    delayMicroseconds(125);
    CLRD = (1 << 4); //6
    CLRD = (1 << 2); //7
    CLRD = (1 << 3); //8
    CLRC = (1 << 3); //9
    delayMicroseconds(3000);
  }
}

