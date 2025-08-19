#include "motor.h"

// Include arduino PWM library
#include <Arduino.h>
// #include <wiring_analog.h>

Motor::Motor(uint8_t motorPin, 
            float pwmFreq,
            PwmResolution pwmRes) :
    m_motorPin(motorPin),
    m_pwmFreq(pwmFreq),
    m_maxResValue(0) {

    switch (pwmRes) {
        case PwmResolution::PWM_8BIT:
            m_maxResValue = 255; // 0-255
            break;
        case PwmResolution::PWM_10BIT:
            m_maxResValue = 1023; // 0-1023
            break;
        case PwmResolution::PWM_12BIT:
            m_maxResValue = 4095; // 0-4095
            break;
        case PwmResolution::PWM_16BIT:
            m_maxResValue = 65535; // 0-65535
            break;
        default:
            m_maxResValue = 0;
            break;
    }

    // Setup the motor pins
    pinMode(motorPin, OUTPUT);

    // analogWriteFrequency(m_motorPin, pwmFreq); // Not available on nRF52840
    analogWriteResolution((uint32_t)pwmRes);

    // Set initial speed to 0
    analogWrite(m_motorPin, 0);
}

Motor::~Motor() {
    analogWrite(m_motorPin, 0); // Stop the motor
    pinMode(m_motorPin, INPUT); // Set the motor pin to input mode
}

void Motor::setMotorSpeed(int percentPower) {
    // Set the motor speed
    if (m_motorPin == 0) {
        return; // Invalid motor pin
    }

    // Limit speed withing normal pwm range
    if (percentPower < 0) {
        percentPower = 0;
    }
    else if (percentPower > 100) {
        percentPower = 100;
    }

    // Calculate duty cycle
    int dutyCycle = (percentPower / 100) * m_maxResValue;
    
    // Set the new duty cycle
    analogWrite(m_motorPin, dutyCycle);

    Serial.print("Set dutyCycle: " + String(dutyCycle) + ", for pin: " + String(m_motorPin) + "\n");
}