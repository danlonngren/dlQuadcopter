#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

class Motor {

    public:
        enum PwmResolution {
            PWM_8BIT = 8,   // 0-255
            PWM_10BIT = 10, // 0-1023
            PWM_12BIT = 12, // 0-4095
            PWM_16BIT = 16  // 0-65535
        };

    public:
        Motor(  uint8_t motorPin, 
                float pwmFreq = 1000.0,
                PwmResolution pwmRes = PwmResolution::PWM_8BIT);
                
        ~Motor();

        void setMotorSpeed(int percentPower);

    private:
        uint8_t m_motorPin;
        float m_pwmFreq;
        int m_maxResValue;
};

#endif