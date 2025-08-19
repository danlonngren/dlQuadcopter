#pragma once

#include <stdint.h>
#include <array>

enum PwmResolution {
    PWM_8BIT = 8,   // 0-255
    PWM_10BIT = 10, // 0-1023
    PWM_12BIT = 12, // 0-4095
    PWM_16BIT = 16  // 0-65535
};


class Motor {
    public:
        Motor(  uint8_t motorPin, 
                float pwmFreq = 1000.0,
                PwmResolution pwmRes = PwmResolution::PWM_8BIT);
                
        ~Motor();

        void init();

        void setMotorSpeed(int percentPower);

    private:
        uint8_t m_motorPin;
        float m_pwmFreq;
        int m_maxResValue;
        PwmResolution m_pwmRes;
};


// Basic X-quad motor mixing
// M1 = Front Left (CW)
// M2 = Front Right (CCW)
// M3 = Rear Right (CW)
// M4 = Rear Left (CCW)
class MotorMixer {
public:
    MotorMixer(std::array<Motor, 4> motors);

    void init() {
        for (auto& motor : m_motors) {
            motor.init();
        }
    };

    void setOutputs(float throttle, float roll, float pitch, float yaw);

private:
    std::array<Motor, 4> m_motors;

    // Optional: normalize to 0-100% safe range
    void applyMotorLimits(std::array<float, 4>& outputs);
};