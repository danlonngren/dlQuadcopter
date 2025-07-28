#ifndef RECEIVER_H
#define RECEIVER_H

#include <stdint.h>
#include <array>
#include <Arduino.h>

using namespace std;

class Receiver {

    public:
        Receiver(uint8_t pin, uint32_t maxPulseWidth = 2000);
        ~Receiver();

        void start();

        uint32_t getPulseWidth() const;

    private:
        static void receiverISR();
        void handlePWM();

    private:
        uint8_t m_pin;
        uint8_t m_channel;
        uint32_t m_maxPulseWidth;

        volatile uint32_t m_startTime;
        volatile uint32_t m_pulseWidth;

        // Static pointer to instance for the ISR
        static Receiver *m_instance;
};

#endif