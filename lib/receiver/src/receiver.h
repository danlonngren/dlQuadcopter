#ifndef RECEIVER_H
#define RECEIVER_H

#include <stdint.h>


class Receiver {
    public:
        /**
         * @brief Constructor for the Receiver class.
         * @param pin The pin number to which the receiver is connected.
         * @param isCPPM Indicates if the receiver is in CPPM mode.
         * @param maxPulseWidth The maximum pulse width in microseconds (default: 2000).
         */
        Receiver(uint8_t pin, bool isCPPM = false, uint32_t maxPulseWidth = 2000);

        /**
         * @brief Destructor for the Receiver class.
         */
        ~Receiver();
        
        /**
         * @brief Attaches the interrupt and starts monitoring for PWM signals.
         */
        void start();

        /**
         * @brief Returns the pulse width of the last received signal.
         * @return The pulse width in microseconds.
         */
        uint32_t getCPPMChannel(uint8_t channel) const;

        /**
         * @brief Returns the pulse width of the last received signal.
         * @return The pulse width in microseconds.
         */
        uint32_t getPulseWidth() const;

    private:
        /**
         * @brief Interrupt Service Routine for the receiver.
         */
        static void receiverISR();

        /**
         * @brief Handles the pulse width measurement.
         */
        void handlePWM();

        /**
         * @brief Handles CPPM
         */
        void handleCPPM();

    private:
        bool m_isCPPM;
        uint8_t m_pin;
        uint8_t m_channel;
        uint32_t m_maxPulseWidth;

        volatile uint64_t m_lastTime;
        volatile uint32_t m_pulseWidth;

        // For CPPM
        volatile uint32_t m_channels[8];
        volatile uint32_t m_channelsSafe[8];
        volatile uint32_t m_channelCount;

        // Static pointer to instance for the ISR
        static Receiver *m_instance;
};

#endif