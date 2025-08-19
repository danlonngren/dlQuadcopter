#include "receiver.h"

#include <Arduino.h>

constexpr uint32_t NUM_CHANNELS = 2100; // Minimum pulse width in microseconds
constexpr uint32_t SYNC_PULSE_MIN = 2100; // Maximum pulse width in microseconds

// Define the static member variable
Receiver* Receiver::m_instance = nullptr;

Receiver::Receiver(uint8_t pin, bool isCPPM, uint32_t maxPulseWidth) :
    m_isCPPM(isCPPM), m_pin(pin), m_maxPulseWidth(maxPulseWidth),
    m_lastTime(0), m_pulseWidth(0), m_channelCount(0) {

    // Set the static instance to this object for interrupt handling
    m_instance = this;

    for (int i = 0; i < 8; i++) {
        m_channels[i] = 0;
    }
}

Receiver::~Receiver() {
    detachInterrupt(digitalPinToInterrupt(m_pin));
    m_instance = nullptr;
}

void Receiver::start() {
    pinMode(m_pin, INPUT);
    if (m_isCPPM)
        attachInterrupt(digitalPinToInterrupt(m_pin), Receiver::receiverISR, RISING);
    else
        attachInterrupt(digitalPinToInterrupt(m_pin), Receiver::receiverISR, CHANGE);
}

uint32_t Receiver::getPulseWidth() const {
    if (m_isCPPM) 
        return 0;

    noInterrupts();
    uint32_t pulseWidth = m_pulseWidth;
    interrupts();
    return pulseWidth;
}

uint32_t Receiver::getCPPMChannel(uint8_t channel) const {
    if (channel >= 8 || m_isCPPM == false) 
        return 0;

    noInterrupts();
    uint32_t pulseWidth = m_channels[channel];
    interrupts();
    
    return pulseWidth;
}

void Receiver::handlePWM() {
    uint32_t now = micros();
    uint32_t pinState = digitalRead(m_pin);

    if (pinState == HIGH) {
        m_lastTime = now;
    } else {
        uint32_t pw = now - m_lastTime;
        if (pw <= m_maxPulseWidth)
            m_pulseWidth = pw;
    }
}

void Receiver::handleCPPM() {
    uint64_t currentMicros = micros();
    uint64_t pulseWidth = currentMicros - m_lastTime;
    m_lastTime = currentMicros;

    if (pulseWidth > SYNC_PULSE_MIN) {
        m_channelCount = 0;
        
        for (int i = 0; i < 8; i++) {
            m_channelsSafe[i] = m_channels[i];
        }
    }
    else if (m_channelCount < 8) {
        m_channels[m_channelCount++] = pulseWidth;
    }
}

void Receiver::receiverISR() {
    if (!m_instance) 
        return;

    if (m_instance->m_isCPPM)
        m_instance->handleCPPM();
    else
        m_instance->handlePWM();
}