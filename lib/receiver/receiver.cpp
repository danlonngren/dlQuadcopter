#include "receiver.h"

#include <Arduino.h>

using namespace std;

Receiver::Receiver(uint8_t pin, uint8_t channel, uint32_t maxPulseWidth) :
    m_pin(pin),
    m_channel(channel),
    m_pulseWidth(0),
    m_startTime(0),
    m_maxPulseWidth(maxPulseWidth) {

    // Set the static instance to this object
    m_instance = this;
}

Receiver::~Receiver() {
    // Destructor
    // No need to do anything here
}

void Receiver::start() {
    // Start the receiver
    uint32_t pinInt = digitalPinToInterrupt(m_pin);
    if (pinInt == NOT_AN_INTERRUPT) {
        Serial.println("Error: Pin " + String(m_pin) + " is not a valid interrupt pin.");
    }
    else {
        pinMode(m_pin, INPUT);
        attachInterrupt(pinInt, Receiver::receiverISR, CHANGE);
    }
}

uint32_t Receiver::getPulseWidth() const {
    noInterrupts();
    uint32_t pulseWidth = m_pulseWidth;
    interrupts();
    return pulseWidth;
}

void Receiver::handlePWM() {
    unsigned long currentTime = micros();
    int pinState = digitalReadFast(m_pin);
    
    if (pinState == HIGH) {
        m_startTime = currentTime;
    }
    else {
        m_pulseWidth = currentTime - m_startTime;
        // Limit the pulse width to a maximum of 2000 microseconds
        if (m_pulseWidth > m_maxPulseWidth)
            m_pulseWidth = m_maxPulseWidth;
    }
}

void Receiver::receiverISR()
{
    // Call the handlePWM function to process the PWM signal
    if (m_instance) 
        m_instance->handlePWM();
}