#pragma once

#ifndef ACTUATOR_PWM_H
#define ACTUATOR_PWM_H

#include <Arduino.h>
#include <Servo.h>

#include "../Utilities/Utilities.h"
#include "../Config/ActuatorsConfig.h"

class ActuatorPWM {
    public:
        ActuatorPWM();

        void begin(
            const uint8_t pin,
            const float frequency,
            const uint16_t pulseMin,
            const uint16_t pulseMax
        );

        void setTrim(float trim_deg);
        void setValue(float value);

        float getValue() const;
        float getTrim() const;

        uint8_t getPin() const;
        float getFrequency() const;

        uint16_t getPulseMin() const;
        uint16_t getPulseMax() const;

        ActuatorPWM_config getConfig() const;

        void write(float value);
        void attach(ActuatorPWM_config config);

    private:
        Servo servo;

        uint8_t pin;
        float frequency;

        uint16_t pulseMin;
        uint16_t pulseMax;

        float valueMin; // associated to pulseMin
        float valueMax; // associated to pulseMax

        float trim_deg;

        float value;
};

#endif