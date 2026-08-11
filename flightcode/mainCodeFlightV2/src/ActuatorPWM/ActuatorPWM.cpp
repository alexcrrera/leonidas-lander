

#include "ActuatorPWM.h"


ActuatorPWM::ActuatorPWM() : pin(0), frequency(0.0f), pulseMin(0), pulseMax(0), valueMin(0.0f), valueMax(0.0f), trim(0.0f) {
    // Constructor implementation
}

ActuatorPWM_config ActuatorPWM::getConfig() const {
    return ActuatorPWM_config{
        .pin = pin,
        .frequency = frequency,
        .pulseMin = pulseMin,
        .pulseMax = pulseMax,
        .valueMin = valueMin,
        .valueMax = valueMax,
        .trim = trim
    };
}

void ActuatorPWM::attach(ActuatorPWM_config config) {
    pin = config.pin;
    frequency = config.frequency;
    pulseMin = config.pulseMin;
    pulseMax = config.pulseMax;
    valueMin = config.valueMin;
    valueMax = config.valueMax;
    trim = config.trim;

    // Additional hardware attachment logic can be added here
}   

void ActuatorPWM::write(float value) {
    // Map the input value to the corresponding pulse width
    float mappedValue = map(value, valueMin, valueMax, pulseMin, pulseMax);
    // Apply trim
    mappedValue += trim;

    // Ensure the mapped value is within the pulse range
    mappedValue = constrain(mappedValue, pulseMin, pulseMax);

    // Write the PWM signal to the pin
    analogWrite(pin, static_cast<int>(mappedValue));
}