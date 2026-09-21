#include "ActuatorPWM.h"


ActuatorPWM::ActuatorPWM()
    : pin(0),
      frequency(50.0f),
      pulseMin(1000),
      pulseMax(2000),
      valueMin(0.0f),
      valueMax(180.0f),
      trim_deg(0.0f),
      value(0.0f)
{
}


void ActuatorPWM::begin(
    const uint8_t pin,
    const float frequency,
    const uint16_t pulseMin,
    const uint16_t pulseMax
)
{
    this->pin = pin;
    this->frequency = frequency;
    this->pulseMin = pulseMin;
    this->pulseMax = pulseMax;

    servo.attach(
        this->pin,
        this->pulseMin,
        this->pulseMax
    );
}


void ActuatorPWM::attach(ActuatorPWM_config config)
{
    pin = config.pin;
    frequency = config.frequency;

    pulseMin = config.pulseMin;
    pulseMax = config.pulseMax;

    valueMin = config.valueMin;
    valueMax = config.valueMax;

    trim_deg = config.trim_deg;

    servo.attach(
        pin,
        pulseMin,
        pulseMax
    );
}


void ActuatorPWM::setTrim(float trim_deg)
{
    this->trim_deg = trim_deg;
}


void ActuatorPWM::setValue(float value)
{
    write(value);
}


float ActuatorPWM::getValue() const
{
    return value;
}


float ActuatorPWM::getTrim() const
{
    return trim_deg;
}


uint8_t ActuatorPWM::getPin() const
{
    return pin;
}


float ActuatorPWM::getFrequency() const
{
    return frequency;
}


uint16_t ActuatorPWM::getPulseMin() const
{
    return pulseMin;
}


uint16_t ActuatorPWM::getPulseMax() const
{
    return pulseMax;
}


ActuatorPWM_config ActuatorPWM::getConfig() const
{
    ActuatorPWM_config config;

    config.pin = pin;
    config.frequency = frequency;

    config.pulseMin = pulseMin;
    config.pulseMax = pulseMax;

    config.valueMin = valueMin;
    config.valueMax = valueMax;

    config.trim_deg = trim_deg;

    return config;
}


void ActuatorPWM::write(float value)
{
    float commandedValue = value + trim_deg;

    commandedValue = constrain(
        commandedValue,
        valueMin,
        valueMax
    );

    float pulseWidth =
        pulseMin +
        (commandedValue - valueMin) *
        (pulseMax - pulseMin) /
        (valueMax - valueMin);

    pulseWidth = constrain(
        pulseWidth,
        static_cast<float>(pulseMin),
        static_cast<float>(pulseMax)
    );

    servo.writeMicroseconds(static_cast<uint16_t>(pulseWidth));

    this->value = value;
}