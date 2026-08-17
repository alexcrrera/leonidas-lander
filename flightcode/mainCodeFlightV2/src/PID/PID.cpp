#include "PID.h"

PID::PID(PID_Gains gains, float frequency)
    : kp(gains.kp), ki(gains.ki), kd(gains.kd), frequency(frequency)
{
    handler.begin(frequency, this, &PID::compute);
}

float PID::update(float measurement_, float measurementDerivative_)
{
    derivativePassed = true;
    measurement = measurement_;
    measurementDerivative = measurementDerivative_;

    handler.handle();

    return output;
}

float PID::update(float measurement_)
{
    derivativePassed = false;
    measurement = measurement_;

    handler.handle();

    return output;
}

void PID::compute(float dt)
{
    float error = desiredTarget - measurement;

    integral = updateIntegral(dt, error);

    float errorDerivative;

    if (derivativePassed)
    {
        errorDerivative = -measurementDerivative;
    }
    else
    {
        errorDerivative = (error - previousError) / dt;
    }

    previousError = error;

    output = kp * error + integral + kd * errorDerivative;

    output = constrain(output, outputMin, outputMax);
}

void PID::setOutputLimits(float uMin, float uMax)
{
    outputMin = uMin;
    outputMax = uMax;
}

void PID::setIntegralLimits(float iMin, float iMax)
{
    integralMin = iMin;
    integralMax = iMax;
}

void PID::setGains(PID_Gains gains)
{
    kp = gains.kp;
    ki = gains.ki;
    kd = gains.kd;
}

void PID::setGains(float kp_, float ki_, float kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void PID::reset()
{
    integral = 0.0f;
    previousError = 0.0f;
    output = 0.0f;
}

void PID::setOffset(float offset_)
{
    offset = offset_;
}

void PID::setTarget(float target)
{
    desiredTarget = target;
}

float PID::updateIntegral(float dt, float error)
{
    integral += ki * error * dt;
    integral = constrain(integral, integralMin, integralMax);

    return integral;
}