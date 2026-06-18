#include "LanderServo.h"

// Linear interpolation helper (no clamping).
static inline float lerp(float x, float inLo, float inHi, float outLo, float outHi) {
    return outLo + (x - inLo) * (outHi - outLo) / (inHi - inLo);
}

bool LanderServo::attach(uint8_t pin,
                         float   frequencyHz,
                         uint8_t resolutionBits,
                         float   minPulseUs,
                         float   maxPulseUs,
                         float   minAngleDeg,
                         float   maxAngleDeg)
{
    if (pin == 255 || frequencyHz <= 0.0f || resolutionBits == 0 || resolutionBits > 16) {
        return false;
    }

    _pin         = pin;
    _freqHz      = frequencyHz;
    _resBits     = resolutionBits;
    _maxDuty     = (1UL << resolutionBits) - 1UL;
    _periodUs    = 1e6f / frequencyHz;

    _minPulseUs  = minPulseUs;
    _maxPulseUs  = maxPulseUs;
    _minAngleDeg = minAngleDeg;
    _maxAngleDeg = maxAngleDeg;

    // Software limits default to the full physical range until overridden.
    _softMinDeg  = minAngleDeg;
    _softMaxDeg  = maxAngleDeg;

    pinMode(_pin, OUTPUT);
    analogWriteResolution(resolutionBits);   // NOTE: global on Teensy
    analogWriteFrequency(_pin, frequencyHz);  // per-pin

    _attached = true;
    return true;
}

void LanderServo::detach()
{
    if (_attached) {
        analogWrite(_pin, 0);
    }
    _attached = false;
    _lastDuty = 0;
}

void LanderServo::setAngleLimits(float minDeg, float maxDeg)
{
    if (minDeg > maxDeg) {                    // tolerate swapped args
        float t = minDeg; minDeg = maxDeg; maxDeg = t;
    }
    // Never allow software limits to exceed the physical envelope.
    _softMinDeg = (minDeg < _minAngleDeg) ? _minAngleDeg : minDeg;
    _softMaxDeg = (maxDeg > _maxAngleDeg) ? _maxAngleDeg : maxDeg;
}

void LanderServo::setPulseCalibration(float minPulseUs, float maxPulseUs)
{
    _minPulseUs = minPulseUs;
    _maxPulseUs = maxPulseUs;
}

uint32_t LanderServo::pulseToDuty(float pulseUs) const
{
    if (pulseUs < _minPulseUs) pulseUs = _minPulseUs;
    if (pulseUs > _maxPulseUs) pulseUs = _maxPulseUs;

    float duty = (pulseUs / _periodUs) * (float)_maxDuty;
    if (duty < 0.0f)             duty = 0.0f;
    if (duty > (float)_maxDuty)  duty = (float)_maxDuty;
    return (uint32_t)(duty + 0.5f);
}

uint32_t LanderServo::angleToDuty(float angleDeg) const
{
    if (angleDeg < _minAngleDeg) angleDeg = _minAngleDeg;
    if (angleDeg > _maxAngleDeg) angleDeg = _maxAngleDeg;

    float pulseUs = lerp(angleDeg,
                         _minAngleDeg, _maxAngleDeg,
                         _minPulseUs,  _maxPulseUs);
    return pulseToDuty(pulseUs);
}

void LanderServo::writeAngle(float angleDeg)
{
    if (!_attached) return;

    // 1. apply mechanical trim
    float cmd = angleDeg + _trimDeg;

    // 2. software (operational) limits, e.g. tvcMaxAngle
    if (cmd < _softMinDeg) cmd = _softMinDeg;
    if (cmd > _softMaxDeg) cmd = _softMaxDeg;

    // 3. physical envelope (angleToDuty clamps again, kept here for state)
    if (cmd < _minAngleDeg) cmd = _minAngleDeg;
    if (cmd > _maxAngleDeg) cmd = _maxAngleDeg;

    _lastAngleDeg = cmd;
    _lastPulseUs  = lerp(cmd, _minAngleDeg, _maxAngleDeg, _minPulseUs, _maxPulseUs);
    _lastDuty     = pulseToDuty(_lastPulseUs);

    analogWrite(_pin, _lastDuty);
}

void LanderServo::writeMicroseconds(float pulseUs)
{
    if (!_attached) return;

    if (pulseUs < _minPulseUs) pulseUs = _minPulseUs;
    if (pulseUs > _maxPulseUs) pulseUs = _maxPulseUs;

    _lastPulseUs  = pulseUs;
    _lastDuty     = pulseToDuty(pulseUs);
    _lastAngleDeg = lerp(pulseUs, _minPulseUs, _maxPulseUs, _minAngleDeg, _maxAngleDeg);

    analogWrite(_pin, _lastDuty);
}

void LanderServo::writeRaw(uint32_t duty)
{
    if (!_attached) return;
    if (duty > _maxDuty) duty = _maxDuty;
    _lastDuty = duty;
    analogWrite(_pin, duty);
}
