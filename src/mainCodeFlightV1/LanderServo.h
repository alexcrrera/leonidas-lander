#ifndef LANDER_SERVO_H
#define LANDER_SERVO_H

#include <Arduino.h>

// ============================================================================
//  LanderServo
//  ----------------------------------------------------------------------------
//  Low-level PWM servo driver for the LEONIDAS TVC vanes (Teensy 4.x).
//
//  Wraps the Teensy core analogWrite path (hardware PWM, non-blocking, no
//  jitter from software timing) but adds what <Servo.h> cannot do:
//    - arbitrary PWM frequency (e.g. 333 Hz) and bit resolution per servo
//    - command in angle (deg) OR raw microseconds
//    - per-servo mechanical trim offset      (replaces MissSX1, MissSX2, ...)
//    - per-servo software angle limits        (replaces tvcMaxAngle clamping)
//
//  Angle -> pulse is a straight linear map of the calibrated endpoints:
//      minAngleDeg -> minPulseUs
//      maxAngleDeg -> maxPulseUs
//  (For a symmetric -50..+50 deg / 1000..2000 us servo, 0 deg = 1500 us.)
//
//  NOTE on Teensy: analogWriteResolution() is GLOBAL, not per-pin. attach()
//  sets it, so every attached LanderServo should use the same resolutionBits.
//  analogWriteFrequency() IS per-pin (per FlexPWM submodule), so mixed
//  frequencies are fine as long as pins are on different timer groups.
// ============================================================================

class LanderServo {
public:
    LanderServo() {}

    // Configure and start driving the pin. Returns false on a bad pin.
    // Defaults match the CM509MG vanes: 333 Hz, 12-bit, 1000-2000 us, +/-50 deg.
    bool attach(uint8_t pin,
                float   frequencyHz    = 333.0f,
                uint8_t resolutionBits = 12,
                float   minPulseUs     = 1000.0f,
                float   maxPulseUs     = 2000.0f,
                float   minAngleDeg    = -50.0f,
                float   maxAngleDeg    =  50.0f);

    void detach();                       // stops PWM (writes 0 duty)
    bool attached() const { return _attached; }

    // ---- Commands ----------------------------------------------------------
    // writeAngle: adds trim, clamps to software limits then physical range,
    //             maps to pulse width, and outputs. This is the normal path.
    void writeAngle(float angleDeg);

    // writeMicroseconds: bypasses angle math; clamps to [minPulseUs,maxPulseUs].
    void writeMicroseconds(float pulseUs);

    // writeRaw: lowest level; clamps to [0, maxDuty] and outputs directly.
    void writeRaw(uint32_t duty);

    // ---- Per-servo configuration ------------------------------------------
    void setTrim(float trimDeg)                 { _trimDeg = trimDeg; }
    void setAngleLimits(float minDeg, float maxDeg);   // software clamp
    void setPulseCalibration(float minPulseUs, float maxPulseUs);

    // ---- Introspection -----------------------------------------------------
    uint8_t  pin()           const { return _pin; }
    float    trim()          const { return _trimDeg; }
    float    lastAngle()     const { return _lastAngleDeg; }   // post-trim, clamped
    float    lastMicros()    const { return _lastPulseUs; }
    uint32_t lastRaw()       const { return _lastDuty; }
    uint32_t maxDuty()       const { return _maxDuty; }

    // Pure conversion helpers (no output), handy for tests / logging.
    uint32_t angleToDuty(float angleDeg) const;
    uint32_t pulseToDuty(float pulseUs)  const;

private:
    uint8_t  _pin       = 255;
    bool     _attached  = false;

    float    _freqHz     = 333.0f;
    uint8_t  _resBits    = 12;
    uint32_t _maxDuty    = 4095;     // (1 << resBits) - 1
    float    _periodUs   = 3003.0f;  // 1e6 / freqHz

    float    _minPulseUs = 1000.0f;
    float    _maxPulseUs = 2000.0f;
    float    _minAngleDeg = -50.0f;
    float    _maxAngleDeg =  50.0f;

    float    _trimDeg     = 0.0f;
    float    _softMinDeg  = -50.0f;  // software limit, defaults to physical
    float    _softMaxDeg  =  50.0f;

    float    _lastAngleDeg = 0.0f;
    float    _lastPulseUs  = 0.0f;
    uint32_t _lastDuty     = 0;
};

#endif // LANDER_SERVO_H
