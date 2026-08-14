#pragma once
#include <Arduino.h>

enum class FlightState : uint8_t {
    Boot,
    Standby,
    Armed,
    Takeoff,
    Flight,
    Landing,
    Landed,
    Abort
};

// ============================================================
// Flight manager status
// ============================================================

