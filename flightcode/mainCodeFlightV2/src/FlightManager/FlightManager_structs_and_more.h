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

struct FlightStatus {
    FlightState state = FlightState::Boot;
    FlightState previousState = FlightState::Boot;

    bool armed = false;
    bool abortRequested = false;

    uint32_t stateEntryTime = 0;
    uint32_t flightStartTime = 0;
    uint32_t timestamp = 0;
};