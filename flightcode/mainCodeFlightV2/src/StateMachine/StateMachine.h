#pragma once

#include "StateMachine_utilities.h"
#include "Arduino.h"

class FlightManager;

class StateMachine {
    public:
        StateMachine() = default;
        void begin(FlightManager* flight_manager_);
        void update();
        
        void setState(STATE_MACHINE_STATES new_state);
        STATE_MACHINE_STATES getState() const;

        String getStateAsString() const;

        bool isInFlight(STATE_MACHINE_STATES state) const;

    private:
        STATE_MACHINE_STATES current_state;
        FlightManager* flight_manager = nullptr;
};