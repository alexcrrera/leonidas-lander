#pragma once

#include "StateMachine_utilities.h"
#include "Arduino.h"
#include "../Config/FlightRegimeConfig.h"

class FlightManager;

class StateMachine {
    public:
        StateMachine() = default;
        void begin(FlightManager* flight_manager_);
        void update();
        
        void setState(STATE_MACHINE_STATES new_state);
        STATE_MACHINE_STATES getState() const;

        String getStateAsString() const;
        String getStateAsString(STATE_MACHINE_STATES state) const;

        bool isInFlight(STATE_MACHINE_STATES state) const;
        FlightRegimeData getCurrentFlightRegimeData() const;

        void requestStateChange(STATE_MACHINE_STATES new_state); // requests a state change, but the actual change will be handled by the flight manager based on conditions




    private:
        STATE_MACHINE_STATES current_state;
        FlightManager* flight_manager = nullptr;


        bool verifyChange(STATE_MACHINE_STATES new_state); // verifies if a state change is valid based on the current state and conditions
        bool handleTakeOff_request();
        bool handleLanding_request();
        bool handleAbort_request();
        

};