#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "../Utilities/Utilities.h"
#include "FlightGuard_utilities.h"
#include "../Lander/Lander_structs.h"
#include "../Utilities/Utilities.h"
#include "../StateMachine/StateMachine.h"
#include "../FlightGuard/FlightGuard_utilities.h"

class FlightManager; // forward declaration of FlightManager class

class FlightGuard {

    public:
    FlightGuard() = default;

    void begin(FlightManager* manager);
    void update();

    
    FlightGuardOverrideFlags overrideFlags; // flags to indicate if EDF and TVC are enabled or disabled, and if there are any flight condition violations      
    
    String motorEnabledStatus() const;
    String motorSafetyStatus() const;

    bool isFlightConditionValid();


    String getFlightGuardStatus() const;
    
     private:

        //bool isFlightConditionValid();
        void checkFlightConditions();

       
       

        FlightManager* flight_manager = nullptr;
       

};