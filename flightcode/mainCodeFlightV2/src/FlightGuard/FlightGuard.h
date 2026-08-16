#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "../Utilities/Utilities.h"
#include "FlightGuard_utilities.h"
#include "../Lander/Lander_structs.h"
class FlightManager; // forward declaration of FlightManager class

class FlightGuard {

    public:
    FlightGuard() = default;

    void begin(FlightManager* manager);
    void update();

    
    FlightGuardOverrideFlags overrideFlags; // flags to indicate if EDF and TVC are enabled or disabled, and if there are any flight condition violations      
    
    String motorEnabledStatus() const;
    String motorSafetyStatus() const;
     private:

        FlightManager* flight_manager = nullptr;
       

};