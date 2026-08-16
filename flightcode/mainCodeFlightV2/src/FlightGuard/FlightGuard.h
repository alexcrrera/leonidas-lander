#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "../Utilities/Utilities.h"
#include "FlightGuard_utilities.h"
#include "../Lander/Lander_structs.h"
class FlightManager; // forward declaration of FlightManager class

class FlightGuard {
    FlightGuard() = default;

    void begin(FlightManager* manager);
    void update();


    private:

        FlightManager* flight_manager = nullptr;

};