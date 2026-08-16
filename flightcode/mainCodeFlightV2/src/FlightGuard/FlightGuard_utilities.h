#pragma once

#include <Arduino.h>
#include <Wire.h>


struct FlightGuardOverrideFlags {

    bool EDF_enabled = true; // if false, the motor is disabled and will not spin
    bool EDF_armed = false; // if false, the motor is not armed and will not spin


    bool TVC_enabled = true; // if false, the TVC is disabled and will not move

    bool vertical_overspeed_indicator = false; // if true, indicates that the lander is overspeeding

    bool horizontal_overspeed_indicator = false; // if true, indicates that the lander is overspeeding

    bool altitude_limit_indicator = false; // if true, indicates that the lander is above the altitude limit

    bool NED_horizontal_boundary_indicator = false; // if true, indicates that the lander is outside the NED boundary


    
};