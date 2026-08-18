#pragma once

#include <Arduino.h>
#include <Wire.h>


struct FlightGuardOverrideFlags {

    bool EDF_enabled = true; // if false, the motor is disabled and will not spin
    bool EDF_armed = false; // if false, the motor is not armed and will not spin


    bool TVC_enabled = true; // if false, the TVC is disabled and will not move

    bool vertical_overspeed_ok = false; // if true, indicates that the lander is overspeeding

    bool horizontal_overspeed_ok = false; // if true, indicates that the lander is overspeeding

    bool altitude_limit_ok = false; // if true, indicates that the lander is above the altitude limit

    bool NED_horizontal_boundary_ok= true; // if true, indicates that the lander is outside the NED boundary

    bool NED_vertical_boundary_ok = true; // if true, indicates that the lander is outside the NED boundary


    bool roll_pitch_ok = true; // if true, indicates that the lander is outside the roll/pitch bounds
    bool yaw_ok = true; // if true, indicates that the lander is outside the yaw bounds
    
};



namespace FlightGuardUtilities {

    
    bool isRollPitchWithinBounds(float pitch_deg, float roll_deg);
    bool isYawWithinBounds(float yaw_deg);
    bool isVerticalVelocityWithinBounds(float speed);
    bool isHorizontalVelocityWithinBounds(float velocity_N, float velocity_E);


    void resetOverrideFlags(FlightGuardOverrideFlags& flags);
}