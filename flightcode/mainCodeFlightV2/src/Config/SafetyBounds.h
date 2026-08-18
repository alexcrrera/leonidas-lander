
#pragma once
#ifndef SAFETY_BOUNDS_H
#define SAFETY_BOUNDS_H
#include "../Utilities/Utilities.h"

/*
SAFETY PARAMETERS THAT CAN TOGGLE ABORT CONDITIONS

*/


 namespace SafetyBounds {

    constexpr float maxAltitude_m = 3.0; // max altitude the vehicle should be allowed 
    constexpr float minHoverAltitude_m = 0.1; // minimum flight altitude
    constexpr float maxRadius_m = 2.0;

  

    constexpr float max_horizontal_velocity_ms = 1.0;
    constexpr float max_vertical_velocity_ms = 1.0;

    constexpr float maxYaw_deg = 45.0;
    constexpr float minYaw_deg = -45.0;


    // rates

    constexpr float max_vertical_acceleration_ms2 = 7.0;
    constexpr float max_horizontal_acceleration_ms2 = 7.0;


    constexpr float max_yaw_velocity_degs = 20;
    constexpr float  max_pitch_roll_velocity_degs = 15.0; 


    constexpr float max_yaw_acceleration_degs2 = 40.0;
    constexpr float max_pitch_roll_acceleration_degs2 = 20.0;


    constexpr float  yaw_abort_angle_deg = 99999.0; // yaw angle that will trigger an abort
    constexpr float pitch_roll_abort_angle_deg = 20.0;
    



 }


#endif