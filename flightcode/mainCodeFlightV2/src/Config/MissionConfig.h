
#pragma once
#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H


#include "../Utilities/Utilities.h"
#include "Arduino.h"



struct MissionTarget{
    WaypointTarget target;
    // constant string
  
};





 namespace MissionConfig {

    constexpr ParameterConfig<uint32_t> holdTimeMs = {
        .min = 0,
        .max = 20 * 1000, // in ms
        .default_ = 0 // default is 0 means we move on automaticallyS
    };


    constexpr float takeOff_altitude_m = 0.15; // altitude above ground for take off
    constexpr float takeOff_yaw_deg = 0.0; // yaw angle for take off
    constexpr float landing_yaw_deg = 0.0; // yaw angle for landing
    constexpr float landing_descentStart_altitude_m = 0.25; // altitude above ground for landing descent start





    constexpr MissionTarget landingTarget_relative_NED = {
        .target = {
            .positionNED = {0.0f, 0.0f, 0.0f}, // landing at the take off point
            .yaw_deg = 0.0f
        },
       
    };

    constexpr MissionTarget NAV_WAYPOINT_1 = {
        .target = {
            .positionNED = {0.5f, 0.0f, 1.0f}, // 0.5 m north of the take off point
            .yaw_deg = 0.0f
        },
       
    };
    


}

#endif
