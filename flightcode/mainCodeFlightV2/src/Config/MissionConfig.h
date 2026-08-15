
#pragma once
#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H


#include "../Utilities/Utilities.h"
#include "Arduino.h"


enum class WaypointState{
    INACTIVE,
    APPROACHING,
    HOLDING,
    COMPLETED
};
enum class WaypointType{
    TAKEOFF,
    NAVIGATION,
    PRE_LANDING,
    LANDING
};




struct MissionTarget {
    WaypointTarget target; // position in NED and YAW
    WaypointType type; // take off, landing, navig..
};

 namespace MissionConfig {

    constexpr ParameterConfig<uint32_t> holdTimeMs = {
        .min = 0,
        .max = 20 * 1000, // in ms
        .default_ = 0 // default is 0 means we move on automaticallyS
    };


    constexpr float takeOff_altitude_m = 0.5; // altitude above ground for take off
    constexpr float takeOff_yaw_deg = 0.0; // yaw angle for take off
    constexpr float landing_yaw_deg = 0.0; // yaw angle for landing
    constexpr float landing_descentStart_altitude_m = 0.5; // altitude above ground for landing descent start





    constexpr MissionTarget landingTarget_relative_NED = {
        .target = {
            .positionNED = {0.67f, 0.420f, 0.0f}, // landing at the take off point
            .yaw_deg = 0.0f
        },
        .type = WaypointType::LANDING
    };
    


}

#endif
