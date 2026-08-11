#pragma once
#ifndef WAYPOINT_H
#define WAYPOINT_H


#include <Arduino.h>
#include "../Utilities/Utilities.h"
#include "../Config/EpsConfig.h"
#include "../Config/MissionConfig.h"

enum class WaypointType{
    TAKEOFF,
    NAVIGATION,
    PRE_LANDING,
    LANDING
};


enum class WaypointState{
    INACTIVE,
    APPROACHING,
    HOLDING,
    COMPLETED
};

struct WaypointTarget {
    Vector3 positionNED;
    float yawDeg;
};


class Waypoint{


    public:


    Waypoint(); // default constructor as a placeholder for a waypoint that is not defined yet
    Waypoint(
        WaypointType type,
        const Vector3& positionNED,
        float yawDeg,
        uint32_t holdTimeMs = MissionConfig::holdTimeMs.default_,
        EpsilonGroup epsilon_group = {}
    );

    void update(
        const Vector3& currentPositionNED,
        float yawDeg
    );

    bool isReached();

    void activate();


    WaypointType getType() const; // getter
    WaypointState getState() const;

    const WaypointTarget& getTarget() const;

    bool isActive();


    private:
        bool constructed = false; // indicates if the waypoint has been constructed or not
        WaypointType type;
        WaypointState state;
        WaypointTarget target;

        // tolerances
        float epsH; // horizontal epsilon
        float epsV; // vertical epsilon
        float epsYaw; // yaw epsilon

        uint32_t holdTimeMs; // time in ms to hold 
        uint32_t holdStartTimeMs;  // time 
        uint32_t entryTimeMs; // time since waypoint activation

        bool reached;


};


#endif