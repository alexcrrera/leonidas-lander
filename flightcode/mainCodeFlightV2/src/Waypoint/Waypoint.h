#pragma once
#ifndef WAYPOINT_H
#define WAYPOINT_H


#include <Arduino.h>
#include "../Utilities/Utilities.h"
#include "../Config/EpsConfig.h"
#include "../Config/MissionConfig.h"






class Waypoint{


    public:


    Waypoint(); // default constructor as a placeholder for a waypoint that is not defined yet
    Waypoint(
        WaypointType type,
        const NED_coordinates& positionNED,
        float yaw_deg,
        uint32_t holdTimeMs = MissionConfig::holdTimeMs.default_,
        EpsilonGroup epsilon_group = {}
    );

    void update(
        const NED_coordinates& currentPositionNED,
        float yaw_deg
    );

    bool isReached() const;

    void activate();


    WaypointType getType() const; // getter
    WaypointState getState() const;

    const WaypointTarget& getTarget() const;

    bool isActive();

    void updatePosition(const NED_coordinates& positionNED, float yaw_deg);

    String getStateAsString() const{
        switch (state) {
            case WaypointState::INACTIVE:
                return "INACTIVE";
            case WaypointState::APPROACHING:
                return "APPROACHING";
            case WaypointState::HOLDING:
                return "HOLDING";
            case WaypointState::COMPLETED:
                return "COMPLETED";
            default:
                return "UNKNOWN STATE";
        }
    }
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

        bool positionReached() const;
        bool yawReached() const;
        bool targetReached() const{return(positionReached() && yawReached());}

};


#endif