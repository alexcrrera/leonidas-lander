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
        const NED_coordinates& positionNED,
        float yaw_deg,
        uint32_t holdTimeMs = MissionConfig::holdTimeMs.default_,
        EpsilonGroup epsilon_group = {}
    );




    const WaypointTarget& getTarget() const;

    void updatePosition(const NED_coordinates& positionNED, float yaw_deg);
    
    String getDataAsString() const;
 
    private:

     

        //
        bool constructed = false; // indicates if the waypoint has been constructed or not
        

        WaypointTarget target; // target position and yaw
        EpsilonGroup epsilon_group; // tolerances for position and yaw

        uint32_t holdTimeMs; // time in ms to hold 
        uint32_t holdStartTimeMs;  // time 
        uint32_t entryTimeMs; // time since waypoint activation
};


#endif