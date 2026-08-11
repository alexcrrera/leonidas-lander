#pragma once

#ifndef MISSION_H
#define MISSION_H


#include <Arduino.h>
#include "../Waypoint/Waypoint.h"
#include "../Config/EpsConfig.h"
#include "../Utilities/Utilities.h"
#include "../Config/MissionConfig.h"

constexpr uint8_t MAX_MISSION_WAYPOINTS = 5;

enum class MissionState {
    NOT_READY_TO_AND_LND_UNDEFINED, // both not defnied
    NOT_READY_TO_UNDEFINED, // take off params not defined
    NOT_READY_LND_UNDEFINED, // landing not defined
    READY, // ready to begin
    ACTIVE, // currently outputting targets
    COMPLETED // mission done
};


struct MissionTarget {
    WaypointTarget target; // position in NED and YAW
    WaypointType type; // take off, landing, navig..
};

enum class MissionStatus {
    SUCCESS,
    INVALID_POSITION,
    INVALID_YAW,
    INVALID_ALTITUDE,
    PROTECTED,
    INVALID_REQUEST
};


class Mission{

    public:
        Mission(); // no params yet
        
        MissionStatus defineTakeOff(
            const Vector3& currentPositionNED,
            float altitude_m, // altitude above ground
            float yawDeg);

        MissionStatus defineLanding(const Vector3& currentPositionNED, 
                            const Vector3& landingRelativePositionNED, // relative offset to the take-off point
                            float descentStartAltitude_m,
                            float yawDeg);

        bool addWaypoint(
            const Vector3& positionNED,
            float yawDeg,
            uint32_t holdTimeMs = MissionConfig::holdTimeMs.default_,
            EpsilonGroup epsilon_group = {} // default parameters if left blank
        );

        bool start();

        void update(const Vector3& currentPositionNED,float currentYawDeg);

        MissionTarget getTarget() const;

        MissionState getState() const;

        bool isReady() const;
        bool isActive() const;
        bool isCompleted() const;

    private:

        void defineLandingApproach(const Vector3& landingPositionNED,
                                   float descentStartAltitude_m,
                                   float yawDeg                            
        );
        

        void updateReadiness();
        Waypoint* getCurrentWaypoint();

        void advanceWaypoint();

        MissionState state;

        bool takeOffDefined;
        bool landingDefined;

        Waypoint takeOffWaypoint;
        Waypoint navigationWaypoints[MAX_MISSION_WAYPOINTS];
        Waypoint landingWaypoint;
        Waypoint landingApproachWaypoint;
        uint8_t navigationWaypointCount;
        uint8_t currentWaypointIndex;

};




#endif