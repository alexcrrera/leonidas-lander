#pragma once

#ifndef MISSION_H
#define MISSION_H


#include <Arduino.h>
#include "../Waypoint/Waypoint.h"
#include "../Config/EpsConfig.h"
#include "../Utilities/Utilities.h"
#include "../Config/MissionConfig.h"
#include "../Lander/Lander_structs.h"
#include <vector>

class FlightManager; // forward declaration to avoid circular dependency

constexpr uint8_t MAX_MISSION_WAYPOINTS = 5;

enum class MissionState {
    NOT_READY_TO_AND_LND_UNDEFINED, // both not defnied
    NOT_READY_TO_UNDEFINED, // take off params not defined
    NOT_READY_LND_UNDEFINED, // landing not defined
    READY, // ready to begin
    ACTIVE, // currently outputting targets
    COMPLETED // mission done
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
        Mission(); 

        void begin(FlightManager* flightManager_);
        
        
        void start();

        void update(); // gets called via the FlightManager update() function, which is called in the main loop

        MissionTarget getTarget(); // RETURNS copy of the target, not a reference to avoid dangling references
       // MissionState getState() const;


        // getters
        bool isReady() const;
        bool isActive() const;
        bool isCompleted() const;


        

        String getStateAsString() const;
        String getMissionDataAsString() const;

// TEMP PUBLIC FOR TESTING
        Waypoint takeOffWaypoint;
        Waypoint navigationWaypoints[MAX_MISSION_WAYPOINTS];
        Waypoint landingWaypoint;
        Waypoint landingTransitionWaypoint; 



    private:


        void defineTakeOff_and_Landing();
        void defineTakeOff();
        void defineLanding();



        bool addWaypoint(
            const NED_coordinates& positionNED,
            float yaw_deg,
            uint32_t holdTimeMs,
            EpsilonGroup epsilon_group // default parameters if left blank
        );
        
        // list/ array of waypoints for the mission
        std::vector<Waypoint> waypoints_list;
        void updateReadiness();
        Waypoint& getCurrentWaypoint();

        void advanceWaypoint();

        MissionState state;

        bool takeOffDefined;
        bool landingDefined;

        
        
        void getCurrentPositionAsWaypoint();
        uint8_t navigationWaypointCount;
        int currentWaypointIndex = -1; // index of the current waypoint in the mission (0 = take off, 1 = first navigation waypoint, 2 = second navigation waypoint, etc.)

        FlightManager* flightManager=nullptr; // pointer to the flight manager to access lander state and other modules


        Waypoint currentPositionWaypoint; // waypoint that represents the current position of the lander, used when the mission is not active to avoid showing an unactive waypoint as the target
};




#endif