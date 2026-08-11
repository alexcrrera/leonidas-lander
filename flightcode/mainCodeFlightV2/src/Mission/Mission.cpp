#include "Mission.h"
#include <cmath>
#include "../Config/FlightRegimeConfig.h"
#include "../Config/CoordinateConfig.h"



Mission::Mission(){
    currentWaypointIndex = 0; // index 0 means that we are dealing with the TAKEOFF scenario
    navigationWaypointCount = 0; // no waypoints have been added so far
    state = MissionState::NOT_READY_TO_AND_LND_UNDEFINED;
}

MissionStatus Mission::defineTakeOff(
            const Vector3& currentPositionNED,
            float altitude_m,
            float yawDeg){


    // will only modify the take off position if not active or completed
    if(state == MissionState::ACTIVE || state == MissionState::COMPLETED){
        return(MissionStatus::PROTECTED);
    }

    // verifies if current position is correct (if not out of bounds)
    if(!CoordinateConfig::isValidPositionNED(currentPositionNED)){
        return(MissionStatus::INVALID_POSITION);
    }

    if(!CoordinateConfig::isValidCoordinate(yawDeg,CoordinateType::Yaw_deg)){
        return(MissionStatus::INVALID_YAW); // yaw command is too large
    }

    if(!CoordinateConfig::isValidCoordinate(altitude_m,CoordinateType::Altitude_m)){
        return(MissionStatus::INVALID_ALTITUDE); // altitude is invalid
    }

    // retrieves epsilon based on the configuration in FlightRegimeConfig

    const auto& regime = FlightRegimeConfig::TAKEOFF;

    EpsilonGroup epsilon_group = regime.epsilon_group;
    uint32_t holdTimeMs = regime.holdTimeMs;


    // defining the take off point at CURRENT NED position (horizontally)
    takeOffWaypoint = Waypoint(
        WaypointType::TAKEOFF,
        {currentPositionNED.x,currentPositionNED.y,currentPositionNED.z-altitude_m}, // NED convention
        yawDeg,
        holdTimeMs,
        epsilon_group);
    
    

    takeOffDefined = true;  // the important flag that is actually used. 
                            // Return value is only for the user

    landingDefined = false; // forces user to define LANDING after take-off
                            // this prevents errors when changing the TO
                            // position - LND needs current vehicule's position...
    
    updateReadiness(); 

    return(MissionStatus::SUCCESS);

}



MissionStatus Mission::defineLanding(const Vector3& currentPositionNED, 
                            const Vector3& landingRelativePositionNED, 
                            float descentStartAltitude_m,
                            float yawDeg){

    if(!landingDefined){ // CANNOT DEFINE LANDING IF TAO POSITION NOT PREDEFINED AND LOCKED
        return(MissionStatus::INVALID_REQUEST);
    }

    if(state == MissionState::ACTIVE || state == MissionState::COMPLETED){
        return(MissionStatus::PROTECTED);
    }

    const auto& regime_landing = FlightRegimeConfig::LANDING;

    EpsilonGroup epsilon_group = regime_landing.epsilon_group; // tolerances to define LANDING
    uint32_t holdTimeMs = regime_landing.holdTimeMs;



    Vector3 LandingSpot = { currentPositionNED.x + landingRelativePositionNED.x,
                            currentPositionNED.y + landingRelativePositionNED.y,
                            currentPositionNED.z  + landingRelativePositionNED.z };

    if(!CoordinateConfig::isValidPositionNED(currentPositionNED)){
        return(MissionStatus::INVALID_POSITION);
    }

    if(!CoordinateConfig::isValidCoordinate(yawDeg,CoordinateType::Yaw_deg)){
        return(MissionStatus::INVALID_YAW); // yaw command is too large
    }

     if(!CoordinateConfig::isValidCoordinate(descentStartAltitude_m,CoordinateType::Altitude_m)){
        return(MissionStatus::INVALID_ALTITUDE); // altitude is invalid
    }



    takeOffWaypoint = Waypoint(
        WaypointType::LANDING,
        {LandingSpot.x,LandingSpot.y,LandingSpot.z}, // NED convention
        yawDeg,
        holdTimeMs,
        epsilon_group);

 


    // define transition point
    const auto& regime_approach = FlightRegimeConfig::PRE_LANDING;
    EpsilonGroup epsilon_group_approach = regime_approach.epsilon_group; // tolerances to define PRE_LANDING
    uint32_t holdTimeMs_approach = regime_approach.holdTimeMs;

    landingApproachWaypoint = Waypoint(
            WaypointType::LANDING,
            {LandingSpot.x,LandingSpot.y,LandingSpot.z-descentStartAltitude_m}, // NED convention
            yawDeg,
            holdTimeMs_approach,
            epsilon_group_approach);

    
    
    
    landingDefined = true;

    updateReadiness();
    



    return(MissionStatus::SUCCESS);

}


void Mission::updateReadiness(){

   
    // checks wether the mission is ready to go.
    // Requires TAKEOFF and LANDING positions to be defined
    if(takeOffDefined && landingDefined){
        state = MissionState::READY;
        return;
    }
    if(takeOffDefined){
        

    }





}


void Mission::update(const Vector3& currentPositionNED,float currentYawDeg){
    
    updateReadiness();

    if(!isActive()){
        return;
    }

    if(currentWaypointIndex>=MAX_MISSION_WAYPOINTS){
        return; // overflow protection
    }

    Waypoint* current_waypoint = getCurrentWaypoint();

    current_waypoint->update(currentPositionNED,currentYawDeg);

    if(current_waypoint->getState()==WaypointState::COMPLETED){
        advanceWaypoint();
    }

}


bool Mission::isReady() const{
    if(state == MissionState::READY){
        return(true);
    }
    return(false);
}

bool Mission::isCompleted() const{
    if(state == MissionState::COMPLETED){
        return(true);
    }
    return(false);
}

bool Mission::isActive() const{
    if(state == MissionState::ACTIVE){
        return(true);
    }
    return(false);
}