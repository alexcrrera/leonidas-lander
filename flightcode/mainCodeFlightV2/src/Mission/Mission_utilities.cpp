#include "Mission.h"
#include <cmath>

#include "../Config/CoordinateConfig.h"
#include "../StateMachine/StateMachine_utilities.h"
#include "../CommandHandler/CommandHandler.h"
#include "../FlightManager/FlightManager.h"


String Mission::getMissionDataAsString() const{
    auto current_waypoint = getCurrentWaypoint();
    String data = "Mission State: " + getStateAsString() + "\n";
    data += "Current Waypoint Index: " + String(currentWaypointIndex) + "\n";
    data += "Current Waypoint Data:\n";
    data += current_waypoint.getDataAsString();
    data += "Navigation Waypoint Count: " + String(getNavigationWaypointCount()) + "\n";

    
    return data;
}


bool Mission::addWaypoint( const NED_coordinates& positionNED,
            float yaw_deg,
            uint32_t holdTimeMs = MissionConfig::holdTimeMs.default_,
            EpsilonGroup epsilon_group = {} // default parameters if left blank
            ){

            waypoints_list_nav_only.push_back(Waypoint(
                positionNED,
                yaw_deg,
                holdTimeMs,
                epsilon_group

            ));


            return true;
        }


void Mission::getCurrentPositionAsWaypoint(){
    // updates the currentPositionWaypoint to the current position of the lander, so that when the mission is not active, the target is set to the current position of the lander instead of an unactive waypoint
        currentPositionWaypoint.updatePosition(
            flightManager->getLander().getState().position,
            flightManager->getLander().getState().attitude.Yaw_SI); 
}




String Mission::getStateAsString() const{
    switch (state) {
        case MissionState::NOT_READY_TO_AND_LND_UNDEFINED:
            return "NOT READY: TAKEOFF AND LANDING UNDEFINED";
        case MissionState::NOT_READY_TO_UNDEFINED:
            return "NOT READY: TAKEOFF UNDEFINED";
        case MissionState::NOT_READY_LND_UNDEFINED:
            return "NOT READY: LANDING UNDEFINED";
        case MissionState::READY:
            return "READY";
        case MissionState::ACTIVE:
            return "ACTIVE";
        case MissionState::COMPLETED:
            return "COMPLETED";
        default:
            return "UNKNOWN STATE";
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


    if(state == MissionState::HOLDING){
        return(true);
    }
    return(false);
}


int Mission::getNavigationWaypointCount() {
    // calculate number of elements in std::vector<Waypoint> waypoints_list;
    
    return waypoints_list_nav_only.size();
}