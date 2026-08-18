#include "Mission.h"
#include <cmath>

#include "../Config/CoordinateConfig.h"
#include "../StateMachine/StateMachine_utilities.h"
#include "../CommandHandler/CommandHandler.h"
#include "../FlightManager/FlightManager.h"

Mission::Mission(){
    
   
    state = MissionState::NOT_READY_TO_AND_LND_UNDEFINED;
}


void Mission::begin(FlightManager* flightManager_){
    flightManager = flightManager_;
    takeOffDefined = false;
    landingDefined = false;
   
    defineTakeOff_and_Landing(); // defines the take off and landing points based on the current position of the lander

    currentPositionWaypoint = Waypoint(
        flightManager->getLander().getState().position, // current position of the lander
        flightManager->getLander().getState().attitude.Yaw_SI, // current yaw of the lander
        0, // hold time is 0 because we are not holding at this point
        {} // default epsilon group
    );



   
}


void Mission::start(){
    if(!isReady()){
        flightManager->getCommandHandler().setFeedback("INV", "MISSION NOT READY");
        return;
    }
    state = MissionState::ACTIVE;
    currentWaypointIndex = -1; // start with the take off waypoint
    flightManager->getCommandHandler().setOKFeedback("MISSION STARTED");
}





void Mission::updateReadiness(){
    // checks wether the mission is ready to go.
    // Requires TAKEOFF and LANDING positions to be defined
    //

    if(state == MissionState::ACTIVE || state == MissionState::COMPLETED){
        return; // if the mission is active or completed, we don't need to check readiness
    }

    if(takeOffDefined || !landingDefined){
        
        state = MissionState::NOT_READY_LND_UNDEFINED;
        return;
    }
    if(!takeOffDefined || landingDefined){
        state = MissionState::NOT_READY_TO_UNDEFINED;
        return;
    }

}


MissionTarget Mission::getTarget(){
    MissionTarget target;
    Waypoint& current_waypoint = getCurrentWaypoint();
    target.target = current_waypoint.getTarget();
   
    return(target);
}








void Mission::update(){
    
    updateReadiness();

    if(!isActive()){return;}

    if(currentWaypointIndex>=MAX_MISSION_WAYPOINTS){return;} // overflow protection}

     Waypoint& waypt = getCurrentWaypoint();

    if(isTargetReached()){
        
       
    if(waypt.getHoldStartTimeMs()==0){

        flightManager->debug_text = "Mission: Target reached, starting hold timer for waypoint " + String(currentWaypointIndex);
        Serial.println("Mission: Target reached, starting hold timer for waypoint " + String(currentWaypointIndex));
        waypt.setHoldStartTimeMs(millis()); // start the hold timer
        state = MissionState::HOLDING;
        return;
    }


    if(millis() - waypt.getHoldStartTimeMs() >= waypt.getHoldTimeMs()){
            advanceWaypoint();
            flightManager->debug_text = "Mission: Hold time completed, advancing to next waypoint " + String(currentWaypointIndex);
            Serial.println("Mission: Hold time completed, advancing to next waypoint " );
            state = MissionState::MOVING;
            return;
        }
    }
    
   

}





bool Mission::isTargetReached(){
    // checks if the current waypoint has been reached based on the current position and yaw of the lander
    Waypoint& waypt = getCurrentWaypoint();
    NED_coordinates target_positionNED = waypt.getTarget().positionNED;
    float target_yaw_deg = waypt.getTarget().yaw_deg;

    NED_coordinates currentPositionNED = flightManager->getLander().getState().position;
    float currentYawDeg = flightManager->getLander().getState().attitude.Yaw_SI;

    if(Utilities::isWithinEps_NED(waypt.getEpsilonGroup().epsH, waypt.getEpsilonGroup().epsV, currentPositionNED, target_positionNED) &&
       Utilities::isWithinEps_Yaw(waypt.getEpsilonGroup().epsYaw, currentYawDeg, target_yaw_deg)) {
        return true;
    }
    return false;
}








Waypoint& Mission::getCurrentWaypoint(){

    if(currentWaypointIndex<-1 ){
        return(currentPositionWaypoint); // if the current waypoint index is -2 or less , return the current position waypoint
    }


    if(currentWaypointIndex==-1){
        return(takeOffWaypoint); // if the current waypoint index is -1, return the take off waypoint
    }

    if(currentWaypointIndex < getNavigationWaypointCount()){
        return(waypoints_list_nav_only[currentWaypointIndex]); // if the current waypoint index is 0 or more, return the corresponding navigation waypoint
    }

    if(currentWaypointIndex == getNavigationWaypointCount()){
        return(landingTransitionWaypoint); // if the current waypoint index is equal to the number of navigation waypoints, return the landing waypoint
    }

    // if we reach here, it means that the nav waypoints have been completed, and we are now at the landing waypoint

    // we will return a default waypoint (take off waypoint) to avoid returning a reference to an invalid object
    return(landingWaypoint); // if the current waypoint index is greater than the number of navigation waypoints, return the landing waypoint
}




void Mission::advanceWaypoint(){
    // activates the next waypoint in the mission sequence, and updates the mission state accordingly
    if(!isActive()){return;} // if the mission is not active, do nothing
    currentWaypointIndex++;
    auto navigationWaypointCount = getNavigationWaypointCount();
    if(currentWaypointIndex>navigationWaypointCount+1){ // if we have completed the landing waypoint, we are done with the mission
        state = MissionState::COMPLETED;
        return;
    }
}





