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

    if(state == MissionState::HOLDING){
        return; // if the mission is holding, we don't need to check readiness
    }

    // else

    if(takeOffDefined && !landingDefined){
        
        state = MissionState::NOT_READY_LND_UNDEFINED;
        return;
    }
    if(!takeOffDefined && landingDefined){
        state = MissionState::NOT_READY_TO_UNDEFINED;
        return;
    }

    if(!takeOffDefined && !landingDefined){
        state = MissionState::NOT_READY_TO_AND_LND_UNDEFINED;
        return;
    }

    if(takeOffDefined && landingDefined){
        state = MissionState::READY;
        return;
    }


}






void Mission::update(){

    auto navigationWaypointCount = getNavigationWaypointCount();
    if(currentWaypointIndex>navigationWaypointCount+2){ // if we have completed the landing waypoint, we are done with the mission
        state = MissionState::COMPLETED;
    return;
    }

    
    updateReadiness();

    if(!isActive()){return;} // mission is not active, do nothing

    if(currentWaypointIndex>=MAX_MISSION_WAYPOINTS){return;} // overflow protection}

    Waypoint& waypt = getCurrentWaypoint();

    auto landerState = flightManager->getLander().getState();


    // if active then we chcek if we've reached the target
    if(state == MissionState::ACTIVE){
        if(waypt.isReached(landerState.position, landerState.attitude.Yaw_SI)){
            flightManager->debug_text = "Mission: Target reached, starting hold timer for waypoint " + String(currentWaypointIndex);
            Serial.println("Mission: Target reached, starting hold timer for waypoint " + String(currentWaypointIndex));
            waypt.setHoldStartTimeMs(millis()); // start the hold timer
            state = MissionState::HOLDING;
            return;
        }
        return; // if we are active and haven't reached the target, do nothing
    }


    // if we are holding, we check if the hold time has elapsed, regardless of current attitude and position, we will advance to the next waypoint after the hold time has elapsed
    
    if(state == MissionState::HOLDING){
            if(millis() - waypt.getHoldStartTimeMs() >= waypt.getHoldTimeMs()){
                advanceWaypoint();
                flightManager->debug_text = "Mission: Hold time completed, advancing to next waypoint " + String(currentWaypointIndex);
                Serial.println("Mission: Hold time completed, advancing to next waypoint " );
                state = MissionState::ACTIVE;
                return;
            }
        
        }
    

   
    
   
}




Waypoint& Mission::getCurrentWaypoint(){

    auto& state_machine = flightManager->getStateMachine();

    if(currentWaypointIndex<-1 ){

        state_machine.requestStateChange(STATE_MACHINE_STATES::AWAIT); // if the current waypoint index is less than -1, we are in an invalid state, so we request a state change to NOGO
        return(currentPositionWaypoint); // if the current waypoint index is -2 or less , return the current position waypoint
    }

    if(currentWaypointIndex==-1){
        state_machine.requestStateChange(STATE_MACHINE_STATES::LAUNCH); // if the current waypoint index is -1, we are in the take off phase, so we request a state change to LAUNCH
        return(takeOffWaypoint); // if the current waypoint index is -1, return the take off waypoint
    }


    if(currentWaypointIndex > -1 && currentWaypointIndex < getNavigationWaypointCount()){
        state_machine.requestStateChange(STATE_MACHINE_STATES::NAVIGATION); // if the current waypoint index is 0 or more, we are in the navigation phase, so we request a state change to NAVIGATION

        return(waypoints_list_nav_only[currentWaypointIndex]); // if the current waypoint index is 0 or more, return the corresponding navigation waypoint
    }


    
    if(currentWaypointIndex== getNavigationWaypointCount() ){
        state_machine.requestStateChange(STATE_MACHINE_STATES::PRE_LANDING); // if the current waypoint index is 0 and there are no navigation waypoints, we are in the pre-landing phase, so we request a state change to PRE_LANDING
        return(landingTransitionWaypoint); // if the current waypoint index is 0 and there are no navigation waypoints, return the landing transition waypoint
    }



    if(currentWaypointIndex == getNavigationWaypointCount()+1){
            // if we reach here, it means that the nav waypoints have been completed, and we are now at the landing waypoint

        state_machine.requestStateChange(STATE_MACHINE_STATES::LANDING); // if the current waypoint index is greater than the number of navigation waypoints, we are in the landing phase, so we request a state change to LANDING
        return(landingWaypoint); // if the current waypoint index is greater than the number of navigation waypoints, return the landing waypoint
    }

    state_machine.requestStateChange(STATE_MACHINE_STATES::AWAIT); // if the current waypoint index is greater than the number of navigation waypoints, we are in an invalid state, so we request a state change to GROUND

    state = MissionState::COMPLETED;
    // we will return a default waypoint (take off waypoint) to avoid returning a reference to an invalid object
    return(currentPositionWaypoint); // if the current waypoint index is greater than the number of navigation waypoints, return the landing waypoint
}




void Mission::advanceWaypoint(){
    // activates the next waypoint in the mission sequence, and updates the mission state accordingly
    if(!isActive()){return;} // if the mission is not active, do nothing
    currentWaypointIndex++;
    auto navigationWaypointCount = getNavigationWaypointCount();

    if(currentWaypointIndex>navigationWaypointCount+1){ // if we have completed the pre_landing, landing waypoints, we are done with the mission
        state = MissionState::COMPLETED;
        return;
    }
}



MissionTarget Mission::getTarget(){
    MissionTarget target;
    Waypoint& current_waypoint = getCurrentWaypoint();
    target.target = current_waypoint.getTarget();
   
    return(target);
}