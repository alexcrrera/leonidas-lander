#include "Mission.h"
#include <cmath>

#include "../Config/CoordinateConfig.h"
#include "../StateMachine/StateMachine_utilities.h"
#include "../CommandHandler/CommandHandler.h"
#include "../FlightManager/FlightManager.h"

Mission::Mission(){
    currentWaypointIndex = -1; // index -1 means that we are not dealing with any specific waypoint
    navigationWaypointCount = 0; // no waypoints have been added so far
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


void Mission::defineTakeOff_and_Landing(){
    defineTakeOff();
    defineLanding();
    
   // updateReadiness();
}

void Mission::defineTakeOff(){


    LanderState landerState = flightManager->getLander().getState();
    float takeOff_yaw_deg = MissionConfig::takeOff_yaw_deg; // take off yaw is defined in the config file
    float altitude_m = MissionConfig::takeOff_altitude_m; // take off altitude is defined in the config file
    

    // will only modify the take off position if not active or completed
    if(isActive() || isCompleted()){
        flightManager->getCommandHandler().setErrorFeedback("MISSION LOCKED");
    }

    // verifies if current position is correct (if not out of bounds)
    if(!CoordinateConfig::isValidCoordinate(landerState.position.North_SI, CoordinateType::North_m) ||
   !CoordinateConfig::isValidCoordinate(landerState.position.East_SI, CoordinateType::East_m))
{
    flightManager->getCommandHandler().setFeedback("INV", "TAO POS");
    return;
}


    if(!CoordinateConfig::isValidCoordinate(takeOff_yaw_deg,CoordinateType::Yaw_deg)){
        flightManager->getCommandHandler().setFeedback("INV", "TAO YAW");
        return; // yaw command is too large
    }


    if(!CoordinateConfig::isValidCoordinate(altitude_m,CoordinateType::Altitude_m)){
        flightManager->getCommandHandler().setFeedback("INV", "TAO ALT");
        return; // altitude is invalid
    }


    // retrieves epsilon based on the configuration in FlightRegimeConfig
    const auto& regime = FlightRegimeConfig::TAKEOFF;
    EpsilonGroup epsilon_group = regime.epsilon_group;
    uint32_t holdTimeMs = regime.holdTimeMs;


    // defining the take off point at CURRENT NED position (horizontally)
    
    NED_coordinates hoverPositionNED = {
        landerState.position.North_SI,
        landerState.position.East_SI,
        landerState.position.Down_SI - altitude_m // NED convention, so down is positive
    };


    takeOffWaypoint = Waypoint(
        hoverPositionNED, // NED convention
        takeOff_yaw_deg,
        holdTimeMs,
        epsilon_group);

    // hover definition


    takeOffDefined = true;  // the important flag that is actually used. 
                            // Return value is only for the user

    flightManager->getCommandHandler().setOKFeedback("TAO DEF");

}


void Mission::defineLanding(){
    
    auto landerState = flightManager->getLander().getState();
    
    NED_coordinates currentPositionNED = landerState.position;

    NED_coordinates landingRelativePositionNED = MissionConfig::landingTarget_relative_NED.target.positionNED;
    float yawDeg_landing = MissionConfig::landingTarget_relative_NED.target.yaw_deg;

    float descentStartAltitude_m = MissionConfig::landing_descentStart_altitude_m; // altitude above ground for landing descent start

    auto state_machine = flightManager->getStateMachine();
    auto current_state = state_machine.getState();
    
    if(state_machine.isInFlight(current_state)){ 
        flightManager->getCommandHandler().setFeedback("INV", "LND REQ");
        return; // cannot define landing while in flight
    }
    // redefines the landing position relative to the current position of the lander

   

    if(isActive() || isCompleted()){
        flightManager->getCommandHandler().setFeedback("INV", "LND STATE");
        return;
    }

    if(!takeOffDefined){
        flightManager->getCommandHandler().setFeedback("INV", "TAO NOT DEF");
        return; // cannot define landing if take off is not defined
    }

    const auto& regime_landing = FlightRegimeConfig::LANDING;

    EpsilonGroup epsilon_group = regime_landing.epsilon_group; // tolerances to define LANDING
    uint32_t holdTimeMs = regime_landing.holdTimeMs;


  
    NED_coordinates landingAbsolutePositionNED = {
        currentPositionNED.North_SI + landingRelativePositionNED.North_SI,
        currentPositionNED.East_SI + landingRelativePositionNED.East_SI,
        currentPositionNED.Down_SI + landingRelativePositionNED.Down_SI
    };


   



//     if(!CoordinateConfig::isValidCoordinate(landerState.position.North_SI, CoordinateType::North_m) ||
//    !CoordinateConfig::isValidCoordinate(landerState.position.East_SI, CoordinateType::East_m))
// 

    if(!CoordinateConfig::isValidCoordinate(landingAbsolutePositionNED.North_SI, CoordinateType::North_m) ||
   !CoordinateConfig::isValidCoordinate(landingAbsolutePositionNED.East_SI, CoordinateType::East_m))
    {
        flightManager->getCommandHandler().setFeedback("INV", "LND POS");
        return;
    }

    if(!CoordinateConfig::isValidCoordinate(yawDeg_landing,CoordinateType::Yaw_deg)){
        flightManager->getCommandHandler().setFeedback("INV", "LND YAW");
        return;
    }

     if(!CoordinateConfig::isValidCoordinate(descentStartAltitude_m,CoordinateType::Altitude_m)){
        flightManager->getCommandHandler().setFeedback("INV", "LND ALT");
        return;
    }


    NED_coordinates landingDescentStartPositionNED = {
        landingAbsolutePositionNED.North_SI,
        landingAbsolutePositionNED.East_SI,
        landingAbsolutePositionNED.Down_SI - descentStartAltitude_m // NED convention, so down is positive
    };

    landingTransitionWaypoint = Waypoint(
        landingDescentStartPositionNED, // NED convention
        yawDeg_landing,
        holdTimeMs,
        epsilon_group);


    landingWaypoint = Waypoint(
        landingAbsolutePositionNED, // NED convention
        yawDeg_landing,
        holdTimeMs,
        epsilon_group);


    landingDefined = true; // the important flag that is actually used. 
                            // Return value is only for the user
    flightManager->getCommandHandler().setOKFeedback("LND DEF");

}



void Mission::updateReadiness(){
    // checks wether the mission is ready to go.
    // Requires TAKEOFF and LANDING positions to be defined
    if(takeOffDefined && landingDefined){
        state = MissionState::READY;
        return;
    }
    if(takeOffDefined || !landingDefined){
        
        state = MissionState::NOT_READY_LND_UNDEFINED;
        return;
    }

}


MissionTarget Mission::getTarget(){
    MissionTarget target;
    Waypoint& current_waypoint = getCurrentWaypoint();
    target.target = current_waypoint.getTarget();
   
    return(target);
}





void Mission::start(){
    if(!isReady()){
        flightManager->getCommandHandler().setFeedback("INV", "MISSION NOT READY");
        return;
    }
    state = MissionState::ACTIVE;
    currentWaypointIndex = 0; // start with the take off waypoint
    flightManager->getCommandHandler().setOKFeedback("MISSION STARTED");
}


void Mission::update(){
    
    updateReadiness();

    if(!isActive()){return;}

    if(currentWaypointIndex>=MAX_MISSION_WAYPOINTS){return;} // overflow protection}

    Waypoint& current_waypoint = getCurrentWaypoint();

    NED_coordinates currentWaypointPositionNED = current_waypoint.getTarget().positionNED;
    float currentWaypointYawDeg = current_waypoint.getTarget().yaw_deg;
   
   
   

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



void Mission::getCurrentPositionAsWaypoint(){
        currentPositionWaypoint.updatePosition(
            flightManager->getLander().getState().position,
            flightManager->getLander().getState().attitude.Yaw_SI
        );
        
    
}



Waypoint& Mission::getCurrentWaypoint(){

    if(currentWaypointIndex == -1){
        
        getCurrentPositionAsWaypoint(); // updates the current position waypoint to the current position of the lander
        return(currentPositionWaypoint); // if the current waypoint index is -1, it means that we are not in a mission, so we return the current position as a waypoint
    }
    if(currentWaypointIndex==0){
        return(takeOffWaypoint);
    }
    if(currentWaypointIndex>0 && currentWaypointIndex<=navigationWaypointCount){
        return(navigationWaypoints[currentWaypointIndex-1]); // -1 because the first navigation waypoint is at index 0
    }
    if(currentWaypointIndex==navigationWaypointCount+1){
        return(landingTransitionWaypoint);
    }
    if(currentWaypointIndex==navigationWaypointCount+2){
        return(landingWaypoint);
    }

    // if we reach here, it means that the current waypoint index is invalid
    // we will return a default waypoint (take off waypoint) to avoid returning a reference to an invalid object
    return(takeOffWaypoint);    
}




void Mission::advanceWaypoint(){
    // activates the next waypoint in the mission sequence, and updates the mission state accordingly
    if(currentWaypointIndex==0){
        currentWaypointIndex++;
        state = MissionState::ACTIVE;
        return;
    }
    if(currentWaypointIndex>0 && currentWaypointIndex<navigationWaypointCount+2){
        currentWaypointIndex++;
        return;
    }
    if(currentWaypointIndex==navigationWaypointCount+2){
        currentWaypointIndex++;
        state = MissionState::COMPLETED;
        return;
    }
}


bool Mission::addWaypoint( const NED_coordinates& positionNED,
            float yaw_deg,
            uint32_t holdTimeMs = MissionConfig::holdTimeMs.default_,
            EpsilonGroup epsilon_group = {} // default parameters if left blank
            ){

            waypoints_list.push_back(Waypoint(
                positionNED,
                yaw_deg,
                holdTimeMs,
                epsilon_group

            ));


            return true;
        }



String Mission::getMissionDataAsString() const{
    auto current_waypoint = getCurrentWaypoint();
    String data = "Mission State: " + getStateAsString() + "\n";
    data += "Current Waypoint Index: " + String(currentWaypointIndex) + "\n";
    data += "Current Waypoint Data:\n";
    data += current_waypoint.getDataAsString();
    
    return data;
}