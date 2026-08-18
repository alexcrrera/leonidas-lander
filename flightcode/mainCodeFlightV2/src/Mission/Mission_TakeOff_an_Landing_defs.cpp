#include "Mission.h"
#include <cmath>

#include "../Config/CoordinateConfig.h"
#include "../StateMachine/StateMachine_utilities.h"
#include "../CommandHandler/CommandHandler.h"
#include "../FlightManager/FlightManager.h"
#include "../Config/FlightRegimeConfig.h"


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



