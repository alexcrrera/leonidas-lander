#include "FlightGuard.h"
#include "../FlightManager/FlightManager.h"
    
void FlightGuard::begin(FlightManager* manager) {
    flight_manager = manager;
    // Initialization code for FlightGuard
}

void FlightGuard::update() {
    // Update code for FlightGuard

    auto regimeData = flight_manager->getStateMachine().getCurrentFlightRegimeData();


    if(!regimeData.isFlying){
        overrideFlags.EDF_enabled = false;
        overrideFlags.TVC_enabled = false;
       
    }

    checkFlightConditions();
  
}




void FlightGuard::checkFlightConditions() {

    auto& state = flight_manager->getLander().getState();
    FlightGuardUtilities::resetOverrideFlags(overrideFlags);
    
    auto& stateMachine = flight_manager->getStateMachine();

    overrideFlags.roll_pitch_ok =
        FlightGuardUtilities::isRollPitchWithinBounds(state.attitude.Pitch_SI,state.attitude.Roll_SI);

    overrideFlags.yaw_ok =
        FlightGuardUtilities::isYawWithinBounds(state.attitude.Yaw_SI);

    overrideFlags.vertical_overspeed_ok =
        FlightGuardUtilities::isVerticalVelocityWithinBounds(state.velocity.Down_SI);

    overrideFlags.horizontal_overspeed_ok =
        FlightGuardUtilities::isHorizontalVelocityWithinBounds(state.velocity.East_SI,state.velocity.North_SI);






}


