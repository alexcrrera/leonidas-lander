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
  
}




// void FlightGuard::checkFlightConditions() {

//     auto& state = flight_manager->getLander().getState();

//     // overspeedCheck();
//     // altitudeLimitCheck();
//     // NEDBoundaryCheck();
//     // Check flight conditions and set override flags accordingly
// }