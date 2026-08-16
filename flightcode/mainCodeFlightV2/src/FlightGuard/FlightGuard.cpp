#include "FlightGuard.h"

void FlightGuard::begin(FlightManager* manager) {
    flight_manager = manager;
    // Initialization code for FlightGuard
}

void FlightGuard::update() {
    // Update code for FlightGuard
}




// void FlightGuard::checkFlightConditions() {

//     auto& state = flight_manager->getLander().getState();

//     // overspeedCheck();
//     // altitudeLimitCheck();
//     // NEDBoundaryCheck();
//     // Check flight conditions and set override flags accordingly
// }