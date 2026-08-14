#include "StateMachine.h"


String StateMachine::getStateAsString() const{
    switch (current_state) {
        case STATE_MACHINE_STATES::BOOTED:
            return "BOOTED";

        case STATE_MACHINE_STATES::BOOT:
            return "BOOTING";
        case STATE_MACHINE_STATES::STANDBY:
            return "STANDBY";
        case STATE_MACHINE_STATES::DESCENT:
            return "DESCENT";
        case STATE_MACHINE_STATES::LANDING:
            return "LANDING";
        case STATE_MACHINE_STATES::LANDED:
            return "LANDED";
        default:
            return "UNKNOWN STATE";
    }
}



void StateMachine::begin(FlightManager* flight_manager_){
    // initialize the state machine with a pointer to the flight manager
    flight_manager = flight_manager_;
    current_state = STATE_MACHINE_STATES::BOOT;
}

void StateMachine::update(){


}


void StateMachine::setState(STATE_MACHINE_STATES new_state){
    current_state = new_state;
}




