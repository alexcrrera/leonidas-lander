#include "StateMachine.h"


String StateMachine::getStateAsString() const{
    switch (current_state) {
        case STATE_MACHINE_STATES::BOOTED:
            return "BOOTED";
        

        case STATE_MACHINE_STATES::LAUNCH:
            return "LAUNCH";

    
        case STATE_MACHINE_STATES::NOGO:
            return "NO GO - GROUNDED";

        case STATE_MACHINE_STATES::BOOT:
            return "BOOTING";
;

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

STATE_MACHINE_STATES StateMachine::getState() const{
    return(current_state);

}

void StateMachine::setState(STATE_MACHINE_STATES new_state){
    current_state = new_state;
}




bool StateMachine::isInFlight(STATE_MACHINE_STATES state)const {
    return (state == STATE_MACHINE_STATES::SPOOL_UP ||
            state == STATE_MACHINE_STATES::LAUNCH ||
            
            
            state == STATE_MACHINE_STATES::LANDING ||
            state == STATE_MACHINE_STATES::SPOOL_DOWN
        );
}


FlightRegimeData StateMachine::getCurrentFlightRegimeData() const {
    

    switch(current_state){

            
        case STATE_MACHINE_STATES::SPOOL_UP:
            return FlightRegimeConfig::TAKEOFF;

        case STATE_MACHINE_STATES::LAUNCH:
            return FlightRegimeConfig::TAKEOFF;




        case STATE_MACHINE_STATES::NAVIGATION:
            return FlightRegimeConfig::NAVIGATION;



        case STATE_MACHINE_STATES::PRE_LANDING:
            return FlightRegimeConfig::PRE_LANDING;

        case STATE_MACHINE_STATES::LANDING:
            return FlightRegimeConfig::LANDING;

        case STATE_MACHINE_STATES::SPOOL_DOWN:
            return FlightRegimeConfig::GROUND; // after landing, the lander is grounded

        

        default:
            return FlightRegimeConfig::GROUND;
    }
}

