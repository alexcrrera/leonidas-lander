#include "StateMachine.h"
#include "../FlightManager/FlightManager.h"


String StateMachine::getStateAsString() const{
    return getStateAsString(current_state);
}



String StateMachine::getStateAsString(STATE_MACHINE_STATES state) const{
    switch (state) {
        case STATE_MACHINE_STATES::AWAIT:
            return "AWAIT";
        

        case STATE_MACHINE_STATES::LAUNCH:
            return "LAUNCH";

    
        case STATE_MACHINE_STATES::NOGO:
            return "NO GO - GROUNDED";

        case STATE_MACHINE_STATES::BOOT:
            return "BOOTING";


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

    if(isInFlight(current_state)){
        flight_manager->getFlightGuard().overrideFlags.EDF_enabled = true;
        flight_manager->getFlightGuard().overrideFlags.TVC_enabled = true;
    }
    else{
        flight_manager->getFlightGuard().overrideFlags.EDF_enabled = false;
        flight_manager->getFlightGuard().overrideFlags.TVC_enabled = false;
    }
}








STATE_MACHINE_STATES StateMachine::getState() const{
    return(current_state);

}

void StateMachine::requestStateChange(STATE_MACHINE_STATES new_state){

    if(new_state == current_state){

        return;
    }
    
    if(verifyChange(new_state)){
        setState(new_state);
        auto& command_handler = flight_manager->getCommandHandler();
        command_handler.setOKFeedback(getStateAsString());
        return;
    }

    else{
        auto& command_handler = flight_manager->getCommandHandler();
        command_handler.setErrorFeedback("NO: " + getStateAsString(new_state));
        return;
    }
}


bool StateMachine::verifyChange(STATE_MACHINE_STATES new_state){

// This function can be used to request a state change, but the actual state change will be handled by the flight manager based on the current state and conditions
    // For now, we will just set the state directly
    if(new_state == STATE_MACHINE_STATES::ABORT){
        return true;
    }

    if(new_state == STATE_MACHINE_STATES::LAUNCH){
        if(!handleTakeOff_request()){
            return false;
        }
    }

    return true;

}


void StateMachine::setState(STATE_MACHINE_STATES new_state){
    current_state = new_state;
}




bool StateMachine::isInFlight(STATE_MACHINE_STATES state)const {
    return (
            state == STATE_MACHINE_STATES::LAUNCH ||

            state == STATE_MACHINE_STATES::POST_LAUNCH_HOVER ||

            state == STATE_MACHINE_STATES::NAVIGATION ||

            state == STATE_MACHINE_STATES::PRE_LANDING ||
            
            state == STATE_MACHINE_STATES::LANDING 
           
        );
}


FlightRegimeData StateMachine::getCurrentFlightRegimeData() const {
    

    switch(current_state){

            
      

        case STATE_MACHINE_STATES::LAUNCH:
            return FlightRegimeConfig::TAKEOFF;

         case STATE_MACHINE_STATES::POST_LAUNCH_HOVER:
            return FlightRegimeConfig::TAKEOFF; 

    
            
            case STATE_MACHINE_STATES::NOGO:
            return FlightRegimeConfig::GROUND;




        case STATE_MACHINE_STATES::NAVIGATION:
            return FlightRegimeConfig::NAVIGATION;



        case STATE_MACHINE_STATES::PRE_LANDING:
            return FlightRegimeConfig::PRE_LANDING;

        case STATE_MACHINE_STATES::LANDING:
            return FlightRegimeConfig::LANDING;

       
        

        default:
            return FlightRegimeConfig::GROUND;
    }
}



