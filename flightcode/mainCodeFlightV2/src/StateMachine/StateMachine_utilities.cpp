#include "StateMachine_utilities.h"
#include "../FlightManager/FlightManager.h"
#include "../StateMachine/StateMachine.h"


bool StateMachine::handleTakeOff_request(){


    if(isInFlight(current_state)){
        Serial.println("StateMachine: Already in flight, cannot take off");
        return false; 
    }

    auto& flight_guard = flight_manager->getFlightGuard();
    
    if(!flight_guard.isFlightConditionValid()){
        Serial.println("flight_guard: Flight conditions not valid, cannot take off");
        return false;
    }


    
    auto& state_estimator = flight_manager->getLander().getStateEstimator();
    
    if(!state_estimator.isFlightConditionValid()){
        Serial.println("StateMachine: Flight conditions not valid, cannot take off");
        return false;
    }

    if(!state_estimator.isHomePositionSet()){
        Serial.println("StateMachine: Home position not set, cannot take off");
        return false;
    }
    Serial.println("StateMachine: Flight conditions valid, can take off");

    return true;




    // auto& mission = flight_manager->getMission();
    // if(!mission.isMissionValid()){
    //     return false;
    // }

    

    //mission.start();








}