#include "FlightManager.h"


// ============================================================
// Constructor
// ============================================================

FlightManager::FlightManager()
    : 
      telemetry_manager(this)
{
}


// ============================================================
// Initialization
// ============================================================

void FlightManager::begin()
{
   

    lander.begin(this);
    command_handler.begin(this);
    telemetry_manager.begin();
    mission.begin(this);
    state_machine.begin(this);
    
    state_machine.setState(STATE_MACHINE_STATES::BOOTED);
    flight_guard.begin(this);
    motor_manager.begin(this);
    controller.begin(this);
   
}




// ============================================================
// Main update
// ============================================================

void FlightManager::update()
{
    const uint32_t now = millis();


    // --------------------------------------------------------
    // Update sensors and estimated lander state first
    // --------------------------------------------------------
    lander.update();
    state_machine.update();
    flight_guard.update();
    //controller.update(lander.getState(), mission.getCurrentRegimeData());
    //mission.update(lander.getState());
    
    mission.update(); 
    controller.update();
    motor_manager.update();
 
    
    telemetry_manager.update(now);


}


