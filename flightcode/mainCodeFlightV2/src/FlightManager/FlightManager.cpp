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
   

    lander.begin();
    command_handler.begin(this);
    telemetry_manager.begin();
    mission.begin(this);
    state_machine.begin(this);
    state_machine.setState(STATE_MACHINE_STATES::BOOTED);
   
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

    //controller.update(lander.getState(), mission.getCurrentRegimeData());
    //mission.update(lander.getState());
    motor_manager.update(controller.getControlCmd());
    
    telemetry_manager.update(now);


}


