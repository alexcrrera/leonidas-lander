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

    telemetry_manager.begin();

   
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

    


    // --------------------------------------------------------
    // Update telemetry
    // --------------------------------------------------------

    telemetry_manager.update(now);


}


