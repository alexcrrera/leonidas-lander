#include "TelemetryManager.h"
#include"Telemetry_utilities.h"
#include "Arduino.h"
#include "../Lander/Lander_structs.h"
#include "../FlightManager/FlightManager.h"
#include "../Lander/Lander.h"
String TelemetryManager::get_debug_payload()
{
    String payload;

    if (flightManager == nullptr)
        return "FlightManager: NULL\n";

    Lander& lander = flightManager->getLander();
   // const LanderState& state  = lander.getState();

    TelemetryUtilities::addDebugTitle(payload, "FLIGHT STATUS");

    // flightmanager debug text
    // control command ouput:
  
    TelemetryUtilities::addDebugGroup(payload, "FLIGHT MANAGER DEBUG");
    payload += flightManager->debug_text;

    // flight regime data
    // getFlightRegimeDataAsString
    auto current_regime = flightManager->getStateMachine().getCurrentFlightRegimeData();
    TelemetryUtilities::addDebugGroup(payload, "FLIGHT REGIME");
    payload += TelemetryUtilities::getFlightRegimeDataAsString(current_regime);
    

    // Mission debug text
    TelemetryUtilities::addDebugGroup(payload, "MISSION DEBUG");
    payload += flightManager->getMission().getMissionDataAsString();

    // target data:
    TelemetryUtilities::addDebugGroup(payload, "MISSION TARGET");
    auto target = flightManager->getMission().getTarget();
    TelemetryUtilities::addDebugGroup(payload, target.target.positionNED, "POSITION NED");
    TelemetryUtilities::addDebugField(payload,"Yaw", String(target.target.yaw_deg));
    
    auto state = lander.getState();
    TelemetryUtilities::addDebugGroup(payload, state.position, "POSITION NED");
        
    TelemetryUtilities::addDebugGroup(payload, "CMD");
      auto cmd = flightManager->getController().getControlCmd();
        TelemetryUtilities::addDebugGroup(payload,cmd );


    // TelemetryUtilities::addDebugGroup(payload, "STATUS");

    // TelemetryUtilities::addDebugField(payload, "State", flightManager->getStateMachine().getStateAsString());

    // TelemetryUtilities::addDebugGroup(payload, "LANDER");

    

    // TelemetryUtilities::addDebugGroup(payload, state.velocity, "VELOCITY NED");

    // TelemetryUtilities::addDebugGroup(payload, state.acceleration, "ACCELERATION NED");

    // TelemetryUtilities::addDebugGroup(payload, state.attitude, "ATTITUDE");

    payload += "\n";

    return payload;
    
}





