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
    const LanderState& state  = lander.getState();

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
    

    // target point
    TelemetryUtilities::addDebugGroup(payload, "MISSION TARGET");
    TelemetryUtilities::addDebugGroup(payload, flightManager->getMission().getTarget().target.positionNED, "Target Point NED");
    TelemetryUtilities::addDebugField(payload, "Target Point Yaw", flightManager->getMission().getTarget().target.yaw_deg);


    // altitude Controller PID data

    TelemetryUtilities::addDebugGroup(payload, "DOWN POS PID");
    payload += flightManager->getController().PID_position.axis_z.getPID_data_as_string();

    // velocity Controller PID data
    TelemetryUtilities::addDebugGroup(payload, "DOWN VELOCITY PID");
    payload += flightManager->getController().PID_velocity.axis_z.getPID_data_as_string();


        
    TelemetryUtilities::addDebugGroup(payload, "CMD");
      auto cmd = flightManager->getController().getControlCmd();
        TelemetryUtilities::addDebugGroup(payload,cmd );


    // TelemetryUtilities::addDebugGroup(payload, "STATUS");

    // TelemetryUtilities::addDebugField(payload, "State", flightManager->getStateMachine().getStateAsString());

    // TelemetryUtilities::addDebugGroup(payload, "LANDER");

    // TelemetryUtilities::addDebugGroup(payload, state.position, "POSITION NED");

    // TelemetryUtilities::addDebugGroup(payload, state.velocity, "VELOCITY NED");

    // TelemetryUtilities::addDebugGroup(payload, state.acceleration, "ACCELERATION NED");

    // TelemetryUtilities::addDebugGroup(payload, state.attitude, "ATTITUDE");

    payload += "\n";

    return payload;
    
}





