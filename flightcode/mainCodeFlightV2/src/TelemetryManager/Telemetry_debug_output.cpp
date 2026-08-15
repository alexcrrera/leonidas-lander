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

    TelemetryUtilities::addDebugGroup(payload, "STATUS");

    TelemetryUtilities::addDebugField(payload, "State", flightManager->getStateMachine().getStateAsString());

    TelemetryUtilities::addDebugGroup(payload, "LANDER");

    TelemetryUtilities::addDebugGroup(payload, state.position, "POSITION NED");

    TelemetryUtilities::addDebugGroup(payload, state.velocity, "VELOCITY NED");

    TelemetryUtilities::addDebugGroup(payload, state.acceleration, "ACCELERATION NED");

    TelemetryUtilities::addDebugGroup(payload, state.attitude, "ATTITUDE");

    payload += "\n";

    return payload;
    
}





