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

    // take off point
  
    


// take off point
TelemetryUtilities::addDebugGroup(payload, "TAKE OFF POINT");

const auto& mission = flightManager->getMission();

TelemetryUtilities::addDebugGroup(payload, mission.takeOffWaypoint.getTarget().positionNED, "Take Off Point NED");
TelemetryUtilities::addDebugField(payload, "Take Off Point Yaw", mission.takeOffWaypoint.getTarget().yaw_deg);


// transition point for landing
TelemetryUtilities::addDebugGroup(payload, "LANDING TRANSITION POINT");

TelemetryUtilities::addDebugGroup(payload, mission.landingTransitionWaypoint.getTarget().positionNED, "Landing Transition Point NED");
TelemetryUtilities::addDebugField(payload, "Landing Transition Point Yaw", mission.landingTransitionWaypoint.getTarget().yaw_deg);


// landing point
TelemetryUtilities::addDebugGroup(payload, "LANDING POINT");

TelemetryUtilities::addDebugGroup(payload, mission.landingWaypoint.getTarget().positionNED, "Landing Point NED");
TelemetryUtilities::addDebugField(payload, "Landing Point Yaw", mission.landingWaypoint.getTarget().yaw_deg);


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





