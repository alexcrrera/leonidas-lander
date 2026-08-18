#include "TelemetryManager.h"
#include"Telemetry_utilities.h"
#include "Arduino.h"
#include "../Lander/Lander_structs.h"
#include "../Lander/Lander.h"
#include "../FlightManager/FlightManager.h"

String TelemetryManager::get_telemetry_payload()
{
    String payload;

    if (flightManager == nullptr)
        return "";

    const LanderState& state = flightManager->getLander().getState();

    
    payload += CommsConfig::outputHeaderTelemetryFormat;

    // -------------------2-4--------------------------
    //addTelemetryGroup(payload, state.position);
    // random data for testing
   TelemetryUtilities::addTelemetryGroup(payload, state.position);

    // ------------------- 5-7 --------------------------
       
    TelemetryUtilities::addTelemetryGroup(payload, state.velocity);


    // -------------------8-10--------------------------
    TelemetryUtilities::addTelemetryGroup(payload, state.attitude);

    // -------------------11-13 ------- GPS DATA -----LATITUDE LONGITUDE ALTITUDE --------------
   // addTelemetryField(payload, solution.state.latitude, 6);
    TelemetryUtilities::addTelemetryField(payload, 48.8466, 6);
    TelemetryUtilities::addTelemetryField(payload, 2.35455, 6);
    ///addTelemetryField(payload, solution.state.longitude, 6);
    TelemetryUtilities::addTelemetryField(payload, state.altitude_M, 2);

    // -------------------Sensor states - 14 - 17--------------------------
    TelemetryUtilities::addTelemetryField(payload, state.validity.positionValid);
    TelemetryUtilities::addTelemetryField(payload, state.validity.velocityValid);
    TelemetryUtilities::addTelemetryField(payload, state.validity.accelerationValid);
    TelemetryUtilities::addTelemetryField(payload, state.validity.altitudeValid);


    //  Command feedback - 18
    TelemetryUtilities::addTelemetryField(payload, flightManager->getCommandHandler().command_feedback);

    // thrust - 19
    TelemetryUtilities::addTelemetryField(payload, flightManager->getMotorManager().getESC_thrust_percentage(), 1);
    // 20 - motor status
    TelemetryUtilities::addTelemetryField(payload, flightManager->getFlightGuard().motorEnabledStatus());
    // 21 - motor safety status
    TelemetryUtilities::addTelemetryField(payload, flightManager->getFlightGuard().motorSafetyStatus());
    // overspeed status - 22
    TelemetryUtilities::addTelemetryField(payload, flightManager->getFlightGuard().overrideFlags.vertical_overspeed_ok);
    
    // horizontal overspeed status - 23
    TelemetryUtilities::addTelemetryField(payload, flightManager->getFlightGuard().overrideFlags.horizontal_overspeed_ok);
    // roll/pitch abort status - 24
    TelemetryUtilities::addTelemetryField(payload, flightManager->getFlightGuard().overrideFlags.roll_pitch_ok);
    // yaw abort status - 25
    TelemetryUtilities::addTelemetryField(payload, flightManager->getFlightGuard().overrideFlags.yaw_ok);



    payload += CommsConfig::outputLineEndingTelemetryFormat;
    payload += "\n";

    return payload;


return("NOTIMPLEMENTED");
}
