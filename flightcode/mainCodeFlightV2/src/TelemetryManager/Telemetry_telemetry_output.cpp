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

    const LanderSolution& solution = flightManager->getLander().getSolution();

    
    payload += CommsConfig::outputHeaderTelemetryFormat;

    // -------------------2-4--------------------------
    //addTelemetryGroup(payload, solution.state.position);
    // random data for testing
    TelemetryUtilities::addTelemetryField(payload, 69);
    TelemetryUtilities::addTelemetryField(payload, 420);
    TelemetryUtilities::addTelemetryField(payload, 1337);

    // -------------------5-7--------------------------
    TelemetryUtilities::addTelemetryGroup(payload, solution.state.velocity);

    // -------------------8-10--------------------------
    TelemetryUtilities::addTelemetryGroup(payload, solution.state.attitude);

    // -------------------11-13 ------- GPS DATA -----LATITUDE LONGITUDE ALTITUDE --------------
   // addTelemetryField(payload, solution.state.latitude, 6);
    TelemetryUtilities::addTelemetryField(payload, 48.8466, 6);
    TelemetryUtilities::addTelemetryField(payload, 2.35455, 6);
    ///addTelemetryField(payload, solution.state.longitude, 6);
    TelemetryUtilities::addTelemetryField(payload, 11, 2);

    // -------------------Sensor states - 14 - 17--------------------------
    TelemetryUtilities::addTelemetryField(payload, solution.validity.positionValid);
    TelemetryUtilities::addTelemetryField(payload, solution.validity.velocityValid);
    TelemetryUtilities::addTelemetryField(payload, solution.validity.accelerationValid);
    TelemetryUtilities::addTelemetryField(payload, solution.validity.altitudeValid);





    payload += CommsConfig::outputLineEndingTelemetryFormat;
    payload += "\n";

    return payload;


return("NOTIMPLEMENTED");
}
