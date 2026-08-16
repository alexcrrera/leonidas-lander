
#include "Arduino.h"
#include "../Utilities/Utilities.h"
#include "CommandHandler.h"
#include "../FlightManager/FlightManager.h"

    void CommandHandler::parseCommand(String header, String payload)
{
    if (header == "ARM")
    {
        setOKFeedback(header);
        Serial.println("Arm command received");
        return;
    }

    if (header == "TGL_RAD_PER") // Toggle RADIO output periodic
    {
        setOKFeedback(header);
        Serial.println("Toggle RADIO output periodic command received");
        flight_manager->getTelemetryManager().toggle_RADIO_periodic();
        return;
    }

    if (header == "DISARM")
    {
        setOKFeedback(header);
        Serial.println("Disarm command received");
        return;
    }

    if (header == "LAUNCH")
    {
        setOKFeedback(header);
        Serial.println("Launch command received");
        return;
    }

    if (header == "ABORT")
    {
        setOKFeedback(header);
        Serial.println("Abort command received");
        return;
    }

    if (header == "LAND")
    {
        setOKFeedback(header);
        Serial.println("Land command received");
        return;
    }

    if (header == "RESET")
    {
        setOKFeedback(header);
        Serial.println("Reset command received");
        return;
    }

    setFeedback("UNK",header);
    Serial.println("Unknown command received: " + header);
}