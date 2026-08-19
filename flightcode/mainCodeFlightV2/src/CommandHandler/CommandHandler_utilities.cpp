
#include "Arduino.h"
#include "../Utilities/Utilities.h"
#include "CommandHandler.h"
#include "../FlightManager/FlightManager.h"

    void CommandHandler::parseCommand(String header, String payload)
{
    Serial.println("Command received: " + header + " with payload: " + payload + "|");
    if(header == "SET_HOME"){
        
        setOKFeedback(header);
        Serial.println("Set home command received");
        auto& state_estimator = flight_manager->getLander().getStateEstimator();
        state_estimator.request_setHomePosition();
        return;
    }

    if(header == "NEXT_WP"){
        setOKFeedback(header);
        Serial.println("Next waypoint command received");
        flight_manager->getMission().advanceWaypoint();
        return;
    }


    if (header == "LINE"){
        setOKFeedback(header);
        flight_manager->getTelemetryManager().send_USB_debug_payload();
        
        return;
    }

    
    if (header == "TGL_USB_PER") // Toggle RADIO output periodic
    {
        setOKFeedback(header);
        Serial.println("Toggle USB output periodic command received");
        flight_manager->getTelemetryManager().toggle_USB_periodic();
        return;
    }



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
        flight_manager->getStateMachine().requestStateChange(STATE_MACHINE_STATES::LAUNCH);
        Serial.println("Launch command received");
        
        flight_manager->getTelemetryManager().send_USB_debug_payload();
        return;
    }


    if(header == "ABORT_LAUNCH")
    {
        setOKFeedback(header);
        auto& mission = flight_manager->getMission();
        
        mission.forceLand(); // force the mission to go to the landing waypoint, regardless of the current state
        Serial.println("Abort launch command received");
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