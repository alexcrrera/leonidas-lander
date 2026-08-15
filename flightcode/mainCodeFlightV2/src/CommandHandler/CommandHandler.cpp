#include "CommandHandler.h"


void CommandHandler::begin(FlightManager* flight_manager_){
    flight_manager = flight_manager_;
}



void CommandHandler::parseCommand(String header, String payload) {
    // Parse the incoming text
    if (header == "ARM") {
        Serial.println("Arm command received");
       // return COMMAND::ARM;
    } else if (header    == "DISARM") {
        Serial.println("Disarm command received");
       // return COMMAND::DISARM;
    } else if (header == "LAUNCH") {
        Serial.println("Launch command received");
       // return COMMAND::LAUNCH;
    } else if (header == "ABORT") {
      Serial.println("Abort command received");
       // return COMMAND::ABORT;
    } else if (header == "LAND") {
       Serial.println("Land command received");
       // return COMMAND::LAND;
    } else if (header == "RESET") {
       Serial.println("Reset command received");
    } else {
        // Handle unknown command
        Serial.println("Unknown command received: " + header);
        return;
    }
} 




void CommandHandler::consumeIncomingCommand(String incomingText) {
   // format "IDENTIFIER: payload1,payload2...\n"
    // Split the incoming text into identifier and payload
    int separatorIndex = incomingText.indexOf(':');
    if (separatorIndex == -1) {
        // Handle error: invalid format
        Serial.println("Invalid command format: " + incomingText);
            return;
    }

    String identifier = incomingText.substring(0, separatorIndex);
    String payload = incomingText.substring(separatorIndex + 1);

    parseCommand(identifier, payload);

    
}