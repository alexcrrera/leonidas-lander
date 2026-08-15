#include "CommandHandler.h"


void CommandHandler::begin(FlightManager* flight_manager_){
    flight_manager = flight_manager_;
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