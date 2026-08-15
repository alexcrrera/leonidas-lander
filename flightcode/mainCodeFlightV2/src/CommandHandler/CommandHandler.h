#pragma once


#include <Arduino.h>

#include "../Utilities/Utilities.h"

#include <TextParser.h>

class FlightManager;





class CommandHandler {
public:
    CommandHandler() = default;
    void begin(FlightManager* flight_manager_);

    void consumeIncomingCommand(String incomingText);

    void setOKFeedback(const String& message) {command_feedback = "[OK] - " + message;}
    void setErrorFeedback(const String& error) {command_feedback = "[ERROR]: " + error;}
    void setFeedback(const String& feedback) {command_feedback = feedback;}

    String command_feedback = "[CLEAR]"; // default feedback message, can be updated after processing a command
private:
    
   
    void parseCommand(String header, String payload);
    FlightManager* flight_manager = nullptr;


   
   // TextParser textParser;
};

