#pragma once


#include <Arduino.h>

#include "../Utilities/Utilities.h"
#include "CommandHandler_utilities.h"
#include <TextParser.h>

class FlightManager;





class CommandHandler {
public:
    CommandHandler() = default;
    void begin(FlightManager* flight_manager_);

    void consumeIncomingCommand(String incomingText);


    


private:
    
   
    void parseCommand(String header, String payload);
    FlightManager* flight_manager = nullptr;


   
   // TextParser textParser;
};

