#pragma once

#include "Utilities.h"
#include "Arduino.h"


 namespace CommsConfig {

    constexpr SerialPortConfig TELEMETRY_2_BASE_STATION = {
            .port = &Serial8,    
            .baudrate = 115200,
            .name = "MAIN COMMS"
    };


    constexpr String Output_Header = "$LNDAS";
    constexpr String Output_LineEnding = "*\n";


 }