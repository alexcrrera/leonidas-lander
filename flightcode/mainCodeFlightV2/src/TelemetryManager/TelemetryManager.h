#ifndef TELEEMTRYMANAGER_H
#define TELEEMTRYMANAGER_H

#include <Arduino.h>

#include "../Utilities/Utilities.h"
#include "CommsConfig.h"

class TelemetryManager
{
public:
    TelemetryManager(); //modif

    void readInput();
    void sendCommand(const String& content); // send one line
    void begin();

private:
    SerialPortConfig serial_port = CommsConfig::TELEMETRY_2_BASE_STATION;
    String header = CommsConfig::Output_Header;
    String line_ending = CommsConfig::Output_LineEnding;
};

#endif
