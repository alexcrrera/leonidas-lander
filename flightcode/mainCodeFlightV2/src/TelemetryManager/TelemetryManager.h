#pragma once

#include <Arduino.h>
#include <textparser.h>

#include "../Utilities/Utilities.h"
#include "../Config/CommsConfig.h"


class TelemetryManager {

public:

    TelemetryManager();

    void begin();

    void readInput();

    void sendCommand(const String& content);


private:

    SerialPortConfig serialPort = CommsConfig::TELEMETRY_2_BASE_STATION;

    String header = CommsConfig::Output_Header;
    String lineEnding = CommsConfig::Output_LineEnding;

    TextParser commaParser{","};
};