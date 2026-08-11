#include "TelemetryManager.h"


TelemetryManager::TelemetryManager() {

}


void TelemetryManager::begin() {

    serialPort.port->begin(serialPort.baudrate);

    sendCommand("TELEM INITIALISED");
}


void TelemetryManager::readInput() {

    if (serialPort.port->available() <= 0) {
        return;
    }

    String input = serialPort.port->readStringUntil('\n');

    input.trim();

    if (input.length() == 0) {
        return;
    }

    // Input parsing will go here.
}


void TelemetryManager::sendCommand(const String& content) {

    serialPort.port->print(header);
    serialPort.port->print(content);
    serialPort.port->print(lineEnding);
}