#include "TelemetryManager.h"




void TelemetryManager::begin(){
    serial_port.port->begin(serial_port.baudrate);
    sendCommand("TELEM INITIALISED");    
}



void TelemetryManager::sendCommand(const String& content){
    serial_port.port->print(content);
    serial_port.port->print("\n");

    
}