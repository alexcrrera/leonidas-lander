#include "TelemetryManager.h"


TelemetryManager::TelemetryManager(FlightManager* flightManager) : flightManager(flightManager) {

}




void TelemetryManager::begin() {

    //configure USB
    if(CommsConfig::USB.enabled) {

        USB_output_type = debugOutputType;
        USB_output_initialized = true;
        USB_output_active = true;

        CommsConfig::USB.port->begin(CommsConfig::USB.baudrate);
        send_USB_String("INIT SERIAL OK", debugOutputType, true);
    }

    if(CommsConfig::RADIO.enabled) {
        
        RADIO_output_type = telemetryOutputType;
        RADIO_output_initialized = true;
        RADIO_output_active = true;

        CommsConfig::RADIO.port->begin(CommsConfig::RADIO.baudrate);
        send_RADIO_String("INIT SERIAL OK", telemetryOutputType, true);
    }




}


void TelemetryManager::update(uint32_t now) {
    // This function can be used to handle periodic tasks related to telemetry management.
    // For example, it could check for incoming data, manage timeouts, or handle other telemetry-related tasks.

    handleInput();
    
    if (now - USB_last_output_time >= (1000.0f / CommsConfig::USB.frequency)) {
        USB_last_output_time = now;
        if (isUSB_OutputEnabled()) {
            if(USB_output_type == OutputType::HUMAN_READABLE) {
                send_USB_String(get_debug_payload(), true);
            } else{
                send_USB_String(get_telemetry_payload(), true);
            }
        }
    }

    if (now - RADIO_last_output_time >= (1000.0f / CommsConfig::RADIO.frequency)) {
        RADIO_last_output_time = now;
        if (isRADIO_OutputEnabled()) {
            if(RADIO_output_type == OutputType::HUMAN_READABLE) {
                send_RADIO_String(get_debug_payload(), true);
            } else{
                send_RADIO_String(get_telemetry_payload(), true);
            }
        }
    }


}


String TelemetryManager::get_telemetry_payload() {
    // This function should generate and return the telemetry payload string based on the current data and format.
    // For now, it returns a placeholder string. You can modify this to include actual telemetry information.
    
    String payload;

    payload +"test,test2,test3";

    return payload;
}

String TelemetryManager::get_debug_payload() {
    // This function should generate and return the debug payload string based on the current data and format.
    // For now, it returns a placeholder string. You can modify this to include actual debug information.
    
    String payload;

    addDebugTitle(payload, "FLIGHT STATUS");

   

    addDebugGroup(payload, "LANDER");

    addDebugVector3(
        payload,
        "Position NED",
        flightManager.lander.estimatePosition()
    );

    addDebugVector3(
        payload,
        "Velocity NED",
        flightManager.lander.estimateVelocity());



    return payload;

}





void TelemetryManager::addDebugTitle(    String& payload,const String& title){
    payload += "\n===== ";
    payload += title;
    payload += " =====\n";
}


void TelemetryManager::addDebugGroup(
    String& payload,
    const String& name
)
{
    payload += "\n--- ";
    payload += name;
    payload += " ---\n";
}


void TelemetryManager::addDebugField(
    String& payload,
    const String& name,
    const String& value
)
{
    payload += name;
    payload += ": ";
    payload += value;
    payload += "\n";
}


void TelemetryManager::addDebugField(
    String& payload,
    const String& name,
    float value
)
{
    payload += name;
    payload += ": ";
    payload += String(value, 3);
    payload += "\n";
}


void TelemetryManager::addDebugField(
    String& payload,
    const String& name,
    bool value
)
{
    payload += name;
    payload += ": ";
    payload += value ? "YES" : "NO";
    payload += "\n";
}


void TelemetryManager::send_RADIO_String(const String& content, OutputType outputType, bool jumpToNextLine) {
    if (!isRADIO_OutputEnabled()) {
        return; // do not send if not enabled
    }
    CommsConfig::RADIO.port->print(getFormattedOutput(outputType, content,jumpToNextLine));
}

void TelemetryManager::send_USB_String(const String& content, bool jumpToNextLine) {
    if (!isUSB_OutputEnabled()) {
        return; // do not send if not enabled
    }

    CommsConfig::USB.port->print(getFormattedOutput(USB_output_type, content,jumpToNextLine));
}

void TelemetryManager::toggle_RADIO(bool enable) {
    if(!RADIO_output_initialized){
        RADIO_output_active = false;
        return; // cannot enable/disable if not initialized
    }
    else {
        RADIO_output_active = enable;
    }
}

void TelemetryManager::toggle_USB(bool enable) {
    if(!USB_output_initialized){
        USB_output_active = false;
        return; // cannot enable/disable if not initialized
    }
    else {
        USB_output_active = enable;
    }
}


String TelemetryManager::getFormattedOutput(OutputType outputType,const String& content, bool jumpToNextLine) const
{
    if (outputType == OutputType::HUMAN_READABLE) {
        return content;
    }
    String output = header_telemetry_format + content + lineEnding_telemetry_format;
    
    if (jumpToNextLine) {
        output += lineEnding_telemetry_format;
    }

    return output;
}

bool TelemetryManager::isUSB_OutputEnabled() {
    return USB_output_active && USB_output_enabled && USB_output_initialized;
}

bool TelemetryManager::isRADIO_OutputEnabled() {
    return RADIO_output_active && RADIO_output_enabled && RADIO_output_initialized;
}