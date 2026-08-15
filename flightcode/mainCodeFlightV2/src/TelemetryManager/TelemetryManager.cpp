#include "TelemetryManager.h"

#include "../FlightManager/FlightManager.h"


TelemetryManager::TelemetryManager(
    FlightManager* flightManager
)
    : flightManager(flightManager)
{
}


// ============================================================
// BEGIN
// ============================================================

void TelemetryManager::begin()
{

    send_USB_String("INIT START OF TELEM",true);

    if (CommsConfig::USB.enabled)
    {
        
        USB_output_initialized = true;
      

        Serial.begin(CommsConfig::USB.baudrate); // Initialize USB serial port

        send_USB_String("INIT SERIAL OK",true);

        //USB_output_handler.begin(CommsConfig::USB.frequency, this, &TelemetryManager::send_USB_String);
        
    }


    if (CommsConfig::RADIO.enabled)
    {
        
       
        CommsConfig::RADIO.port->begin(CommsConfig::RADIO.baudrate);
        RADIO_output_initialized = true;
        //RADIO_output_handler.begin(CommsConfig::RADIO.frequency, this, &TelemetryManager::send_RADIO_String);   

        send_RADIO_String(get_telemetry_payload(),false);
    }
}




void TelemetryManager::update(uint32_t now)
{
    handleInput();

    // --------------------------------------------------------
    // USB
    // --------------------------------------------------------
    if (USB_output_initialized && USB_periodic_output_enabled)
    {
    if (now - USB_last_output_time >=static_cast<uint32_t>(1000.0f / CommsConfig::USB.frequency)){
        
        USB_last_output_time = now;


        if (USB_output_enabled)
        {
            if (USB_output_type ==SerialOutputType::HUMAN_READABLE)
            {
                send_USB_String(get_debug_payload(),false);
            }
            else
            {
                send_USB_String(get_telemetry_payload(),false);
            }
        }
    }
}


    // --------------------------------------------------------
    // RADIO
    // --------------------------------------------------------

    if (RADIO_periodic_output_enabled && (now - RADIO_last_output_time >= static_cast<uint32_t>(1000.0f / CommsConfig::RADIO.frequency)))
    {
        RADIO_last_output_time = now;


        if (RADIO_output_enabled)
        {
           
            if (RADIO_output_type ==SerialOutputType::HUMAN_READABLE){
                
                send_RADIO_String(get_debug_payload(),false);
            }
            else
            {
                send_RADIO_String(get_telemetry_payload(),false);
            }
        }
    }
}



// ============================================================
// TELEMETRY PAYLOAD
// ============================================================


// ============================================================
// DEBUG PAYLOAD
// ============================================================












// ============================================================
// UPDATE
// ============================================================
void TelemetryManager::handleInput()
{
    if (USB_input_enabled)
    {
        handle_USB_input();
    }

    if (RADIO_input_enabled)
    {
        handle_RADIO_input();
    }
}


void TelemetryManager::handle_USB_input()
{
    while (Serial.available() > 0)
    {
        char incomingChar = Serial.read();

        if (incomingChar == '\n' || incomingChar == '\r')
        {
            if (dataIndexUSB > 0)
            {
                incomingDataUSB[dataIndexUSB] = '\0'; // Null-terminate the string
                String incomingText = String(incomingDataUSB);
                flightManager->getCommandHandler().consumeIncomingCommand(incomingText);
                dataIndexUSB = 0; // Reset index for next command
            }
        }
        else
        {
            if (dataIndexUSB < bufferSizeUSB - 1)
            {
                incomingDataUSB[dataIndexUSB++] = incomingChar;
            }
            if (dataIndexUSB >= bufferSizeUSB - 1)
            {
                // Buffer overflow, reset index
                dataIndexUSB = 0;
            }
        }
    }
}


void TelemetryManager::handle_RADIO_input()
{
    while (RADIO_port->available() > 0)
    {
        char incomingChar = RADIO_port->read();

        if (incomingChar == '\n' || incomingChar == '\r')
        {
            if (dataIndexRADIO > 0)
            {
                incomingDataRADIO[dataIndexRADIO] = '\0'; // Null-terminate the string
                String incomingText = String(incomingDataRADIO);
                flightManager->getCommandHandler().consumeIncomingCommand(incomingText);
                dataIndexRADIO = 0; // Reset index for next command
            }
        }
        else
        {
            if (dataIndexRADIO < bufferSizeRADIO - 1)
            {
                incomingDataRADIO[dataIndexRADIO++] = incomingChar;
            }
            if (dataIndexRADIO >= bufferSizeRADIO - 1)
            {
                // Buffer overflow, reset index
                dataIndexRADIO = 0;
            }
        }
    }
}






















// ============================================================
// RADIO OUTPUT
// ============================================================

void TelemetryManager::send_RADIO_String(
    const String& content,
    bool jumpToNextLine
)
{
    
    if(!RADIO_output_enabled || !RADIO_output_initialized){
        return;
    }


    RADIO_port->print(content);


    if (jumpToNextLine)
    {
        RADIO_port->print("\n");
    }
}


// ============================================================
// USB OUTPUT
// ============================================================

void TelemetryManager::send_USB_String(
    const String& content,
    bool jumpToNextLine
)
{

    if(!USB_output_enabled || !USB_output_initialized){
        return;
    }

    USB_port->print(content);


    if (jumpToNextLine)
    {
        USB_port->print("\n");
    }
}






// ============================================================
// OUTPUT CONTROL
// ============================================================

void TelemetryManager::toggle_RADIO(bool enable)
{
    if (!RADIO_output_initialized)
    {
        RADIO_output_enabled = false;
        return;
    }

    RADIO_output_enabled = enable;
}


void TelemetryManager::toggle_USB(bool enable)
{
    if (!USB_output_initialized)
    {
        USB_output_enabled = false;
        return;
    }

    USB_output_enabled = enable;
}





