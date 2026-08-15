#pragma once

#include <Arduino.h>
#include <textparser.h>

#include "../Utilities/Utilities.h"
#include "../Config/CommsConfig.h"
#include "Telemetry_utilities.h"

#include <Handler.h>
#define TELEMETRY_MAX_CMD_HISTORY 15


class FlightManager;




class TelemetryManager {

public:

    TelemetryManager(FlightManager* flightManager);

    void begin();

    void update(uint32_t now);

   


    void toggle_USB(bool enable);
    void toggle_RADIO(bool enable);

    void toggle_USB();
    void toggle_RADIO();

    void toggle_USB_periodic();
    void toggle_RADIO_periodic();


    void send_USB_String(const String& content,bool jumpToNextLine = false);
    void send_RADIO_String(const String& content,bool jumpToNextLine = false    );


   





private:

    Stream* USB_port = &Serial;
    HardwareSerial* RADIO_port = CommsConfig::RADIO.port;

    Handler USB_output_handler;
    Handler RADIO_output_handler;

    FlightManager* flightManager;


    int bufferSizeUSB = CommsConfig::USB.bufferSize;
    int bufferSizeRADIO = CommsConfig::RADIO.bufferSize;
    
    
    void handleInput();
    void handle_USB_input();
    void handle_RADIO_input();
     String get_debug_payload();

    String get_telemetry_payload();

    // --------------------------------------------------------
    // Output timing
    // --------------------------------------------------------

    uint32_t USB_last_output_time = 0;
    uint32_t RADIO_last_output_time = 0;


    // --------------------------------------------------------
    // Output configuration
    // --------------------------------------------------------

    SerialOutputType USB_output_type = CommsConfig::USB.outputType;
    SerialOutputType RADIO_output_type = CommsConfig::RADIO.outputType;


    // --------------------------------------------------------
    // Telemetry
    // --------------------------------------------------------

    String telemetry_payload;


    String header_telemetry_format = CommsConfig::outputHeaderTelemetryFormat;

    String lineEnding_telemetry_format = CommsConfig::outputLineEndingTelemetryFormat;


    TextParser commaParser{","};



    int dataIndexUSB = 0; //  for the incoming data buffer via USB
    int dataIndexRADIO = 0; //  for the incoming data buffer via RADIO

    char incomingDataUSB[CommsConfig::USB.bufferSize];
    char incomingDataRADIO[CommsConfig::RADIO.bufferSize];


    bool USB_output_enabled = CommsConfig::USB.output_enabled;
    bool RADIO_output_enabled = CommsConfig::RADIO.output_enabled;

    bool USB_input_enabled = CommsConfig::USB.input_enabled;
    bool RADIO_input_enabled = CommsConfig::RADIO.input_enabled;

    bool USB_periodic_output_enabled = CommsConfig::USB.periodic_output_enabled;
    bool RADIO_periodic_output_enabled = CommsConfig::RADIO.periodic_output_enabled;


    bool USB_output_initialized = false;
    bool RADIO_output_initialized = false;



  
};

