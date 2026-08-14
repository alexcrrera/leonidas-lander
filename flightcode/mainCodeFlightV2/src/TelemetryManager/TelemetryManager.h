#pragma once

#include <Arduino.h>
#include <textparser.h>

#include "../Utilities/Utilities.h"
#include "../Config/CommsConfig.h"
#include "Telemetry_utilities.h"


#define TELEMETRY_MAX_CMD_HISTORY 15


class FlightManager;


// placeholder for the command history
enum class COMMANDS {
    NONE,
    ARM,
    DISARM,
    TAKEOFF,
    LAND,
    ABORT
};

class TelemetryManager {

public:

    TelemetryManager(FlightManager* flightManager);

    void begin(SerialOutputType debugOutputType = SerialOutputType::HUMAN_READABLE,SerialOutputType telemetryOutputType = SerialOutputType::BASE_STATION_FORMAT);

    void update(uint32_t now);

    void handleInput();


    void toggle_USB(bool enable);
    void toggle_RADIO(bool enable);


    void send_USB_String(const String& content,bool jumpToNextLine = false);
    void send_RADIO_String(const String& content,bool jumpToNextLine = false    );


    String get_debug_payload();

    String get_telemetry_payload();





private:

    FlightManager* flightManager;


    // --------------------------------------------------------
    // Output timing
    // --------------------------------------------------------

    uint32_t USB_last_output_time = 0;
    uint32_t RADIO_last_output_time = 0;


    // --------------------------------------------------------
    // Output configuration
    // --------------------------------------------------------

    SerialOutputType USB_output_type = SerialOutputType::HUMAN_READABLE;

    SerialOutputType RADIO_output_type = SerialOutputType::BASE_STATION_FORMAT;


    bool USB_output_active = true;
    bool RADIO_output_active = true;

    bool USB_output_initialized = false;
    bool RADIO_output_initialized = false;


    // --------------------------------------------------------
    // Command history
    // --------------------------------------------------------

    COMMANDS commandHistory[TELEMETRY_MAX_CMD_HISTORY];

    int commandHistoryIndex = 0;


    // --------------------------------------------------------
    // Telemetry
    // --------------------------------------------------------

    String telemetry_payload;


    String header_telemetry_format = CommsConfig::outputHeaderTelemetryFormat;

    String lineEnding_telemetry_format = CommsConfig::outputLineEndingTelemetryFormat;


    TextParser commaParser{","};


  
    // --------------------------------------------------------
    // Debug formatting helpers
    // --------------------------------------------------------
  


    


   bool isUSB_OutputEnabled();
    bool isRADIO_OutputEnabled();
};

