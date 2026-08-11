#pragma once

#include <Arduino.h>
#include <textparser.h>

#include "../Utilities/Utilities.h"
#include "../Config/CommsConfig.h"


#define TELEMETRY_MAX_CMD_HISTORY 15


class FlightManager;


enum class COMMANDS {
    TOGGLE_USB_OUTPUT,
    TOGGLE_RADIO_OUTPUT
};


class TelemetryManager {

public:

    TelemetryManager(FlightManager* flightManager);

    void begin(
        SerialOutputType debugOutputType = SerialOutputType::HUMAN_READABLE,
        SerialOutputType telemetryOutputType = SerialOutputType::BASE_STATION_FORMAT
    );

    void update(uint32_t now);

    void handleInput();


    void toggle_USB(bool enable);
    void toggle_RADIO(bool enable);


    void send_USB_String(
        const String& content,
        bool jumpToNextLine = false
    );

    void send_RADIO_String(
        const String& content,
        bool jumpToNextLine = false
    );


    String get_debug_payload();

    String get_telemetry_payload();


    COMMANDS getLatestCommand();


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

    SerialOutputType USB_output_type =
        SerialOutputType::HUMAN_READABLE;

    SerialOutputType RADIO_output_type =
        SerialOutputType::BASE_STATION_FORMAT;


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


    String header_telemetry_format =
        CommsConfig::outputHeaderTelemetryFormat;

    String lineEnding_telemetry_format =
        CommsConfig::outputLineEndingTelemetryFormat;


    TextParser commaParser{","};


    // --------------------------------------------------------
    // Formatting
    // --------------------------------------------------------

   String getFormattedOutput(
    SerialOutputType outputType,
    const String& content,
    bool jumpToNextLine
) const;


    // --------------------------------------------------------
    // Debug formatting helpers
    // --------------------------------------------------------

    void addDebugTitle(
        String& payload,
        const String& title
    );

    void addDebugGroup(
        String& payload,
        const String& name
    );

    void addDebugField(
        String& payload,
        const String& name,
        const String& value
    );

    void addDebugField(
        String& payload,
        const String& name,
        float value
    );

    void addDebugField(
        String& payload,
        const String& name,
        bool value
    );


    // --------------------------------------------------------
    // Output state
    // --------------------------------------------------------

    bool isUSB_OutputEnabled();
    bool isRADIO_OutputEnabled();
};