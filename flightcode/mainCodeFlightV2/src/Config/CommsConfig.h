#pragma once

#include <Arduino.h>
#include <array>

#include "../Utilities/Utilities.h"

enum class SerialOutputType {
    HUMAN_READABLE, 
    BASE_STATION_FORMAT, // $LNDAS,val1,val2,...,valn,*\n
};

struct SerialPortConfig {

    HardwareSerial* port;

    uint32_t baudrate;

    const char* name;   
    bool enabled = true; // default to true, can be set to false if the port is not needed
     bool input_enabled = true; // default to true, can be set to false if input is not needed
    bool output_enabled = true; // default to true, can be set to false if output is not needed
    bool periodic_output_enabled = true; // default to true, can be set to false if periodic output is not needed

    float frequency;

    SerialOutputType outputType;

    int bufferSize = 100; // default buffer size for incoming data
    
};


namespace CommsConfig {

    

    constexpr SerialPortConfig RADIO = {
        .port = &Serial5,
        .baudrate = 115200,
        .name = "MAIN COMMS",
        .input_enabled = true,
        .output_enabled = true,
        .periodic_output_enabled = true,
        .frequency = 10.0f, // 10 Hz
        .outputType = SerialOutputType::BASE_STATION_FORMAT,
        .bufferSize = 30 // buffer size for incoming data
    };


    constexpr SerialPortConfig USB = {
        .port = &Serial2, /// unused, but still initialized to avoid conflicts
        .baudrate = 115200,
        .name = "DEBUG COMMS",
        .input_enabled = true, // enable input for USB
        .output_enabled = true,
        .periodic_output_enabled = true,
        .frequency = 2.0f, // 2 Hz
        .outputType = SerialOutputType::HUMAN_READABLE,
        .bufferSize = 30 // buffer size for incoming data
       
    };


    constexpr const char* outputHeaderTelemetryFormat =
        "$LNDAS";

    constexpr const char* outputLineEndingTelemetryFormat =
        "*";


    constexpr std::array<uint32_t, 9> baudratesAllowed = {
        9600,
        19200,
        38400,
        57600,
        115200,
        230400,
        250000,
        500000,
        1000000
    };

}


constexpr ParameterConfig<float> frequencyConfig = {
    .min = 0.1f,
    .max = 100.0f,
    .default_ = 10.0f
};

constexpr bool isBaudrateAllowed(uint32_t baudrate)
{
    for (uint32_t rate : CommsConfig::baudratesAllowed) {

        if (rate == baudrate) {
            return true;
        }
    }

    return false;
}



static_assert(isBaudrateAllowed(CommsConfig::RADIO.baudrate),"RADIO baudrate is not allowed.");


static_assert(isBaudrateAllowed(CommsConfig::USB.baudrate),"USB baudrate is not allowed.");



static_assert(
    Utilities::isBounded(
        CommsConfig::RADIO.frequency,frequencyConfig.min,
        frequencyConfig.max
    ),
    "RADIO frequency is out of range."
);

static_assert(
    Utilities::isBounded(
        CommsConfig::USB.frequency,
        frequencyConfig.min,
        frequencyConfig.max
    ),
    "USB frequency is out of range."
);