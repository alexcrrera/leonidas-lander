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

    bool enabled;

    float frequency;

    SerialOutputType outputType;
};


namespace CommsConfig {

    constexpr SerialPortConfig RADIO = {
        .port = &Serial1,
        .baudrate = 115200,
        .name = "MAIN COMMS",
        .enabled = true,
        .frequency = 10.0f, // 10 Hz
        .outputType = SerialOutputType::HUMAN_READABLE
    };


    constexpr SerialPortConfig USB = {
        .port = &Serial1,
        .baudrate = 115200,
        .name = "DEBUG COMMS",
        .enabled = false,
        .frequency = 10.0f, // 10 Hz
        .outputType = SerialOutputType::HUMAN_READABLE
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


constexpr bool isPortConflicting(
    const SerialPortConfig& port1,
    const SerialPortConfig& port2
)
{
    return (
        port1.port == port2.port &&
        port1.enabled &&
        port2.enabled
    );
}


static_assert(
    isBaudrateAllowed(CommsConfig::RADIO.baudrate),
    "RADIO baudrate is not allowed."
);


static_assert(
    isBaudrateAllowed(CommsConfig::USB.baudrate),
    "USB baudrate is not allowed."
);


static_assert(
    !isPortConflicting(
        CommsConfig::RADIO,
        CommsConfig::USB
    ),
    "RADIO and USB ports cannot be the same."
);

static_assert(
    Utilities::isBounded(
        CommsConfig::RADIO.frequency,
        frequencyConfig.min,
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