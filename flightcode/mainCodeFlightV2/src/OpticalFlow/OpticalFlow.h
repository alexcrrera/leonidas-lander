#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "../Sensor/Sensor.h"


struct OpticalFlowMeasurement {

    int16_t deltaX = 0;
    int16_t deltaY = 0;

    uint8_t surfaceQuality = 0;

    float flowRateX = 0.0f;
    float flowRateY = 0.0f;

    float dt = 0.0f;
};


class OpticalFlow : public Sensor<OpticalFlowMeasurement> {

public:

    explicit OpticalFlow(
        HardwareSerial& serialPort,
        uint32_t baudRate = 19200,
        unsigned long sensorTimeoutMs = 500
    );

    bool begin() override;
    void update() override;
    void zero() override;

    OpticalFlowMeasurement getMeasurement() const override;


protected:

    void validateMeasurement(
        const OpticalFlowMeasurement& rawMeasurement
    ) override;


private:

    static constexpr uint8_t packetHeader = 0xFE;
    static constexpr uint8_t packetLength = 9;
    static constexpr uint8_t packetFooter = 0xAA;
    static constexpr uint8_t expectedDataLength = 0x04;

    static constexpr float pixelScaling = 1.76e-3f;

    HardwareSerial& serialPort;
    uint32_t baudRate;

    uint8_t packetBuffer[packetLength] = {};
    uint8_t packetIndex = 0;

    OpticalFlowMeasurement filteredMeasurement{};

    unsigned long previousMeasurementTime = 0;

    bool readPacket(
        OpticalFlowMeasurement& measurement
    );

    bool decodePacket(
        const uint8_t* packet,
        OpticalFlowMeasurement& measurement
    );

    void resetParser();

    float calculateDeltaTime(
        unsigned long currentTime
    ) const;
};