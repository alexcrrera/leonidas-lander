#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "../Sensor/Sensor.h"


struct OpticalFlowMeasurement
{
    uint32_t systemTimeMs = 0;

    uint32_t distanceMm = 0;

    uint8_t distanceStrength = 0;
    uint8_t distancePrecision = 0;
    uint8_t distanceStatus = 0;

    int16_t flowVelocityX = 0;
    int16_t flowVelocityY = 0;

    uint8_t flowQuality = 0;
    uint8_t flowStatus = 0;

    float flowRateX = 0.0f;
    float flowRateY = 0.0f;

    float dt = 0.0f;
};

class OpticalFlow : public Sensor<OpticalFlowMeasurement>
{
public:

    OpticalFlow();

    bool begin() override;
    void update() override;
    void zero() override;

    OpticalFlowMeasurement getMeasurement() const override;


protected:

    void validateMeasurement(
        const OpticalFlowMeasurement& measurement
    ) override;


private:

   static constexpr uint8_t packetHeader = 0xEF;

static constexpr uint8_t expectedDeviceId = 0x0F;
static constexpr uint8_t expectedSystemId = 0x00;
static constexpr uint8_t expectedMessageId = 0x51;

static constexpr uint8_t expectedPayloadLength = 0x14;

static constexpr uint8_t headerLength = 6;
static constexpr uint8_t payloadLength = 20;
static constexpr uint8_t packetLength = 27;

    static constexpr float pixelScaling = 1.76e-3f;

    HardwareSerial* serialPort = nullptr;

    uint8_t packetBuffer[packetLength] = {};
    uint8_t packetIndex = 0;

    OpticalFlowMeasurement filteredMeasurement{};

    uint32_t previousMeasurementTime = 0;


    bool readPacket(
        OpticalFlowMeasurement& measurement
    );

    bool decodePacket(
        const uint8_t* packet,
        OpticalFlowMeasurement& measurement
    );

    void resetParser();

    float calculateDeltaTime(
        uint32_t currentTime
    ) const;

    uint8_t calculateChecksum(
    const uint8_t* packet
) const;

};