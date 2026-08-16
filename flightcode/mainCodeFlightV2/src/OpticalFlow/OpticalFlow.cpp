#include "OpticalFlow.h"

#include <math.h>

#include "../Config/SensorConfig.h"


OpticalFlow::OpticalFlow()
    : serialPort(nullptr)
{
}


bool OpticalFlow::begin()
{
    Serial.println("OPTICAL FLOW: BEGIN");

    serialPort = SensorConfig::OPTICAL_FLOW.port;

    if (serialPort == nullptr)
    {
        Serial.println("OPTICAL FLOW: ERROR - NULL SERIAL PORT");
        return false;
    }

    Serial.print("OPTICAL FLOW: BAUD = ");
    Serial.println(SensorConfig::OPTICAL_FLOW.baudrate);

    serialPort->begin(SensorConfig::OPTICAL_FLOW.baudrate);

    resetParser();

    Serial.println("OPTICAL FLOW: SERIAL INITIALIZED");
    Serial.println("OPTICAL FLOW: BEGIN COMPLETE");

    return true;
}


void OpticalFlow::update(LanderState& landerState)
{
    readPacket(landerState);
}



void OpticalFlow::projectOpticalFlowToNED(LanderState& landerState)
{
    float flowVelocityX = measurement.flowVelocityX;
    float flowVelocityY = measurement.flowVelocityY;

    float pitchRad = landerState.attitude.Pitch_SI * (M_PI / 180.0f);
    float rollRad = landerState.attitude.Roll_SI * (M_PI / 180.0f);

    float cosPitch = cos(pitchRad);
    float sinPitch = sin(pitchRad);
    float cosRoll = cos(rollRad);
    float sinRoll = sin(rollRad);

    float velocity_North_SI =
        flowVelocityX * cosPitch + flowVelocityY * sinRoll * sinPitch;

    float velocity_East_SI =
        flowVelocityY * cosRoll;

    opti_measurement.velocity_North_SI = velocity_North_SI;
    opti_measurement.velocity_East_SI = velocity_East_SI;

    if (measurement.flowStatus ==1 && measurement.flowQuality > 0)
    {
        opti_measurement.velocityValid = true;
        
    }
    else
    {
        opti_measurement.velocityValid = false;
    }
}



bool OpticalFlow::readPacket(LanderState& landerState)
{
    while (serialPort->available() > 0)
    {
        const uint8_t incomingByte = static_cast<uint8_t>(serialPort->read());

        if (packetIndex == 0)
        {
            if (incomingByte != packetHeader)
                continue;

            packetBuffer[packetIndex++] = incomingByte;
            continue;
        }

        if (packetIndex >= packetLength)
        {
            resetParser();
            continue;
        }

        packetBuffer[packetIndex++] = incomingByte;

        if (packetIndex == 2)
        {
            if (packetBuffer[1] != expectedDeviceId)
            {
                resetParser();
                continue;
            }
        }

        if (packetIndex == 3)
        {
            if (packetBuffer[2] != expectedSystemId)
            {
                resetParser();
                continue;
            }
        }

        if (packetIndex == 4)
        {
            if (packetBuffer[3] != expectedMessageId)
            {
                resetParser();
                continue;
            }
        }

        if (packetIndex == 6)
        {
            const uint8_t payloadLength = packetBuffer[5];

            if (payloadLength != expectedPayloadLength)
            {
                resetParser();
                continue;
            }
        }

        if (packetIndex < packetLength)
            continue;

        const uint8_t receivedChecksum = packetBuffer[packetLength - 1];
        const uint8_t calculatedChecksum = calculateChecksum(packetBuffer);

        if (receivedChecksum != calculatedChecksum)
        {
            resetParser();
            continue;
        }

        if (decodePacket(packetBuffer, landerState))
        {
            resetParser();
            return true;
        }

        resetParser();
    }

    return false;
}


uint8_t OpticalFlow::calculateChecksum(const uint8_t* packet) const
{
    uint8_t checksum = 0;

    for (uint8_t i = 0; i < packetLength - 1; i++)
        checksum += packet[i];

    return checksum;
}



void OpticalFlow::resetParser()
{
    packetIndex = 0;
}


bool OpticalFlow::decodePacket(const uint8_t* packet, LanderState& landerState){
    const uint8_t* payload = &packet[6];

    uint32_t systemTimeMs =
        static_cast<uint32_t>(payload[0]) |
        (static_cast<uint32_t>(payload[1]) << 8) |
        (static_cast<uint32_t>(payload[2]) << 16) |
        (static_cast<uint32_t>(payload[3]) << 24);

    uint32_t distanceMm =
        static_cast<uint32_t>(payload[4]) |
        (static_cast<uint32_t>(payload[5]) << 8) |
        (static_cast<uint32_t>(payload[6]) << 16) |
        (static_cast<uint32_t>(payload[7]) << 24);

    uint8_t strength = payload[8];
    uint8_t precision = payload[9];
    uint8_t distanceStatus = payload[10];

    int16_t flowVelocityX =
        static_cast<int16_t>(
            static_cast<uint16_t>(payload[12]) |
            (static_cast<uint16_t>(payload[13]) << 8)
        );

    int16_t flowVelocityY =
        static_cast<int16_t>(
            static_cast<uint16_t>(payload[14]) |
            (static_cast<uint16_t>(payload[15]) << 8)
        );

    uint8_t flowQuality = payload[16];
    uint8_t flowStatus = payload[17];

    measurement.systemTimeMs = systemTimeMs;
    measurement.distanceMm = distanceMm * 0.001f;
    measurement.distanceStrength = strength;
    measurement.distancePrecision = precision;
    measurement.distanceStatus = distanceStatus;
    measurement.flowVelocityX = flowVelocityX * 0.01f;
    measurement.flowVelocityY = flowVelocityY * 0.01f;
    measurement.flowQuality = flowQuality;
    measurement.flowStatus = flowStatus;

    projectOpticalFlowToNED(landerState);

    return true;
}