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

    latestMeasurement = {};


    Serial.println("OPTICAL FLOW: SERIAL INITIALIZED");
    Serial.println("OPTICAL FLOW: BEGIN COMPLETE");

    return true;
}


void OpticalFlow::update()
{
   

  
}


bool OpticalFlow::readPacket(OpticalFlowMeasurement& measurement)
{
    while (serialPort->available() > 0)
    {
        const uint8_t incomingByte =
            static_cast<uint8_t>(serialPort->read());

        /*
         * Search for Micolink STX.
         *
         * MTF-01:
         * 0xEF
         */

        if (packetIndex == 0)
        {
            if (incomingByte != packetHeader)
            {
                continue;
            }

            packetBuffer[packetIndex++] = incomingByte;

            continue;
        }

        /*
         * Buffer protection.
         */

        if (packetIndex >= packetLength)
        {
            resetParser();

            

            continue;
        }

        packetBuffer[packetIndex++] = incomingByte;

        /*
         * Header validation.
         *
         * Byte 1: Device ID
         * Byte 2: System ID
         * Byte 3: Message ID
         * Byte 4: Sequence
         * Byte 5: Payload length
         */

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

        /*
         * Payload length has arrived.
         */

        if (packetIndex == 6)
        {
            const uint8_t payloadLength = packetBuffer[5];

            if (payloadLength != expectedPayloadLength)
            {
                resetParser();
                continue;
            }
        }

        /*
         * Wait until the complete packet is received.
         *
         * 6 header bytes
         * 20 payload bytes
         * 1 checksum byte
         *
         * Total = 27 bytes
         */

        if (packetIndex < packetLength)
        {
            continue;
        }

        /*
         * Checksum.
         */

        const uint8_t receivedChecksum =
            packetBuffer[packetLength - 1];

        const uint8_t calculatedChecksum =
            calculateChecksum(packetBuffer);

        if (receivedChecksum != calculatedChecksum)
        {
            resetParser();

            continue;
        }

        /*
         * Decode.
         */

        if (decodePacket(packetBuffer, measurement))
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
    {
        checksum += packet[i];
    }

    return checksum;
}


OpticalFlowMeasurement OpticalFlow::getMeasurement() const
{
    return latestMeasurement;
}


void OpticalFlow::resetParser()
{
    packetIndex = 0;
}


bool OpticalFlow::decodePacket(
    const uint8_t* packet,
    OpticalFlowMeasurement& measurement
)
{
    /*
     * Micolink MTF-01 frame:
     *
     * 0   STX          0xEF
     * 1   Device ID   0x0F
     * 2   System ID   0x00
     * 3   Message ID  0x51
     * 4   Sequence
     * 5   Payload Len 0x14
     *
     * Payload starts at byte 6.
     *
     * Payload:
     *
     *  0-3   uint32  system time [ms]
     *  4-7   uint32  distance [mm]
     *  8     uint8   strength
     *  9     uint8   precision
     * 10     uint8   distance status
     * 11     uint8   reserved
     * 12-13  int16   flow velocity X [cm/s @ 1m]
     * 14-15  int16   flow velocity Y [cm/s @ 1m]
     * 16     uint8   flow quality
     * 17     uint8   flow status
     * 18-19  uint16  reserved
     *
     * Byte 26:
     * checksum
     */

   

    return true;
}