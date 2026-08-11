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

    serialPort =
        SensorConfig::OPTICAL_FLOW.port;

    if (serialPort == nullptr)
    {
        Serial.println(
            "OPTICAL FLOW: ERROR - NULL SERIAL PORT"
        );

        setFault(
            SensorFault::CommunicationError
        );

        return false;
    }

    Serial.print(
        "OPTICAL FLOW: BAUD = "
    );

    Serial.println(
        SensorConfig::OPTICAL_FLOW.baudrate
    );

    serialPort->begin(
        SensorConfig::OPTICAL_FLOW.baudrate
    );

    initializeSensorState(
        500
    );

    resetParser();

    filteredMeasurement = {};

    previousMeasurementTime = millis();

    Serial.println(
        "OPTICAL FLOW: SERIAL INITIALIZED"
    );

    Serial.println(
        "OPTICAL FLOW: BEGIN COMPLETE"
    );

    return true;
}


void OpticalFlow::update()
{
    if (serialPort == nullptr)
    {
        setFault(
            SensorFault::CommunicationError
        );

        return;
    }

    bool receivedMeasurement = false;

    OpticalFlowMeasurement measurement{};

    while (readPacket(measurement))
    {
        receivedMeasurement = true;

        registerPacket();

        clearFaults();

        validateMeasurement(
            measurement
        );

        if (!isHealthy())
        {
            continue;
        }

        storeRawMeasurement(
            measurement
        );

        filteredMeasurement =
            measurement;
    }

    checkTimeout();

    if (!receivedMeasurement)
    {
        return;
    }
}


bool OpticalFlow::readPacket(
    OpticalFlowMeasurement& measurement
)
{
    while (serialPort->available() > 0)
    {
        const uint8_t incomingByte =
            static_cast<uint8_t>(
                serialPort->read()
            );

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

            packetBuffer[packetIndex++] =
                incomingByte;

            continue;
        }

        /*
         * Buffer protection.
         */

        if (packetIndex >= packetLength)
        {
            resetParser();

            setFault(
                SensorFault::InvalidData
            );

            continue;
        }

        packetBuffer[packetIndex++] =
            incomingByte;

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
            if (
                packetBuffer[1] !=
                expectedDeviceId
            )
            {
                resetParser();
                continue;
            }
        }

        if (packetIndex == 3)
        {
            if (
                packetBuffer[2] !=
                expectedSystemId
            )
            {
                resetParser();
                continue;
            }
        }

        if (packetIndex == 4)
        {
            if (
                packetBuffer[3] !=
                expectedMessageId
            )
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
            const uint8_t payloadLength =
                packetBuffer[5];

            if (
                payloadLength !=
                expectedPayloadLength
            )
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
            packetBuffer[
                packetLength - 1
            ];

        const uint8_t calculatedChecksum =
            calculateChecksum(
                packetBuffer
            );

        if (
            receivedChecksum !=
            calculatedChecksum
        )
        {
            resetParser();

            setFault(
                SensorFault::InvalidData
            );

            continue;
        }

        /*
         * Decode.
         */

        if (
            decodePacket(
                packetBuffer,
                measurement
            )
        )
        {
            resetParser();

            return true;
        }

        resetParser();

        setFault(
            SensorFault::InvalidData
        );
    }

    return false;
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

    const uint8_t* payload =
        &packet[6];

    /*
     * System time.
     */

    measurement.systemTimeMs =
        static_cast<uint32_t>(
            payload[0]
            |
            (
                static_cast<uint32_t>(
                    payload[1]
                ) << 8
            )
            |
            (
                static_cast<uint32_t>(
                    payload[2]
                ) << 16
            )
            |
            (
                static_cast<uint32_t>(
                    payload[3]
                ) << 24
            )
        );

    /*
     * Distance.
     */

    measurement.distanceMm =
        static_cast<uint32_t>(
            payload[4]
            |
            (
                static_cast<uint32_t>(
                    payload[5]
                ) << 8
            )
            |
            (
                static_cast<uint32_t>(
                    payload[6]
                ) << 16
            )
            |
            (
                static_cast<uint32_t>(
                    payload[7]
                ) << 24
            )
        );

    /*
     * Distance information.
     */

    measurement.distanceStrength =
        payload[8];

    measurement.distancePrecision =
        payload[9];

    measurement.distanceStatus =
        payload[10];

    /*
     * Raw optical flow velocity.
     *
     * Unit:
     *
     * cm/s @ 1m
     */

    measurement.flowVelocityX =
        static_cast<int16_t>(
            static_cast<uint16_t>(
                payload[12]
            )
            |
            (
                static_cast<uint16_t>(
                    payload[13]
                ) << 8
            )
        );

    measurement.flowVelocityY =
        static_cast<int16_t>(
            static_cast<uint16_t>(
                payload[14]
            )
            |
            (
                static_cast<uint16_t>(
                    payload[15]
                ) << 8
            )
        );

    /*
     * Optical flow status.
     */

    measurement.flowQuality =
        payload[16];

    measurement.flowStatus =
        payload[17];

    /*
     * Calculate physical velocity.
     *
     * speed(cm/s) =
     *     flow_velocity * distance(m)
     *
     * speed(m/s) =
     *     flow_velocity
     *     * distance(m)
     *     * 0.01
     */

    if (
        measurement.distanceStatus == 1 &&
        measurement.distanceMm >= 10 &&
        measurement.flowStatus == 1
    )
    {
        const float distanceM =
            static_cast<float>(
                measurement.distanceMm
            )
            * 0.001f;

        measurement.flowRateX =
            static_cast<float>(
                measurement.flowVelocityX
            )
            * distanceM
            * 0.01f;

        measurement.flowRateY =
            static_cast<float>(
                measurement.flowVelocityY
            )
            * distanceM
            * 0.01f;
    }
    else
    {
        measurement.flowRateX = 0.0f;
        measurement.flowRateY = 0.0f;
    }

    /*
     * Delta time.
     */

    const uint32_t currentTime =
        millis();

    measurement.dt =
        calculateDeltaTime(
            currentTime
        );

    previousMeasurementTime =
        currentTime;

    return true;
}


uint8_t OpticalFlow::calculateChecksum(
    const uint8_t* packet
) const
{
    uint8_t checksum = 0;

    for (
        uint8_t i = 0;
        i < packetLength - 1;
        i++
    )
    {
        checksum += packet[i];
    }

    return checksum;
}


float OpticalFlow::calculateDeltaTime(
    uint32_t currentTime
) const
{
    if (previousMeasurementTime == 0)
    {
        return 0.0f;
    }

    return
        static_cast<float>(
            currentTime -
            previousMeasurementTime
        )
        * 1.0e-3f;
}


void OpticalFlow::validateMeasurement(
    const OpticalFlowMeasurement& measurement
)
{
    /*
     * Distance status:
     *
     * 1 = valid
     * 0 = invalid
     */

    if (
        measurement.distanceStatus != 1
    )
    {
        setFault(
            SensorFault::InvalidData
        );

        return;
    }

    /*
     * MicoAir specifies:
     *
     * distance >= 10 mm
     */

    if (
        measurement.distanceMm < 10
    )
    {
        setFault(
            SensorFault::OutOfRange
        );

        return;
    }

    /*
     * Optical flow status:
     *
     * 1 = valid
     * 0 = invalid
     */

    if (
        measurement.flowStatus != 1
    )
    {
        setFault(
            SensorFault::InvalidData
        );

        return;
    }

    if (
        !isfinite(
            measurement.flowRateX
        )
        ||
        !isfinite(
            measurement.flowRateY
        )
    )
    {
        setFault(
            SensorFault::InvalidData
        );

        return;
    }

    constexpr float maxFlowRate =
        20.0f;

    if (
        fabsf(
            measurement.flowRateX
        ) > maxFlowRate
        ||
        fabsf(
            measurement.flowRateY
        ) > maxFlowRate
    )
    {
        setFault(
            SensorFault::OutOfRange
        );

        return;
    }
}


OpticalFlowMeasurement
OpticalFlow::getMeasurement() const
{
    return filteredMeasurement;
}


void OpticalFlow::zero()
{
    /*
     * Optical flow is not a biased sensor
     * in the same sense as an accelerometer.
     *
     * No zero operation required.
     */
}


void OpticalFlow::resetParser()
{
    packetIndex = 0;
}