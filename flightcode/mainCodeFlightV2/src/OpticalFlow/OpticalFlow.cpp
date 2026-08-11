#include "OpticalFlow.h"
#include <math.h>


OpticalFlow::OpticalFlow(
    HardwareSerial& serialPort,
    uint32_t baudRate,
    unsigned long sensorTimeoutMs
)
    :
    serialPort(serialPort),
    baudRate(baudRate)
{
    timeoutMs = sensorTimeoutMs;
}


bool OpticalFlow::begin()
{
    serialPort.begin(baudRate);

    initializeSensorState(timeoutMs);

    resetParser();

    filteredMeasurement = {};
    
    previousMeasurementTime = millis();

    return true;
}


void OpticalFlow::update()
{
    bool receivedMeasurement = false;

    OpticalFlowMeasurement rawMeasurement{};

    while (readPacket(rawMeasurement)) {

        receivedMeasurement = true;

        registerPacket();

        clearFaults();

        validateMeasurement(rawMeasurement);

        if (!isHealthy()) {
            continue;
        }

        storeRawMeasurement(rawMeasurement);

        filteredMeasurement = rawMeasurement;
    }

    checkTimeout();

    if (!receivedMeasurement) {
        return;
    }
}


bool OpticalFlow::readPacket(
    OpticalFlowMeasurement& measurement
)
{
    while (serialPort.available() > 0) {

        const uint8_t incomingByte =
            static_cast<uint8_t>(serialPort.read());


        if (packetIndex == 0) {

            if (incomingByte != packetHeader) {
                continue;
            }

            packetBuffer[packetIndex++] = incomingByte;

            continue;
        }


        packetBuffer[packetIndex++] = incomingByte;


        if (packetIndex < packetLength) {
            continue;
        }


        packetIndex = 0;


        if (packetBuffer[0] != packetHeader ||
            packetBuffer[1] != expectedDataLength ||
            packetBuffer[packetLength - 1] != packetFooter)
        {
            resetParser();

            setFault(SensorFault::InvalidData);

            continue;
        }


        if (decodePacket(packetBuffer, measurement)) {
            return true;
        }


        setFault(SensorFault::InvalidData);
    }


    return false;
}


bool OpticalFlow::decodePacket(
    const uint8_t* packet,
    OpticalFlowMeasurement& measurement
)
{
    /*
     * Holybro PMW3901 UART packet:
     *
     * 0: 0xFE
     * 1: 0x04
     * 2: X high
     * 3: X low
     * 4: Y high
     * 5: Y low
     * 6: checksum
     * 7: surface quality
     * 8: 0xAA
     *
     * The checksum byte is retained in the packet but is not
     * interpreted here because the published CX-OF-compatible
     * frame format does not specify a checksum algorithm.
     */

    measurement.deltaX =
        static_cast<int16_t>(
            (static_cast<uint16_t>(packet[2]) << 8) |
            static_cast<uint16_t>(packet[3])
        );


    measurement.deltaY =
        static_cast<int16_t>(
            (static_cast<uint16_t>(packet[4]) << 8) |
            static_cast<uint16_t>(packet[5])
        );


    measurement.surfaceQuality = packet[7];


    const unsigned long currentTime = millis();

    measurement.dt =
        calculateDeltaTime(currentTime);

    previousMeasurementTime = currentTime;


    if (measurement.dt > 0.0f) {

        measurement.flowRateX =
            static_cast<float>(measurement.deltaX)
            * pixelScaling
            / measurement.dt;

        measurement.flowRateY =
            static_cast<float>(measurement.deltaY)
            * pixelScaling
            / measurement.dt;
    }
    else {

        measurement.flowRateX = 0.0f;
        measurement.flowRateY = 0.0f;
    }


    return true;
}


float OpticalFlow::calculateDeltaTime(
    unsigned long currentTime
) const
{
    if (previousMeasurementTime == 0) {
        return 0.0f;
    }

    return
        static_cast<float>(
            currentTime - previousMeasurementTime
        )
        * 1.0e-3f;
}


void OpticalFlow::validateMeasurement(
    const OpticalFlowMeasurement& rawMeasurement
)
{
    if (!isfinite(rawMeasurement.flowRateX) ||
        !isfinite(rawMeasurement.flowRateY) ||
        !isfinite(rawMeasurement.dt))
    {
        setFault(SensorFault::InvalidData);

        return;
    }


    if (rawMeasurement.dt <= 0.0f ||
        rawMeasurement.dt > 0.5f)
    {
        setFault(SensorFault::InvalidData);

        return;
    }


    /*
     * The PMW3901 reports surface quality as an unsigned byte.
     * No additional range check is required.
     */

    constexpr float maxFlowRate = 20.0f;

    if (fabsf(rawMeasurement.flowRateX) > maxFlowRate ||
        fabsf(rawMeasurement.flowRateY) > maxFlowRate)
    {
        setFault(SensorFault::OutOfRange);

        return;
    }
}


OpticalFlowMeasurement OpticalFlow::getMeasurement() const
{
    return filteredMeasurement;
}


void OpticalFlow::zero()
{
    /*
     * Optical flow reports frame-to-frame displacement.
     * There is no static bias to zero in the same sense as
     * an accelerometer or gyroscope.
     */
}


void OpticalFlow::resetParser()
{
    packetIndex = 0;
}