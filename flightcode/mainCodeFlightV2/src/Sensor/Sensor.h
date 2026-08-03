#pragma once

#include <Arduino.h>
#include <stdint.h>


enum class SensorFault : uint16_t
{
    None               = 0,
    NotInitialized     = 1 << 0,
    NoData             = 1 << 1,
    Timeout            = 1 << 2,
    OutOfRange         = 1 << 3,
    FastDelta          = 1 << 4,
    InvalidData        = 1 << 5,
    CommunicationError = 1 << 6,
    SensorError        = 1 << 7
};


template<typename T>
class Sensor
{
public:

    virtual ~Sensor() = default;

    virtual bool begin() = 0;
    virtual void update() = 0;
    virtual void zero() = 0;

    virtual T getMeasurement() const = 0;


    const T& getLastValidRawMeasurement() const
    {
        return lastValidRawMeasurement;
    }


    bool isHealthy() const
    {
        return faults == 0;
    }


    bool hasValidData() const
    {
        return hasValidMeasurement;
    }


    bool hasContinuity() const
    {
        return measurementContinuity;
    }


    uint16_t status() const
    {
        return faults;
    }


    uint16_t latchedStatus() const
    {
        return latchedFaults;
    }


    bool hasFault(SensorFault fault) const
    {
        return
            (faults & static_cast<uint16_t>(fault)) != 0;
    }


    unsigned long getLastPacketTime() const
    {
        return lastPacketTime;
    }


    unsigned long getLastValidMeasurementTime() const
    {
        return lastValidMeasurementTime;
    }


protected:

    T lastValidRawMeasurement{};

    uint16_t faults =
        static_cast<uint16_t>(
            SensorFault::NotInitialized
        );

    uint16_t latchedFaults =
        static_cast<uint16_t>(
            SensorFault::NotInitialized
        );

    bool hasValidMeasurement = false;
    bool measurementContinuity = false;

    unsigned long lastPacketTime = 0;
    unsigned long lastValidMeasurementTime = 0;

    unsigned long timeoutMs = 500;


    virtual void validateMeasurement(
        const T& rawMeasurement
    ) = 0;


    void initializeSensorState(
        unsigned long sensorTimeoutMs
    )
    {
        timeoutMs = sensorTimeoutMs;

        lastPacketTime = millis();
        lastValidMeasurementTime = 0;

        hasValidMeasurement = false;
        measurementContinuity = false;

        clearFaults();
        clearLatchedFaults();
    }


    /*
     * Call whenever a correctly formed sensor
     * measurement has been received.
     */
    void registerPacket()
    {
        lastPacketTime = millis();
    }


    /*
     * Call only after the measurement passes all
     * sensor-specific sanity checks.
     */
    void storeRawMeasurement(
        const T& rawMeasurement
    )
    {
        lastValidRawMeasurement =
            rawMeasurement;

        hasValidMeasurement = true;
        measurementContinuity = true;

        lastValidMeasurementTime =
            millis();
    }


    /*
     * Generic timeout check.
     *
     * A timeout breaks measurement continuity,
     * meaning the next measurement must not be
     * delta-checked against an old measurement.
     */
    void checkTimeout()
    {
        if (
            (millis() - lastPacketTime) >
            timeoutMs
        )
        {
            setFault(
                SensorFault::Timeout
            );

            measurementContinuity = false;
        }
    }


    /*
     * Clear every currently active fault.
     *
     * Does not affect latched faults.
     */
    void clearFaults()
    {
        faults = 0;
    }


    /*
     * Set an active fault and latch it.
     */
    void setFault(
        SensorFault fault
    )
    {
        uint16_t faultBit =
            static_cast<uint16_t>(
                fault
            );

        faults |= faultBit;
        latchedFaults |= faultBit;
    }


    /*
     * Clear one active fault.
     *
     * The corresponding latched fault remains set.
     */
    void clearFault(
        SensorFault fault
    )
    {
        uint16_t faultBit =
            static_cast<uint16_t>(
                fault
            );

        faults &= ~faultBit;
    }


    /*
     * Clear fault history.
     *
     * Does not affect currently active faults.
     */
    void clearLatchedFaults()
    {
        latchedFaults = 0;
    }
};