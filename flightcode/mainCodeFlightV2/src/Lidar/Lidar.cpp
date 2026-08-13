#include "Lidar.h"


bool Lidar::begin()
{
    initializeSensorState(500);

    wire = SensorConfig::LIDAR.port;
    address = SensorConfig::LIDAR.address;

    if (wire == nullptr)
    {
        setFault(
            SensorFault::CommunicationError
        );

        return false;
    }


    if (SensorConfig::LIDAR.parameters.frequency > 0.0f)
    {
        setFrequency(
            SensorConfig::LIDAR.parameters.frequency
        );
    }


    measurementPending = false;
    filterInitialized = false;

    filteredDistanceM = 0.0f;
    offsetM = 0.0f;

    lastUpdateUs = micros();


    wire->begin();


    if (!checkCommunication())
    {
        setFault(
            SensorFault::CommunicationError
        );

        return false;
    }


    return true;
}


void Lidar::update()
{
    checkTimeout();

    unsigned long currentUs = micros();


    // ========================================
    // Measurement currently in progress
    // ========================================

    if (measurementPending)
    {
        LidarReadyState state =
            getMeasurementState();


        // ------------------------------------
        // Sensor disappeared during acquisition
        // ------------------------------------

        if (state == LidarReadyState::CommunicationError)
        {
            measurementPending = false;

            setFault(
                SensorFault::CommunicationError
            );

            return;
        }


        // ------------------------------------
        // Sensor still measuring
        // ------------------------------------

        if (state == LidarReadyState::Busy)
        {
            if (
                (currentUs - measurementStartUs) >
                measurementTimeoutUs
            )
            {
                measurementPending = false;

                setFault(
                    SensorFault::NoData
                );
            }

            return;
        }


        // ========================================
        // Measurement ready
        // ========================================

        float distanceM = 0.0f;

        if (!readDistance(distanceM))
        {
            measurementPending = false;

            setFault(
                SensorFault::CommunicationError
            );

            return;
        }


        measurementPending = false;

        registerPacket();


        // Communication and acquisition have
        // successfully recovered.
        clearFault(
            SensorFault::CommunicationError
        );

        clearFault(
            SensorFault::NoData
        );

        clearFault(
            SensorFault::Timeout
        );


        processMeasurement(distanceM);

        return;
    }


    // ========================================
    // Acquisition rate limiter
    // ========================================

    if (
        (currentUs - lastUpdateUs) <
        updatePeriodUs
    )
    {
        return;
    }

    lastUpdateUs = currentUs;


    // ========================================
    // Start next measurement
    // ========================================

    if (!startMeasurement())
    {
        setFault(
            SensorFault::CommunicationError
        );

        return;
    }


    // Successful I2C transaction means
    // communication has returned.
    clearFault(
        SensorFault::CommunicationError
    );


    measurementPending = true;
    measurementStartUs = currentUs;
}


bool Lidar::checkCommunication()
{
    wire->beginTransmission(address);

    return wire->endTransmission() == 0;
}


bool Lidar::startMeasurement()
{
    wire->beginTransmission(address);

    wire->write(
        regAcqCommand
    );

    // 0x04 starts acquisition with
    // receiver bias correction.
    wire->write(0x04);

    return wire->endTransmission() == 0;
}


LidarReadyState Lidar::getMeasurementState()
{
    wire->beginTransmission(address);

    wire->write(
        regStatus
    );


    if (
        wire->endTransmission(false) != 0
    )
    {
        return
            LidarReadyState::CommunicationError;
    }


    uint8_t bytesReceived =
        wire->requestFrom(
            address,
            static_cast<uint8_t>(1)
        );


    if (bytesReceived != 1)
    {
        return
            LidarReadyState::CommunicationError;
    }


    uint8_t statusRegister =
        wire->read();


    // Bit 0 = BUSY.
    if (
        (statusRegister & 0x01) != 0
    )
    {
        return
            LidarReadyState::Busy;
    }


    return
        LidarReadyState::Ready;
}


bool Lidar::readDistance(
    float& distanceM
)
{
    wire->beginTransmission(address);

    wire->write(
        regDistance
    );


    if (
        wire->endTransmission(false) != 0
    )
    {
        return false;
    }


    uint8_t bytesReceived =
        wire->requestFrom(
            address,
            static_cast<uint8_t>(2)
        );


    if (bytesReceived != 2)
    {
        return false;
    }


    uint16_t distanceCm =
        static_cast<uint16_t>(
            wire->read()
        ) << 8;


    distanceCm |=
        wire->read();


    distanceM =
        static_cast<float>(
            distanceCm
        ) / 100.0f;


    return true;
}


void Lidar::processMeasurement(
    float distanceM
)
{
    LidarMeasurement measurement;

    measurement.rawDistanceM =
        distanceM;


    // ========================================
    // Validate raw measurement
    // ========================================

    validateMeasurement(
        measurement
    );


    if (
        hasFault(SensorFault::OutOfRange) ||
        hasFault(SensorFault::InvalidData)
    )
    {
        return;
    }


    // Valid data means previous data-validation
    // faults have recovered.
    clearFault(
        SensorFault::OutOfRange
    );

    clearFault(
        SensorFault::InvalidData
    );


    // ========================================
    // Exponential weighted average
    // ========================================

    if (!filterInitialized)
    {
        filteredDistanceM =
            distanceM;

        filterInitialized =
            true;
    }
    else
    {
        filteredDistanceM =
            filterAlpha *
            distanceM
            +
            (1.0f - filterAlpha) *
            filteredDistanceM;
    }


    measurement.filteredDistanceM =
        filteredDistanceM;


    // ========================================
    // Store valid measurement
    // ========================================

    storeRawMeasurement(
        measurement
    );
}


void Lidar::validateMeasurement(
    const LidarMeasurement& rawMeasurement
)
{
    float distanceM =
        rawMeasurement.rawDistanceM;


    if (
        !isfinite(distanceM) ||
        distanceM <= 0.0f
    )
    {
        setFault(
            SensorFault::InvalidData
        );

        return;
    }


    if (
        distanceM >= maxDistanceM
    )
    {
        setFault(
            SensorFault::OutOfRange
        );

        return;
    }
}


LidarMeasurement Lidar::getMeasurement() const
{
    LidarMeasurement measurement;


    if (!hasValidMeasurement)
    {
        return measurement;
    }


    measurement.rawDistanceM =
        lastValidRawMeasurement.rawDistanceM
        - offsetM;


    measurement.filteredDistanceM =
        lastValidRawMeasurement.filteredDistanceM
        - offsetM;


    return measurement;
}


void Lidar::zero()
{
    if (!hasValidMeasurement)
    {
        return;
    }


    offsetM =
        lastValidRawMeasurement.filteredDistanceM;
}


void Lidar::setFrequency(
    float frequencyHz
)
{
    if (frequencyHz <= 0.0f)
    {
        return;
    }


    if (frequencyHz > maxFrequencyHz)
    {
        frequencyHz =
            maxFrequencyHz;
    }


    this->frequencyHz =
        frequencyHz;


    updatePeriodUs =
        static_cast<unsigned long>(
            1000000.0f /
            frequencyHz
        );
}


void Lidar::setFilterAlpha(
    float alpha
)
{
    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }


    if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }


    filterAlpha =
        alpha;
}


float Lidar::getRawDistance() const
{
    if (!hasValidMeasurement)
    {
        return 0.0f;
    }


    return
        lastValidRawMeasurement.rawDistanceM
        - offsetM;
}


float Lidar::getFilteredDistance() const
{
    if (!hasValidMeasurement)
    {
        return 0.0f;
    }


    return
        lastValidRawMeasurement.filteredDistanceM
        - offsetM;
}


float Lidar::getOffset() const
{
    return offsetM;
}