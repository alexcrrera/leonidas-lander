#include "Lidar.h"


bool Lidar::begin()
{
 

    wire = SensorConfig::LIDAR.port;
    address = SensorConfig::LIDAR.address;

    if (wire == nullptr)
    {   
        return false;
    }


   
    wire->begin();

    return true;
}


void Lidar::update(LanderState& landerState)
{
    if (measurementState == MeasurementState::Idle)
    {
        if (micros() - lastMeasurementTime < periodUS)
            return;

        lastMeasurementTime = micros();

        if (startMeasurement())
        {
            measurementStartTime = lastMeasurementTime;
            measurementState = MeasurementState::Measuring;
        }

        return;
    }

    switch (measurementState)
    {
        case MeasurementState::Measuring:
            if (micros() - measurementStartTime >= 1000)
                measurementState = MeasurementState::Reading;
            break;

        case MeasurementState::Reading:
        {
         

            if (readDistance())
                processMeasurement(landerState);

            measurementState = MeasurementState::Idle;
            break;
        }

        default:
            break;
    }
}


bool Lidar::startMeasurement()
{
    wire->beginTransmission(address);

    wire->write(
        LiDAR_utilities::regAcqCommand
    );

    // 0x04 starts acquisition with
    // receiver bias correction.
    wire->write(0x04);

    return wire->endTransmission() == 0;
}




bool Lidar::readDistance()
{
    wire->beginTransmission(address);
    wire->write(LiDAR_utilities::regDistance);
    
    if (wire->endTransmission(false) != 0)
        return false;

    if (wire->requestFrom((uint8_t)address, (uint8_t)2) != 2)
    return false;

    uint16_t distance_cm = (wire->read() << 8) | wire->read();

    measurement.raw_distance_M = distance_cm * 0.01f; // Convert cm to meters

    return true;
}


void Lidar::processMeasurement(LanderState& landerState)
{
    measurement.filtered_distance_M = Utilities::EWA(
        EWA_alpha,
        measurement.filtered_distance_M,
        measurement.raw_distance_M
    );

    measurement.altitude_M = calculateAltitude(landerState);

    const uint32_t nowUS = micros();

    if (hasPreviousAltitude)
    {
        const float dt = (nowUS - previousAltitudeTimeUS) * 1e-6f;

        if (dt > 0.0f)
        {
            const float altitudeVelocity_MS =
                (measurement.altitude_M - previousAltitude_M) / dt;

            // LiDAR altitude is positive upward.
            // NED Down velocity is positive downward.

            measurement.velocity_Down_SI = Utilities::EWA(0.2,measurement.velocity_Down_SI,-altitudeVelocity_MS);
        }
    }

    previousAltitude_M = measurement.altitude_M;
    previousAltitudeTimeUS = nowUS;
    hasPreviousAltitude = true;
}


float Lidar::calculateAltitude(LanderState& landerState)
{  
    float pitchRad = landerState.attitude.Pitch_SI * (M_PI / 180.0f);
    float rollRad = landerState.attitude.Roll_SI * (M_PI / 180.0f);

    float tanPitch = tanf(pitchRad);
    float tanRoll = tanf(rollRad);

    float altitude_M = measurement.filtered_distance_M / sqrtf(
        1.0f + tanPitch * tanPitch + tanRoll * tanRoll);

    
    return altitude_M;
}



