#ifndef LANDER_H
#define LANDER_H

#include <Arduino.h>

#include "../Sensor/Sensor.h"
#include "../VN300/VN300.h"
#include "../Lidar/Lidar.h"

#include "../Utilities/Utilities.h"



struct LanderState
{
    // Local NED position [m]
    NED_coordinates position;

    // NED velocity [m/s]
    NED_coordinates velocity;

    // NED linear acceleration [m/s^2]
    NED_coordinates acceleration;

    Rotation_Euler_coordinates attitude;

    // Body angular rates [rad/s]
    Rotation_Euler_coordinates angularVelocity;

    // Height above ground from LiDAR [m]
    float altitude = 0.0f;

    uint32_t timestamp = 0;
};

struct StateValidity
{
    bool positionValid = false;
    bool velocityValid = false;
    bool accelerationValid = false;
    bool attitudeValid = false;
    bool altitudeValid = false;
};

enum class AltitudeSource : uint8_t
{
    None,
    Lidar,
    VN300,
    Fused
};

enum class PositionSource : uint8_t
{
    None,
    VN300,
    Inertial,
    Fused
};

enum class VelocitySource : uint8_t
{
    None,
    VN300,
    Inertial,
    Fused
};

enum class AttitudeSource : uint8_t
{
    None,
    VN300,
    Fused
};

struct SolutionSource
{
    AltitudeSource altitude = AltitudeSource::None;
    PositionSource position = PositionSource::None;
    VelocitySource velocity = VelocitySource::None;
    AttitudeSource attitude = AttitudeSource::None;
};

enum LanderError : uint16_t
{
    LANDER_ERROR_NONE              = 0,

    LANDER_ERROR_NO_POSITION       = 1 << 0,
    LANDER_ERROR_NO_ALTITUDE       = 1 << 1,
    LANDER_ERROR_NO_ATTITUDE       = 1 << 2,
    LANDER_ERROR_NO_VELOCITY       = 1 << 3,

    LANDER_ERROR_VN300_UNAVAILABLE = 1 << 4,
    LANDER_ERROR_LIDAR_UNAVAILABLE = 1 << 5,
    LANDER_ERROR_GNSS_INVALID      = 1 << 6,

    LANDER_ERROR_SOLUTION_STALE    = 1 << 7
};

struct LanderSolution
{
    LanderState state;
    StateValidity validity;
    SolutionSource source;

    uint16_t errors = LANDER_ERROR_NONE;

    uint32_t timestamp = 0;
};

class Lander
{
public:
    Lander(VN300& vn300, Lidar& lidar);

    void update();

    const LanderSolution& getSolution() const;

    bool hasPositionSolution() const;
    bool hasVelocitySolution() const;
    bool hasAltitudeSolution() const;
    bool hasAttitudeSolution() const;

    bool hasError(LanderError error) const;
    uint16_t getErrors() const;

    void printStatus(Stream& serialPort = Serial) const;

private:
    VN300& vn300;
    Lidar& lidar;

    LanderSolution solution;

    // First valid Register 58 GNSS position defines local NED origin.
    bool positionOriginInitialized = false;
    double originLatitude = 0.0;
    double originLongitude = 0.0;
    double originAltitude = 0.0;

    void clearSolutionStatus();

    void estimateAttitude();
    void estimateAcceleration();
    void estimateVelocity();
    void estimatePosition();
    void estimateAltitude();

    bool gnssSolutionUsable(
        const VN300Measurement& measurement
    ) const;

    void initializePositionOrigin(
        const VN300Measurement& measurement
    );

    void convertLlaToLocalNed(
        const VN300Measurement& measurement,
        NED_coordinates& position
    ) const;

    void updateErrors();

    void setError(LanderError error);
};

#endif
