#pragma once



#include <Arduino.h>
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
    Fused,
    OpticalFlow
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

struct LanderState
{
    // Local NED position [m]
    NED_coordinates position;

    // NED velocity [m/s]
    NED_coordinates velocity;

    // NED linear acceleration [m/s^2]
    NED_coordinates acceleration;

    // Attitude in Euler angles [deg]
    Rotation_Euler_coordinates attitude;

    // Body angular rates [deg/s]
    Rotation_Euler_coordinates angularVelocity;

};


struct LanderSolution
{
    LanderState state;
    StateValidity validity;
    SolutionSource source;

    uint16_t errors = LANDER_ERROR_NONE;
    uint32_t timestamp = 0;
};

