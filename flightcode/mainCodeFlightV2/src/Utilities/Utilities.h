#pragma once
#ifndef UTILITIES_H
#define UTILITIES_H

#include "Arduino.h"
struct Vector3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct NED_coordinates {
    // Position -> units are meters
    // velocity -> units are m/s ...
    float North_SI;
    float East_SI; 
    float Down_SI;
}

struct Rotation_Euler_coordinates {
    float Roll_SI = 0.0f;
    float Pitch_SI = 0.0f;
    float Yaw_SI = 0.0f;
};

template<typename T>
struct ParameterConfig
{
    T min; 
    T max;
    T default_;
};

enum class OutputType{
    SUCCESS,
    CMD,
    PROTECTED,
    PARAMETER,
    STATE,
    SAFETY,
    SENSOR,
    ACTUATOR,
    COMMS,
    TIMEOUT,
    MISC
};


struct SerialPortConfig {
    HardwareSerial* port;
    uint32_t baudrate;
    const char* name;
}






namespace Utilities{

    float constrain(float u, float u_min, float u_max);
    float clamping(float u, float u_max);
    
    float EWA(float alpha, float u, float measurement);

    bool isWithinEps_1D(float eps, float u, float u_ref);
    bool isWithinEps_2D(float eps, float x, float y, float x_ref, float u_ref_2);
    bool isWithinEps_3D(float epsH, float epsV, float x, float y, float z, float x_ref, float y_ref, float z_ref);
    bool isWithinEps_3D(float epsH, float epsV,const Vector3& p, const Vector3& p_ref);
    bool isWithinEps_NED(float epsH, float epsV, const Vector3& p, const Vector3& p_ref);
    bool isWithinEps_Yaw(float epsYaw, float yaw, float yaw_ref);

    bool isBounded(float u, float u_min, float u_max);

    float distance1D(float u, float u_ref);
    float distance2D(float x, float y, float x_ref, float y_ref);
    float distance3D(float x, float y, float z, float x_ref, float y_ref, float z_ref);
    float distance3D(const Vector3& p, const Vector3& p_ref);
    float verticalDistanceNED(const Vector3& p, const Vector3& t);
    float horizontalDistanceNED(const Vector3& p, const Vector3& t);



}
#endif
