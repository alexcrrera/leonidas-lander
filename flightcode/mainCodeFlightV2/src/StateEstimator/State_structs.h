#pragma once


#ifndef STATE_STRUCTS_H
#define STATE_STRUCTS_H



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

#endif // STATE_STRUCTS_H