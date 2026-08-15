#pragma once

#ifndef LANDER_STRUCTS_H
#define LANDER_STRUCTS_H


struct OpticalFlow_data
{
    uint32_t systemTimeMs = 0;

    uint32_t distanceMm = 0;

    uint8_t distanceStrength = 0;
    uint8_t distancePrecision = 0;
    uint8_t distanceStatus = 0;

    int16_t flowVelocityX = 0;
    int16_t flowVelocityY = 0;

    uint8_t flowQuality = 0;
    uint8_t flowStatus = 0;

    float flowRateX = 0.0f;
    float flowRateY = 0.0f;

    float dt = 0.0f;
};


struct OpticalFlowMeasurement
{
    float velocity_North_SI = 0.0f;
    float velocity_East_SI = 0.0f;

    bool velocityValid = false;
};


#endif // LANDER_STRUCTS_H