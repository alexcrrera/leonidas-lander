#pragma once

#include <Arduino.h>
#include <Wire.h>
struct LidarMeasurement
{
    float raw_distance_M = 0.0f;
    float filtered_distance_M = 0.0f;
    float offset_M = 0.0f;
    float altitude_M = 0.0f;
    float velocity_Down_SI = 0.0f;
};


namespace LiDAR_utilities
{
    static constexpr uint8_t regAcqCommand = 0x00;
    static constexpr uint8_t regStatus = 0x01;
    static constexpr uint8_t regDistance = 0x8F;
//     #define LIDAR_ADDRESS 0x62
// #define REG_ACQ_COMMAND 0x00
// #define REG_DISTANCE 0x8F
    static constexpr uint8_t LIDAR_ADDRESS = 0x62;
    static constexpr uint8_t REG_ACQ_COMMAND = 0x00;
    static constexpr uint8_t REG_DISTANCE = 0x8F;




}