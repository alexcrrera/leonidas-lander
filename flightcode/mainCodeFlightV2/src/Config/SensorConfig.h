
#pragma once
#include <Arduino.h>
#include "../Config/CommsConfig.h"

#include <Wire.h>

struct SensorParameters {

    float frequency = -1; // means no polling
    float EWA_alpha = 0.1f; // default exponential weighted average filter alpha

};




struct SerialSensorConfig {

    HardwareSerial* port;

    uint32_t baudrate;

  

    SensorParameters parameters;
  uint bufferSize = 180;

};


struct I2CSensorConfig {

    TwoWire* port;

    uint8_t address;

    SensorParameters parameters;
};




namespace SensorConfig {

    constexpr SerialSensorConfig VN300 = {
        .port = &Serial8,
        .baudrate = 115200,
        .parameters = {
            .frequency = 10.0f,
            .EWA_alpha = 0.1f
        },
        .bufferSize = 180
    };

    constexpr float INS_Solution_LLA_poll_frequency = 20.0f; // Hz
    constexpr float YPR_LinearAccel_Gyro_poll_frequency = 100.0f; // Hz
    constexpr float GNSS_Solution_LLA_poll_frequency = 5.0f; // Hz


    constexpr I2CSensorConfig LIDAR = {
        .port = &Wire,
        .address = 0x62,
        .parameters = {
            .frequency = 100.0f,
            .EWA_alpha = 0.1f
        }
    };


    // optical flow
    constexpr SerialSensorConfig OPTICAL_FLOW = {
        .port = &Serial3,
        .baudrate = 115200,
        .parameters = {
            .frequency = -1.0f, // no polling
            .EWA_alpha = 0.1f
        }
    };

}


static_assert(SensorConfig::INS_Solution_LLA_poll_frequency > 0.0f, "INS_Solution_LLA_poll_frequency must be greater than 0");
static_assert(SensorConfig::YPR_LinearAccel_Gyro_poll_frequency > 0.0f, "YPR_LinearAccel_Gyro_poll_frequency must be greater than 0");
static_assert(SensorConfig::GNSS_Solution_LLA_poll_frequency > 0.0f, "GNSS_Solution_LLA_poll_frequency must be greater than 0");