
#pragma once
#include <Arduino.h>
#include "../Config/CommsConfig.h"

#include <Wire.h>

struct SerialSensorConfig {

    HardwareSerial* port;

    uint32_t baudrate;


    float frequency = -1; // means no polling

};


struct I2CSensorConfig {

    TwoWire* port;

    uint8_t address;

    float frequency=-1; // means no polling
};




namespace SensorConfig {

    constexpr SerialSensorConfig VN300 = {
        .port = &Serial8,
        .baudrate = 115200,
        .frequency = 10.0f,
    };


    constexpr I2CSensorConfig LIDAR = {
        .port = &Wire,
        .address = 0x62,
        .frequency = 100.0f
    };


    // optical flow
    constexpr SerialSensorConfig OPTICAL_FLOW = {
        .port = &Serial3,
        .baudrate = 115200,
        .frequency = -1.0f, // no polling
        
    };

}