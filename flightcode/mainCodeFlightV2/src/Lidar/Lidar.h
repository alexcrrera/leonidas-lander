#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "../Config/SensorConfig.h"
#include "LiDAR_utilities.h"
#include "../Lander/Lander_structs.h"


enum class MeasurementState
{
    Idle,
    Measuring,
    Reading
};

class Lidar
{
public:

    Lidar() = default;

    bool begin();
    void update(LanderState& landerState); ;

    
    LidarMeasurement& getMeasurement() { return measurement; }


private:

    void processMeasurement(LanderState& landerState);
    bool startMeasurement();
    bool readDistance();
    
    float calculateAltitude(LanderState& landerState);
    TwoWire* wire = nullptr;

    uint8_t address = SensorConfig::LIDAR.address;
    float frequencyHz = SensorConfig::LIDAR.parameters.frequency;
    uint32_t periodUS = 1000000.0f / frequencyHz;


    MeasurementState measurementState = MeasurementState::Idle;
    uint32_t measurementStartTime = 0;
    uint32_t lastMeasurementTime = 0;
   

    float EWA_alpha = SensorConfig::LIDAR.parameters.EWA_alpha;

    LidarMeasurement measurement; 
    
    
  

    float previousAltitude_M = 0.0f;
    uint32_t previousAltitudeTimeUS = 0;
    bool hasPreviousAltitude = false;
};