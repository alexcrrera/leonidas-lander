#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "../Sensor/Sensor.h"
#include "../Config/SensorConfig.h"


struct LidarMeasurement
{
    float rawDistanceM = 0.0f;
    float filteredDistanceM = 0.0f;
};


enum class LidarReadyState
{
    Ready,
    Busy,
    CommunicationError
};


class Lidar : public Sensor<LidarMeasurement>
{
public:

    Lidar() = default;

    bool begin() override;
    void update() override;
    void zero() override;

    LidarMeasurement getMeasurement() const override;

    void setFrequency(float frequencyHz);
    void setFilterAlpha(float alpha);

    float getRawDistance() const;
    float getFilteredDistance() const;
    float getOffset() const;


private:

    static constexpr uint8_t regAcqCommand = 0x00;
    static constexpr uint8_t regStatus = 0x01;
    static constexpr uint8_t regDistance = 0x8F;

    static constexpr float maxFrequencyHz = 100.0f;
    static constexpr float maxDistanceM = 20.0f;

    static constexpr unsigned long measurementTimeoutUs = 20000;


    // ========================================
    // Hardware configuration
    // ========================================

    TwoWire* wire = nullptr;

    uint8_t address = 0x62;


    // ========================================
    // Timing
    // ========================================

    float frequencyHz = 100.0f;

    unsigned long updatePeriodUs = 10000;

    unsigned long lastUpdateUs = 0;


    // ========================================
    // Filtering
    // ========================================

    float filterAlpha = 0.3f;

    float filteredDistanceM = 0.0f;

    bool filterInitialized = false;


    // ========================================
    // Zero offset
    // ========================================

    float offsetM = 0.0f;


    // ========================================
    // Acquisition
    // ========================================

    bool measurementPending = false;

    unsigned long measurementStartUs = 0;


    // ========================================
    // Internal functions
    // ========================================

    bool checkCommunication();

    bool startMeasurement();

    LidarReadyState getMeasurementState();

    bool readDistance(float& distanceM);

    void processMeasurement(float distanceM);

    void validateMeasurement(
        const LidarMeasurement& rawMeasurement
    ) override;
};