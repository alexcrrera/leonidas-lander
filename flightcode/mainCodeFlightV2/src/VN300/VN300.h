#pragma once

#include <Arduino.h>
#include <textparser.h>

#include "../Sensor/Sensor.h"


struct VN300Measurement
{
    // Attitude [deg] from Register 240
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;

    // Linear acceleration in inertial NED frame [m/s^2]
    float accelNorth = 0.0f;
    float accelEast = 0.0f;
    float accelDown = 0.0f;

    // Compensated angular rates in body frame [rad/s]
    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;

    // Primary GNSS solution, Register 58
    double gpsTow = 0.0;
    uint16_t gpsWeek = 0;
    uint8_t gnssFix = 0;
    uint8_t numSats = 0;

    // Geodetic position
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;

    // GNSS velocity in NED frame [m/s]
    float velocityNorth = 0.0f;
    float velocityEast = 0.0f;
    float velocityDown = 0.0f;

    // GNSS position uncertainty in NED frame [m]
    float positionUncertaintyNorth = 0.0f;
    float positionUncertaintyEast = 0.0f;
    float positionUncertaintyDown = 0.0f;

    // GNSS scalar velocity uncertainty [m/s]
    float velocityUncertainty = 0.0f;

    // GNSS time uncertainty [s]
    float timeUncertainty = 0.0f;
};


class VN300 : public Sensor<VN300Measurement>
{
public:

    VN300(HardwareSerial& serialPort);

    bool begin() override;
    void update() override;
    void zero() override;

    VN300Measurement getMeasurement() const override;

    void setFilterAlpha(float alpha);


protected:

    void validateMeasurement(
        const VN300Measurement& rawMeasurement
    ) override;


private:

    static constexpr unsigned long gnssPollPeriodMs = 100;
    static constexpr int bufferSize = 180;
    static constexpr unsigned long vn300TimeoutMs = 500;
    static constexpr float defaultFilterAlpha = 0.10f;

    unsigned long lastGnssPollMs = 0;

    HardwareSerial& vectornav;

    TextParser commaParser;

    char incomingData[bufferSize];

    String incomingDataString = "";

    int dataIndex = 0;

    VN300Measurement offset{};

    VN300Measurement filteredMeasurement{};

    bool filterInitialized = false;

    float filterAlpha = defaultFilterAlpha;

    void processVectornav();

    void checkOverflowVectornav();

    int checkHeaderVectornav();

    void pollGnssSolution();

    void parseGnssSolution();

    void applyFilter(
        const VN300Measurement& rawMeasurement
    );

    float ewa(
        float previousValue,
        float newValue
    ) const;
};
