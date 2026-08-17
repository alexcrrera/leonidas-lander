#pragma once 

#ifndef VN300_DATA_STRUCTS_H
#define VN300_DATA_STRUCTS_H

#include "Arduino.h"

struct InsStatus
{
    uint16_t mode : 2;              // Bits 0-1: Current mode of the INS filter
    uint16_t gnssFix : 1;           // Bit 2: GNSS has a valid fix
    uint16_t resv1 : 1;             // Bit 3: Reserved
    uint16_t imuErr : 1;            // Bit 4: Gyro or accelerometer subsystem error
    uint16_t magPresErr : 1;        // Bit 5: Magnetometer or pressure subsystem error
    uint16_t gnssErr : 1;           // Bit 6: GNSS communication error or invalid PPS
    uint16_t resv2 : 1;             // Bit 7: Reserved, ignore
    uint16_t gnssHeadingIns : 1;    // Bit 8: GNSS Compass aiding INS heading solution
    uint16_t gnssCompass : 1;       // Bit 9: GNSS compass operational and reporting heading
    uint16_t reserved : 6;          // Bits 10-15: Reserved
};






struct INS_Solution_LLA {
    double GpsTow = 0.0;
    uint16_t GpsWeek = 0;
    uint16_t InsStatus = 0;
    float Yaw = 0.0f;
    float Pitch = 0.0f;
    float Roll = 0.0f;
    double PosLat = 0.0;
    double PosLon = 0.0;
    double PosAlt = 0.0;
    float VelN = 0.0f;
    float VelE = 0.0f;
    float VelD = 0.0f;
    float AttUncertainty = 0.0f;
    float PosUncertainty = 0.0f;
    float VelUncertainty = 0.0f;
};

struct YPR_LinearAccel_Gyro {
    float Yaw = 0.0f;
    float Pitch = 0.0f;
    float Roll = 0.0f;
    float LinAccelN = 0.0f;
    float LinAccelE = 0.0f;
    float LinAccelD = 0.0f;
    float GyroX = 0.0f;
    float GyroY = 0.0f;
    float GyroZ = 0.0f;
};





struct GNSS_Solution_LLA {
    double GpsTow = 0.0;
    uint16_t GpsWeek = 0;
    uint8_t GnssFix = 0;
    uint8_t NumSats = 0;
    uint8_t Pad[4] = {0, 0, 0, 0};
    double Lat = 0.0;
    double Lon = 0.0;
    double Alt = 0.0;
    float VelN = 0.0f;
    float VelE = 0.0f;
    float VelD = 0.0f;
    float PosUncertaintyN = 0.0f;
    float PosUncertaintyE = 0.0f;
    float PosUncertaintyD = 0.0f;
    float GnssVelUncertainty = 0.0f;
    float GnssTimeUncertainty = 0.0f;
};



struct VN300Measurement
{
    INS_Solution_LLA insSolution;
    YPR_LinearAccel_Gyro yprLinearAccelGyro;
    GNSS_Solution_LLA gnssSolution;
};






#endif // VN300_DATA_STRUCTS_H
