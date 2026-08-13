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
    double GpsTow;
    uint16_t GpsWeek;
    uint16_t InsStatus;
    float Yaw;
    float Pitch;
    float Roll;
    double PosLat;
    double PosLon;
    double PosAlt;
    float VelN;
    float VelE;
    float VelD;
    float AttUncertainty;
    float PosUncertainty;
    float VelUncertainty;
};

struct YPR_LinearAccel_Gyro {
    float Yaw;
    float Pitch;
    float Roll;
    float LinAccelN;
    float LinAccelE;
    float LinAccelD;
    float GyroX;
    float GyroY;
    float GyroZ;
};





struct GNSS_Solution_LLA {
    double GpsTow;
    uint16_t GpsWeek;
    uint8_t GnssFix;
    uint8_t NumSats;
    uint8_t Pad[4];
    double Lat;
    double Lon;
    double Alt;
    float VelN;
    float VelE;
    float VelD;
    float PosUncertaintyN;
    float PosUncertaintyE;
    float PosUncertaintyD;
    float GnssVelUncertainty;
    float GnssTimeUncertainty;
};



struct VN300Measurement
{
    INS_Solution_LLA insSolution;
    YPR_LinearAccel_Gyro yprLinearAccelGyro;
    GNSS_Solution_LLA gnssSolution;
};






#endif // VN300_DATA_STRUCTS_H
