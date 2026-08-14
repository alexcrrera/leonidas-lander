#pragma once


#ifndef VN300_PARSING_H
#define VN300_PARSING_H


#include <Arduino.h>
#include <textparser.h>

#include "VN300_data_structs.h"



namespace VN300Parsing
{
    inline TextParser commaParser(",");
    void parse_GNSS_Solution_LLA(char *payload, VN300Measurement& measurement);
    void parse_INS_Solution_LLA(char *payload, VN300Measurement& measurement);
    void parse_YPR_LinearAccel_Gyro(char *payload, VN300Measurement& measurement);
}


void VN300Parsing::parse_GNSS_Solution_LLA(char* payload, VN300Measurement& measurement)
{

   
    char header[7];

    commaParser.parseLine(
        payload,
        header,

        measurement.gnssSolution.GpsTow,
        measurement.gnssSolution.GpsWeek,
        measurement.gnssSolution.GnssFix,
        measurement.gnssSolution.NumSats,
        measurement.gnssSolution.Pad, // 4 bytes of padding

        measurement.gnssSolution.Lat,
        measurement.gnssSolution.Lon,
        measurement.gnssSolution.Alt,
        measurement.gnssSolution.VelN,
        measurement.gnssSolution.VelE,
        measurement.gnssSolution.VelD,

        measurement.gnssSolution.PosUncertaintyN,
        measurement.gnssSolution.PosUncertaintyE,
        measurement.gnssSolution.PosUncertaintyD,

        measurement.gnssSolution.GnssVelUncertainty,
        measurement.gnssSolution.GnssTimeUncertainty
    );
}


void VN300Parsing::parse_INS_Solution_LLA(char* payload, VN300Measurement& measurement){
    char header[7];



    commaParser.parseLine(payload,header,

        measurement.insSolution.GpsTow,
        measurement.insSolution.GpsWeek,
        measurement.insSolution.InsStatus,

        measurement.insSolution.Yaw,
        measurement.insSolution.Pitch,
        measurement.insSolution.Roll,

        measurement.insSolution.PosLat,
        measurement.insSolution.PosLon,
        measurement.insSolution.PosAlt,

        measurement.insSolution.VelN,
        measurement.insSolution.VelE,
        measurement.insSolution.VelD,

        measurement.insSolution.AttUncertainty,
        measurement.insSolution.PosUncertainty,
        measurement.insSolution.VelUncertainty
    );
}





void VN300Parsing::parse_YPR_LinearAccel_Gyro(char* payload, VN300Measurement& measurement  )
{
    char header[7];

    commaParser.parseLine(payload,header,

//         struct YPR_LinearAccel_Gyro {
//     float Yaw;
//     float Pitch;
//     float Roll;
//     float LinAccelN;
//     float LinAccelE;
//     float LinAccelD;
//     float GyroX;
//     float GyroY;
//     float GyroZ;
// };
        measurement.yprLinearAccelGyro.Yaw,
        measurement.yprLinearAccelGyro.Pitch,
        measurement.yprLinearAccelGyro.Roll,

        measurement.yprLinearAccelGyro.LinAccelN,
        measurement.yprLinearAccelGyro.LinAccelE,
        measurement.yprLinearAccelGyro.LinAccelD,

        measurement.yprLinearAccelGyro.GyroX,
        measurement.yprLinearAccelGyro.GyroY,
        measurement.yprLinearAccelGyro.GyroZ
    );
}



#endif // VN300_PARSING_H