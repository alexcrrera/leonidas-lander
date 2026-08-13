#pragma once


#ifndef VN300_PARSING_H
#define VN300_PARSING_H


#include <Arduino.h>
#include <textparser.h>

#include "VN300_data_structs.h"



namespace VN300Parsing
{
    void parse_GNSS_Solution(char *payload, VN300Measurement& measurement);
    void parse_INS_Solution(char *payload, VN300Measurement& measurement);
    void parse_YPR_LinearAccel_Gyro(char *payload, VN300Measurement& measurement);
}


void VN300Parsing::parse_GNSS_Solution(char* payload, VN300Measurement& measurement)
{

   
    char header[7];

    commaParser.parseLine(
        payload,
        header,

        measurement.gnssSolution.gpsTow,
        measurement.gnssSolution.gpsWeek,
        measurement.gnssSolution.gnssFix,
        measurement.gnssSolution.numSats,

        measurement.gnssSolution.latitude,
        measurement.gnssSolution.longitude,
        measurement.gnssSolution.altitude,
        measurement.gnssSolution.velocityNorth,
        measurement.gnssSolution.velocityEast,
        measurement.gnssSolution.velocityDown,

        measurement.gnssSolution.positionUncertaintyNorth,
        measurement.gnssSolution.positionUncertaintyEast,
        measurement.gnssSolution.positionUncertaintyDown,

        measurement.gnssSolution.velocityUncertainty,
        measurement.gnssSolution.timeUncertainty
    );
}


void VN300Parsing::parse_INS_Solution(char* payload, VN300Measurement& measurement){
    char header[7];

    commaParser.parseLine(payload,header,

        measurement.insSolution.gpsTow,
        measurement.insSolution.gpsWeek,
        measurement.insSolution.insStatus,

        measurement.insSolution.yaw,
        measurement.insSolution.pitch,
        measurement.insSolution.roll,

        measurement.insSolution.latitude,
        measurement.insSolution.longitude,
        measurement.insSolution.altitude,

        measurement.insSolution.velocityNorth,
        measurement.insSolution.velocityEast,
        measurement.insSolution.velocityDown,

        measurement.insSolution.attitudeUncertainty,
        measurement.insSolution.positionUncertainty,
        measurement.insSolution.velocityUncertainty
    );
}





void VN300Parsing::parse_YPR_LinearAccel_Gyro(char* payload, VN300Measurement& measurement  )
{
    char header[7];

    commaParser.parseLine(payload,header,

        measurement.yaw,
        measurement.pitch,
        measurement.roll,

        measurement.linearAccelerationNorth,
        measurement.linearAccelerationEast,
        measurement.linearAccelerationDown,

        measurement.gyroX,
        measurement.gyroY,
        measurement.gyroZ
    );
}



#endif // VN300_PARSING_H