#pragma once

#ifndef VN300_UTILITIES_H
#define VN300_UTILITIES_H

#include "Arduino.h"


// ============================================================
// Helper functions for VN300
// ============================================================



namespace VN300Utilities
{
    inline String configureAsynchOutputType(uint type); // Returns the command string to configure the VN300 for asynchronous output of Yaw, Pitch, Roll, Linear Acceleration, and Compensated Angular Rates at the specified frequency.
    inline String configureAsynchOutputFrequency(float frequency); // Returns the command string to configure the VN300 for asynchronous output of Yaw, Pitch, Roll, Linear Acceleration, and Compensated Angular Rates at the specified frequency.
    inline String write_register_VN300_cmd(uint reg, String payload); // Returns the command string to write a value to a specific register on the VN300.
    inline String get_cmd_poll_GNSS_Solution_LLA(); // Returns the command string to poll the VN300 for GNSS Solution LLA data.
    inline String get_cmd_poll_INS_Solution_LLA(); // Returns the command string to poll the VN300 for INS Solution LLA data.
    inline String get_cmd_poll_YPR_LinearAccel_Gyro(); // Returns the command string to poll the VN300 for Yaw, Pitch, Roll, Linear Acceleration, and Compensated Angular Rates data.

    inline int getHeader(const char* incomingData); // Returns the header type of the incoming data string from the VN300.


    

}


inline int VN300Utilities::getHeader(const char* incomingData)
{
    if (strstr(incomingData, "$VNYPR") != nullptr) return 1;
    if (strstr(incomingData, "$VNYIA") != nullptr) return 240;
    if (strstr(incomingData, "$VNRRG,58,") != nullptr) return 58;

    return -1;
}

// ============================================================
// VN300 Asynchronous Output Types

inline String VN300Utilities::get_cmd_poll_GNSS_Solution_LLA()
{
    return write_register_VN300_cmd(58, ""); // As per VN300 documentation, register 58 is for polling GNSS Solution LLA data
}

inline String VN300Utilities::get_cmd_poll_INS_Solution_LLA()
{
    return write_register_VN300_cmd(63, ""); // As per VN300 documentation, register 63 is for polling INS Solution LLA data
}

inline String VN300Utilities::get_cmd_poll_YPR_LinearAccel_Gyro()
{
    return write_register_VN300_cmd(240, ""); // As per VN300 documentation, register 240 is for polling Yaw, Pitch, Roll, Linear Acceleration, and Compensated Angular Rates data
}


// ============================================================
// VN300 Register Write Command

inline String VN300Utilities::write_register_VN300_cmd(uint reg, String payload)
{
    String cmd = "$VNWRG," + String(reg) + "," + payload + "*XX";
    return cmd;
}



inline String VN300Utilities::configureAsynchOutputType(uint type=17)
{
    return(write_register_VN300_cmd(6, String(type))); // as per VN300 documentation, register 6 is for configuring asynchronous output type
}

inline String VN300Utilities::configureAsynchOutputFrequency(float frequency)
{
    return(write_register_VN300_cmd(7, String(frequency))); // as per VN300 documentation, register 7 is for configuring asynchronous output frequency
}



#endif // VN300_UTILITIES_H





/*
 * VN-300 Asynchronous Output Types
 *
 * NAME   VALUE   DESCRIPTION
 * OFF      0     Asynchronous output turned off
 * YPR      1     Yaw, Pitch, Roll: Register 8
 * QTN      2     Quaternion: Register 9
 * QMR      8     Quaternion, Magnetic, Acceleration and Angular Rates: Register 15
 * MAG     10     Magnetic Measurements: Register 17
 * ACC     11     Acceleration Measurements: Register 18
 * GYR     12     Angular Rate Measurements: Register 19
 * MAR     13     Magnetic, Acceleration and Angular Rate Measurements: Register 20
 * YMR     14     Yaw, Pitch, Roll, Magnetic, Acceleration, Angular Rate Measurements: Register 27
 * YBA     16     Yaw, Pitch, Roll, Body True Acceleration and Angular Rate Measurements: Register 239
 * YIA     17     Yaw, Pitch, Roll, Inertial True Acceleration and Angular Rate Measurements: Register 240
 * IMU     19     IMU Measurements: Register 54
 * GPS     20     GNSS Solution LLA: Register 58
 * GPE     21     GNSS Solution ECEF: Register 59
 * INS     22     INS LLA: Register 63
 * INE     23     INS ECEF: Register 64
 * ISL     28     INS LLA 2: Register 72
 * ISE     29     INS ECEF 2: Register 73
 * DTV     30     Delta Theta and Delta Velocity: Register 80
 * G2S     32     GNSS2 LLA: Register 103
 * G2E     33     GNSS2 ECEF: Register 104
 * HVE     32     Heave: Register 115
 */