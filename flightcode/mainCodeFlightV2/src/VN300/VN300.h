#pragma once

#include <Arduino.h>
#include <textparser.h>


#include "VN300_utilities.h"
#include "VN300_data_structs.h"

#include "../Utilities/Utilities.h"

#include "../Config/SensorConfig.h"

#include <Handler.h>





class VN300 
{
public:

    VN300();

    bool begin() ;
    void update() ;
  
  
  
    VN300Measurement& getMeasurement() ;

private:

    VN300Measurement measurement_data;

    Handler INS_Solution_LLA_poll_handler;
    //Handler YPR_LinearAccel_Gyro_poll_handler;
    Handler GNSS_Solution_LLA_poll_handler;



    HardwareSerial* vectornav = nullptr;
    TextParser commaParser;
    int bufferSize = SensorConfig::VN300.bufferSize;
    char incomingData[SensorConfig::VN300.bufferSize];

    int dataIndex = 0;
    
    void process_data();

    void manage_incoming_data();

    void poll_GNSS_Solution_LLA();
    void poll_INS_Solution_LLA();
    void poll_YPR_LinearAccel_Gyro();
    
};