#pragma once

#include <Arduino.h>
#include <textparser.h>

#include "../Sensor/Sensor.h"

#include "VN300_utilities.h"
#include "VN300_data_structs.h"

#include "../Utilities/Utilities.h"

#include "../Config/SensorConfig.h"

#include <handlerLib.h>





class VN300 : public Sensor<VN300Measurement>
{
public:

    VN300();

    bool begin() override;
    void update() override;
  
  
  
    VN300Measurement getMeasurement() const override;

private:

    VN300Measurement measurement_data;

    //Handler INS_Solution_LLA_poll_handler;
    Handler YPR_LinearAccel_Gyro_poll_handler;
    Handler GNSS_Solution_LLA_poll_handler;


    HardwareSerial* vectornav = nullptr;
    TextParser commaParser;

    char incomingData[SensorConfig::VN300.bufferSize];

    int dataIndex = 0;

    void process_data();

    void manage_incoming_data();

 
};