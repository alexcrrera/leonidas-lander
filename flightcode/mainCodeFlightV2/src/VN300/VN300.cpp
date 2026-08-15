#include "VN300.h"


#include <math.h>

#include "VN300_parsing.h"

#include <string.h>

VN300::VN300()
    : vectornav(nullptr),
      commaParser(",")
{
}


bool VN300::begin()
{
    Serial.println("INITIALIZING VECTORNAV");

    vectornav = SensorConfig::VN300.port;

   

    vectornav->begin(SensorConfig::VN300.baudrate);

    // Register 240:
    // Yaw / Pitch / Roll
    // Linear acceleration NED
    // Compensated angular rates
    vectornav->println(VN300Utilities::configureAsynchOutputType(17));

    // Asynchronous output frequency
    vectornav->println(VN300Utilities::configureAsynchOutputFrequency(SensorConfig::YPR_LinearAccel_Gyro_poll_frequency));


    dataIndex = 0;
    

  
    // polls solutions at the specified rates
    //YPR_LinearAccel_Gyro_poll_handler.begin(SensorConfig::YPR_LinearAccel_Gyro_poll_frequency,[this]() { poll_YPR_LinearAccel_Gyro(); }); // set to automatic output from VN300, no need to poll
   GNSS_Solution_LLA_poll_handler.begin(
    SensorConfig::GNSS_Solution_LLA_poll_frequency,
    this,
    &VN300::poll_GNSS_Solution_LLA
);

INS_Solution_LLA_poll_handler.begin(
    SensorConfig::INS_Solution_LLA_poll_frequency,
    this,
    &VN300::poll_INS_Solution_LLA
);

    return true;
}




void VN300::update()
{
    // YPR Linear Accel Gyro is set to automatic output from VN300, no need to poll
    GNSS_Solution_LLA_poll_handler.handle();
    INS_Solution_LLA_poll_handler.handle();

    manage_incoming_data(); // read one byte at a time
   

}


void VN300::process_data()
{
    int vectornavIdentity = VN300Utilities::getHeader(incomingData);

    switch (vectornavIdentity)
    {
        case 1:
        {
           break;}

        case 58: // GNS Solution LLA
        {
            VN300Parsing::parse_GNSS_Solution_LLA(incomingData, measurement_data);
            break;}

        case 63: //  INSSolution-LLA
        {
            VN300Parsing::parse_INS_Solution_LLA(incomingData, measurement_data);
            break;}


        case 240: // Yaw / Pitch / Roll, Linear Acceleration, Compensated Angular Rates
        {
            VN300Parsing::parse_YPR_LinearAccel_Gyro(incomingData, measurement_data);
            break;}

        default:
            break;
    }
}






void VN300::manage_incoming_data(){
    // reads one byte at a time from the serial port 
    // and processes it when a newline character is received

    if (vectornav->available() > 0)
    {
        const char incomingChar = vectornav->read();

        if (incomingChar == '\n')
        {
            incomingData[dataIndex] = '\0';

            process_data();

            dataIndex = 0;

            return;
        }

        if (incomingChar == '\r')
        {
            return;
        }

        if (dataIndex >= bufferSize - 1) //  overflow protection
        {
            dataIndex = 0;
        }

        incomingData[dataIndex] = incomingChar;
        dataIndex++;
    }
}




void VN300::poll_GNSS_Solution_LLA()
{
    vectornav->println(VN300Utilities::get_cmd_poll_GNSS_Solution_LLA());
}
void VN300::poll_INS_Solution_LLA()
{
    vectornav->println(VN300Utilities::get_cmd_poll_INS_Solution_LLA());
}
void VN300::poll_YPR_LinearAccel_Gyro()
{
    vectornav->println(VN300Utilities::get_cmd_poll_YPR_LinearAccel_Gyro());
}


VN300Measurement& VN300::getMeasurement()
{
    return measurement_data;
}