#include "Lidar.h"


bool Lidar::begin()
{
 

    wire = SensorConfig::LIDAR.port;
    address = SensorConfig::LIDAR.address;

    if (wire == nullptr)
    {
   
        return false;
    }


    if (SensorConfig::LIDAR.parameters.frequency > 0.0f)
    {
        setFrequency(SensorConfig::LIDAR.parameters.frequency);
    }


   
    wire->begin();


  

    return true;
}


void Lidar::update()
{
    

}


bool Lidar::checkCommunication()
{
    wire->beginTransmission(address);

    return wire->endTransmission() == 0;
}


bool Lidar::startMeasurement()
{
    wire->beginTransmission(address);

    wire->write(
        regAcqCommand
    );

    // 0x04 starts acquisition with
    // receiver bias correction.
    wire->write(0x04);

    return wire->endTransmission() == 0;
}


LidarReadyState Lidar::getMeasurementState()
{
    
return LidarReadyState::CommunicationError;
}


bool Lidar::readDistance(
    float& distanceM
)
{
   return false;
}


void Lidar::processMeasurement(
    float distanceM
)
{
}


void Lidar::validateMeasurement(
    const LidarMeasurement& rawMeasurement
)
{
}


LidarMeasurement Lidar::getMeasurement() const
{
    LidarMeasurement measurement;




    measurement.rawDistanceM = -2;


    measurement.filteredDistanceM = -2;


    return measurement;
}



void Lidar::setFrequency(
    float frequencyHz
)
{
    if (frequencyHz <= 0.0f)
    {
        return;
    }


    if (frequencyHz > maxFrequencyHz)
    {
        frequencyHz =
            maxFrequencyHz;
    }


    this->frequencyHz =
        frequencyHz;


    updatePeriodUs =
        static_cast<unsigned long>(
            1000000.0f /
            frequencyHz
        );
}


void Lidar::setFilterAlpha(
    float alpha
)
{
    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }


    if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }


    filterAlpha =
        alpha;
}


float Lidar::getRawDistance() const
{
   return -1;
}


float Lidar::getFilteredDistance() const
{
    return -1;


    
}


float Lidar::getOffset() const
{
    return  -1;
}